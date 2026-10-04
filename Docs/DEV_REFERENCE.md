# Developer Reference — GNC & Navigation Autopilot

Offline working guide for the navigation and autopilot subsystem. Covers every file you would touch when modifying guidance logic, phase transitions, or control laws. For build/run steps see [../README.md](../README.md) and [OPERATIONS.md](OPERATIONS.md); for the phase-by-phase build history see [PROJECT.md](PROJECT.md).

---

## Quick Orientation

The autopilot is split across two codebases that talk over UDP:

```
Unity (Mac)                                  cFS (Docker)
───────────────────────────────────          ─────────────────────────────────
ChaserSensors.cs → IMU, star tracker, LIDAR  sim_io       → SimLink UDP ⇄ Software Bus
VehicleState.cs  → Rigidbody wrapper         nav          → Kalman filter: sensors → nav solution
RCSModel.cs      → 16 thruster valves        gnc_app.c    → SelectPhase(), ComputeControl()
HandController.cs→ crew sticks (keyboard)    rcs          → allocation + PWM (thruster table)
ClohessyWiltshire.cs → orbital drift         *_tbl.h      → geometry, gains, mass, thruster table
RelativeNav.cs   → TRUTH (HUD, capture only)
                      ←──────────────────────────────────
                         SIM_STATE (port 5005, every 0.2 s sim time, 116 B, raw sensors)
                      ──────────────────────────────────→
                         THRUSTER_CMD (port 5006, answers each SIM_STATE — lock-step, 100 B)
```

**Each SIM_STATE (5 Hz of sim time, lock-step with Unity's 50 Hz physics) runs one chain on the Software Bus: `nav` turns the raw sensor readings into a navigation solution, then GNC runs once on that solution. `GNC_APP_ComputeControl()` outputs the impulse it wants this cycle: P = m·Δv and L = I·Δω, body frame. The cFS `rcs` app allocates that across the thrusters with NNLS over its thruster table and converts it to per-valve on-times, honoring the minimum impulse bit and the per-cycle maximum. Unity just opens each valve for its on-time at full thrust. See [SIMLINK_ICD.md](SIMLINK_ICD.md) and [RCS_THRUSTER_REFERENCE.md](RCS_THRUSTER_REFERENCE.md).**

---

## Source File Map

### cFS Side (C)

| File | What it does | Edit when |
|------|-------------|-----------|
| `cFS/apps/gnc_app/fsw/src/gnc_app.c` | Phase state machine, control law, wakeup handler | Adding phases, changing guidance logic |
| `cFS/apps/sim_io/` | Device I/O app — owns the Unity sockets, SimLink framing, lock-step `SIM_STATE`/`THRUSTER_CMD` SB messages | Changing packet format or network topology (see `Docs/SIMLINK_ICD.md`) |
| `cFS/apps/nav/` | Navigation — CW Kalman filter (IMU-propagated, LIDAR-updated), docking-frame port position/velocity, quaternion attitude error; geometry + sensor model in `nav_cfg_tbl.c`. `nav_filter.c` is cFE-free, with a stand-alone harness in `unit-test/` | Changing sensors, the estimator, or vehicle/port geometry |
| `cFS/apps/rcs/` | RCS manager — NNLS thruster allocation + PWM (min impulse bit, per-cycle max, saturation scaling) from `rcs_thr_tbl.c` | Changing thruster geometry, thrust, pulse limits, or the allocation algorithm |
| `cFS/apps/gnc_app/fsw/src/gnc_app.h` | Internal enums, structs, constants, event IDs, prototypes | Adding new state or event IDs |
| `cFS/apps/gnc_app/fsw/inc/gnc_app_msg.h` | Public interface: command codes, HK and per-cycle STATE packets (size-asserted) | Adding a telemetry field or command (then update LC tables / ground tools that read it) |
| `cFS/sample_defs/cpu1/tables/` | Mission FDIR + recorder tables: LC watchpoints/actionpoints, SC RTS 1–4, DS recorder, HS app monitor | Changing a fault threshold, response, or what gets recorded |
| `cFS/apps/gnc_app/fsw/inc/gnc_app_tbl.h` | `GNC_ParamTbl_t` — tunable gains, mass properties, `LatVelAtPort` | Adding new gains you want in the table |
| `cFS/apps/gnc_app/fsw/tables/gnc_param_tbl.c` | Default values for the gain table | Changing defaults at compile time |

### Unity Side (C#)

| File | What it does | Edit when |
|------|-------------|-----------|
| `cFS_DockingSim/Assets/ChaserSensors.cs` | Sensor hardware: gyro, accelerometer Δv, star tracker, LIDAR, capture switch, with noise and fault injection; "Log cFS NAV table" context menu (Play mode) regenerates `nav_cfg_tbl.c` geometry. Added at runtime by `UdpTelemetrySender` | Changing a sensor, its noise or mounting |
| `cFS_DockingSim/Assets/RelativeNav.cs` | **Truth** range, closing speed, lateral offset, attitude errors — HUD, `DockingDetector` and truth log only; flight software never sees it | Changing what the HUD shows or the capture check |
| `cFS_DockingSim/Assets/RCSModel.cs` | Thruster valve hardware: `SetThrusterOnTimes()` opens each valve for its cFS on-time at full thrust; "Log cFS thruster table" context menu regenerates `rcs_thr_tbl.c` | Changing thruster layout or thrust |
| `cFS_DockingSim/Assets/HandController.cs` | Crew hand controllers (keyboard) → `Thc`/`Rhc` in every SIM_STATE; GNC flies them in MANUAL | Changing manual-flight inputs |
| `cFS_DockingSim/Assets/VehicleState.cs` | Rigidbody wrapper — mass set directly on Rigidbody; inertia auto-computed from mass + shape | Changing vehicle physical properties |
| `cFS_DockingSim/Assets/ClohessyWiltshire.cs` | Applies orbital differential gravity every FixedUpdate | Changing mean motion or orbital altitude |
| `cFS_DockingSim/Assets/ApproachCorridor.cs` | 15° cone; `inCorridor` flag, `corridorAngle` | Changing approach geometry |
| `cFS_DockingSim/Assets/DockingDetector.cs` | Latches `isDocked` when 4 thresholds met; fires `onDock` event | Changing docking contact thresholds |
| `cFS_DockingSim/Assets/RateDamping.cs` | Proportional rate-null controller (H key toggle); suppressed when cFS has authority | Changing attitude hold behavior |
| `cFS_DockingSim/Assets/ThrusterDiagnostic.cs` | F8 hardware bench test: fires thruster groups and single thrusters, logs measured vs geometry-predicted ΔV/Δω | Suspected dead/mis-canted thruster |
| `cFS_DockingSim/Assets/UdpTelemetrySender.cs` | Lock-step master: samples `ChaserSensors` + hand controllers into `SIM_STATE` each GNC cycle, waits for the answer | Cycle rate, lock-step behaviour |
| `cFS_DockingSim/Assets/UdpCommandReceiver.cs` | Validates `THRUSTER_CMD` frames; applies them via `RCSModel.SetThrusterOnTimes()` | Changing command format or timeout |

---

## Phase State Machine

Defined in `gnc_app.h` as `GNC_Phase_t`. Transitions are computed in `GNC_APP_SelectPhase()` in `gnc_app.c`.

```
              startup
                 │
                 ▼
    ┌──────────────────────┐
    │        IDLE (0)       │  No telemetry, or AbortLatch set.
    │    All thrust off.    │  Promoted to CORRECT on first valid telemetry.
    └──────────┬───────────┘
               │  first valid telemetry
               ▼
    ┌──────────────────────┐
    │     CORRECT (1)       │  Lateral offset > LatCorrectGate (1.50 m default).
    │  Station-keep axially │  Drives the port LatOffset_X/Y to zero.
    │  Kill lateral drift.  │◄──────────────────────┐
    └──────────┬───────────┘  lat offset > gate       │
               │  lat offset < LatApproachGate       │
               │  (1.00 m default)                   │
               ▼                                     │
    ┌──────────────────────┐                         │
    │     APPROACH (2)      │─────────────────────────┘
    │  Tiered proportional  │  Autonomous hold points
    │  axial closure + lat  │  (brake-distance lookahead)
    │  velocity damping.    │──────────────┐
    └──────────┬───────────┘  range ≤ HoldPt + brake_dist │
               │                            ▼
               │               ┌──────────────────────┐
               │               │       HOLD (4)        │  Ground-commanded or
               │               │  Position + velocity  │  autonomous waypoint.
               │               │  station-keep toward  │
               │               │  HoldRange_m.          │
               │               └──────────┬───────────┘
               │  GO command              │  GO command
               └──────────────────────────┘
               │  docked flag set
               ▼
    ┌──────────────────────┐
    │     DOCKED (3)        │  Contact confirmed. No thrust. Terminal state.
    └──────────────────────┘

    Any state ──ABORT──► IDLE  (AbortLatch set; GO required to release)
    Any undocked state ──hand-controller deflection──► MANUAL (5)  (crew flying; GO → CORRECT)
```

**Key transition constants** (live in `GNC_ParamTbl_t`, current defaults):

| Constant | Default | Role |
|----------|---------|------|
| `LatApproachGate` | 1.00 m | CORRECT→APPROACH trigger |
| `LatCorrectGate` | 1.50 m | APPROACH→CORRECT trigger (hysteresis) |
| `HoldPoint1_m` | 20.0 m | Outer autonomous HOLD range (0 = disabled) |
| `HoldPoint2_m` | 3.0 m | Inner autonomous HOLD range (0 = disabled) |

`SelectPhase()` also checks the Docked flag (bit 1 of telemetry Flags) and the HOLD command; those override gate logic. HOLD is sticky — only a ground `GO` or `ABORT` releases it; autonomous gate transitions never override a ground-commanded hold.

**Autonomous hold points:** each hold point fires at most once per approach sequence and is re-armed only by `ABORT`+`GO` (which implies a scenario reset). The trigger range is adjusted by a brake-distance lookahead (`v²/AxialBrakeAccel_mss`) so the vehicle actually stops near the configured waypoint rather than overshooting it. When a hold point fires — or a ground `HOLD` command is received — `GNC_APP_Data.HoldRange_m` captures the current range; this becomes the axial position-hold target (see Channel 1 below).

---

## Control Law

Implemented in `GNC_APP_ComputeControl()` in `gnc_app.c`, which runs once per NAV_SOLUTION (one per SIM_STATE, every 0.2 s of sim time). It returns the **impulse** GNC wants imparted this cycle, `{Px, Py, Pz, Lx, Ly, Lz}` in the body frame, not thrusters or durations:

```
P = VehicleMass × Δv          (N·s)
L = Inertia_kgm2[axis] × Δω   (N·m·s, per axis — roll inertia is half of pitch/yaw)
```

The cFS **RCS app** turns that into valve on-times. It runs NNLS allocation over its thruster table, then PWM: 20 ms minimum impulse bit, 0.19 s per-cycle maximum, and direction-preserving scaling when a request saturates. So GNC no longer carries thruster force, moment arms, per-group accelerations or burn-duration limits. Phase 2 removed the brake-group choice, the shared-duration rescaling and the 0.65 CORRECT-phase +Fz coupling fudge, because they all existed to compensate for GNC not knowing the geometry.

### Clohessy-Wiltshire feedforward

The predicted CW Δv for one cycle is folded into each channel's velocity error, using the +V-bar LVLH mapping (radial = +Y, along-track = −Z, cross-track = +X):

```c
a_rad   =  3n²·RelPos_Y + 2n·(−RelVel_Z)     ff_y = −a_rad·dt
a_along = −2n·RelVel_Y                        ff_z = +a_along·dt   (world az = −a_along)
a_cross = −n²·RelPos_X                        ff_x = −a_cross·dt
```

Known limitation (roadmap phase 5): the lateral feed-forward is added inside the deadbanded velocity error, so the ~5×10⁻⁵ m/s-per-cycle V-bar Coriolis term never builds up into a burn by itself.

### Channel breakdown

**Channel 1 — Axial (body Z)**

```
APPROACH:  v_target = clamp(AxialKp × range, MinCloseSpeed, cap)
             cap = MaxCloseSpeed before HoldPoint1_m fires, MaxCloseSpeed_Inner after
HOLD:      v_target = clamp(AxialHoldKp × (Range_m − HoldRange_m), ±MaxHoldSpeed)
CORRECT:   v_target = 0
Δv_z = v_target − ClosingSpeed_ms + ff_z
```

**Channel 2 — Lateral X / Y** (steers on the port-relative `LatOffset_X/Y`)

```
v_target = clamp(−Kp × LatOffset, ±MaxLatSpeed)     Kp = LatKp (CORRECT/HOLD), LatKp_Approach (APPROACH)
v_error  = v_target − RelVel + ff
Δv       = v_error ∓ deadband   if |v_error| > deadband, else 0
             deadband = LatVelDeadband_ms (CORRECT), LatVelDeadband_Approach_ms (APPROACH)
```

**Channel 3 — Attitude (pitch/yaw/roll, all active phases)**

```
omega_target = clamp(AttKp × error_rad, ±MaxAttRate)
Δω           = omega_target − AngVel
```

This channel is skipped when all three errors are inside `AttDeadband_deg` (×0.5 in APPROACH, ×2 in CORRECT) **and** no axis spins faster than `SpinThreshold_rads`.

**MANUAL — crew hand controllers** (`HandController.cs` → `Thc`/`Rhc` in SIM_STATE)

```
Δv = Thc × ManualAccel_mss × dt                  (translation: acceleration command)
Δω = Rhc × ManualRate_rads − AngVel              (rotation: rate command, rate HOLD when released)
     (0 inside ManualRateDeadband_rads, so the minimum impulse bit doesn't chatter)
```

Any stick deflection over 0.5 enters MANUAL from any undocked phase, including IDLE with the abort latch set. Only GO (→ CORRECT) or ABORT leaves it.

---

## UDP Interface (SimLink v4)

The byte-level layouts, framing, CRC and lock-step protocol are defined in **[SIMLINK_ICD.md](SIMLINK_ICD.md)**. Short version:

- Unity sends a 116-byte `SIM_STATE` frame (raw sensor readings + crew hand controllers + capture switch) to port 5005 once per GNC cycle (0.2 s of sim time), then holds that physics step.
- cFS `sim_io` validates the frame and publishes it on the Software Bus. `nav` publishes a navigation solution; `gnc_app` runs one cycle on it and publishes an impulse request; `rcs` allocates it and publishes `THRUSTER_CMD` (valve on-times); `sim_io` frames that back to Unity on port 5006 with the same Seq.
- Unity applies the command in the same physics step and continues. With no answer within 500 ms it drops to free-running, and it re-engages automatically.
- If no command arrives for 3 s, `UdpCommandReceiver` clears cFS authority and zeros the thrusters.

---

## Tunable Parameters (Parameter Tables)

**GNC** gains live in `GNC_ParamTbl_t` (`gnc_app_tbl.h`, defaults in `gnc_param_tbl.c` → `/cf/gnc_param_tbl.tbl`). `CFE_TBL_Manage` runs every cycle, so a new image can be uplinked to a running cFS. Highlights:

| Field | Default | Units | Role |
|-------|---------|-------|------|
| `AxialKp` | 0.02 | 1/s | Target closing speed = Kp × range |
| `MinCloseSpeed` / `MaxCloseSpeed` / `MaxCloseSpeed_Inner` | 0.10 / 0.30 / 0.10 | m/s | Axial speed floor, outer cap, inner cap |
| `VehicleMass` | 12000 | kg | FSW mass model (impulse = m·Δv) |
| `Inertia_kgm2` | 48000, 48000, 24000 | kg·m² | FSW inertia model per body axis (impulse = I·Δω) |
| `LatKp` / `LatKp_Approach` | 0.02 / 0.006 | 1/s | Lateral position gains |
| `MaxLatSpeed` | 0.05 | m/s | Lateral speed cap |
| `LatVelDeadband_ms` / `_Approach_ms` | 0.00035 / 0.002 | m/s | Lateral velocity deadbands |
| `AttKp` / `MaxAttRate` | 0.25 / 0.20 | (rad/s)/rad, rad/s | Attitude gain and rate cap |
| `AttDeadband_deg` / `SpinThreshold_rads` | 1.0 / 0.003 | deg, rad/s | Attitude deadband and spin override |
| `HoldPoint1_m` / `HoldPoint2_m` | 20 / 3 | m | Autonomous hold points (0 = off) |
| `AxialBrakeAccel_mss` | 0.189 | m/s² | Planning estimate for the hold-point braking lookahead only |
| `ManualAccel_mss` / `ManualRate_rads` / `ManualRateDeadband_rads` | 0.02 / 0.0175 / 0.0008 | m/s², rad/s | Crew hand-controller authority |
| `EntrySettleCycles` | 15 | cycles | CORRECT → APPROACH settle gate (integer since phase 3) |
| `LatVelAtPort` | 1 | 0/1 | Lateral channel velocity: 1 = docking-port point velocity (v_cm + ω×r), 0 = CoM velocity (pre-phase-4) |

Fault thresholds (sim link loss after 2 s, 3-cycle axial under-delivery) are not GNC parameters. They are LC watchpoints; see [Fault Protection](#fault-protection-fdir-lc--sc).

**NAV** lives in `NAV_CfgTbl_t` (`nav_tbl.h`, defaults in `nav_cfg_tbl.c` → `/cf/nav_cfg_tbl.tbl`, managed on the 1 Hz HK tick): chaser port / LIDAR mounting, ISS port pose and docking roll index, the sensor noise model, and filter tuning (process noise, 6-sigma innovation gate, 10-reject re-initialisation, the sigma limits for "relative nav VALID").

**RCS** lives in `RCS_ThrTbl_t` (`rcs_tbl.h`, defaults in `rcs_thr_tbl.c` → `/cf/rcs_thr_tbl.tbl`, managed on the 1 Hz HK tick). It holds `Thrust_N` (400), `MinOnTime_s` (0.020), `MaxOnTime_s` (0.190), and per-thruster `Pos_m`/`Dir`/`Enabled` (T00–T03 disabled).

---

## Navigation State Available to the GNC

Flight software navigates for itself (realism phase 4). Unity's `ChaserSensors` sends raw readings; the cFS `nav` app estimates the state each cycle and publishes `NAV_SOLUTION` (`nav_msg.h`):

| Group | Source | Fields |
|-------|--------|--------|
| Attitude | star tracker (gyro-propagated for up to 10 s if it drops out) | `AttQuat_L`, `AttErr_B` (rotation vector to the docking attitude, from the table's `DockedQuat_TP`) |
| Rates | gyro | `AngRate_B` |
| Translation | 6-state Kalman filter in LVLH: CW dynamics + IMU Δv, LIDAR position fixes | `CmPos_L`, `CmVel_L` (chaser CoM − ISS reference) |
| Docking port | filter + attitude + table geometry, docking frame D | `PortPos_D`, `PortVel_D` (v_cm + ω×r), `Range_m`, `ClosingSpeed_ms`, `LateralOffset_m` |
| Health | covariance and gates | `Status` (`NAV_STATUS_*`), `PosSigma_m`, `VelSigma_ms` |

GNC builds its per-cycle input (`GNC_APP_Input_t`) from that, plus the hand controllers and capture switch straight from `SIM_STATE`. It uses:
- `PortPos_D` X/Y (`LatOffset_X/Y`) for lateral position control, and `PortVel_D` X/Y (or `CmVel_L`, per `LatVelAtPort`) for the lateral velocity error
- `CmPos_L` / `CmVel_L` for the CW feed-forward
- `ClosingSpeed_ms` for axial closure, `Range_m` / `LateralOffset_m` for phase gates and the hold-point lookahead
- `AttErr_B` (as pitch/yaw/roll degrees) and `AngRate_B` for the attitude P and D terms
- `DeltaV_B` (IMU) for the actuator-health check
- `NAV_STATUS_RELNAV_VALID`: while it is clear, translation coasts and phase gates hold; attitude control continues on the star tracker and gyro

The truth equivalents (`RelativeNav.cs`) still drive the Unity HUD and the capture check, so the HUD and the ground console can disagree by the navigation error.

---

## Key Coupling Constraints

The Unity scene is the real hardware and cFS tables are flight software's model of it, so most values no longer have to match exactly. A difference is a realistic modeling error that the closed loop absorbs, not a silent bug. The rows marked **must match** are interface definitions, and a mismatch breaks the link.

| Value | cFS location | Unity location | |
|-------|-------------|----------------|--|
| SimLink frame layouts | `simlink_icd.h` (sim_io), static-asserted sizes | `SimLinkProtocol.cs` | **must match** |
| Thruster count / index order (16, T00–T15) | `RCS_ThrTbl` rows | `RCSModel.thrusterTransforms` order | **must match** |
| Thruster geometry | `rcs_thr_tbl.c` | thruster transforms + CoM override | model (regenerate via context menu) |
| Thrust 400 N | `RCS_ThrTbl.Thrust_N` | `RCSModel.thrusterForce` | model |
| Mass 12000 kg / inertia (48000, 48000, 24000) kg·m² | `ParamTbl.VehicleMass`, `Inertia_kgm2` | Rigidbody mass; `VehicleState` shape | model |
| Mean motion 0.00113 rad/s | `GNC_CW_MEAN_MOTION` in `gnc_app.h` | `ClohessyWiltshire.meanMotion` | model |
| Approach corridor half-angle 15° | `ParamTbl.ConeHalfAngle_deg` | `ApproachCorridor.coneHalfAngle` | model |

---

## Fault Protection (FDIR: LC → SC)

GNC measures, LC decides, SC responds. GNC publishes what it observes (`TlmStaleSec` in HK, `UnderDeliveryStreak` in the per-cycle STATE packet) and takes no fault action itself. LC watchpoints turn those fields into conditions, LC actionpoints combine them into flight rules, and a rule that fails starts an SC stored command sequence (RTS). The ABORT and HOLD they send are the same GNC commands the ground uses. Every threshold and response lives in a table under `cFS/sample_defs/cpu1/tables/`, so the flight rules can change without touching GNC code.

| AP | Fails when (RPN over watchpoints) | Sampled by | Response |
|----|-----------------------------------|------------|----------|
| 0 | GNC HK `TlmStaleSec ≥ 2` AND NOT `Phase == DOCKED` AND NOT `Flags & ABORT_LATCH` | SCH_LAB, 1 Hz wall clock (must run while the sim is silent) | RTS 2: GNC ABORT |
| 1 | GNC STATE `UnderDeliveryStreak ≥ 3` AND `Phase == APPROACH` | gnc_app, end of every lock-step cycle (persistence counts sim cycles) | RTS 3: GNC HOLD |

| RTS | When | Does |
|-----|------|------|
| 1 | Automatically at power-on | LC → ACTIVE (it boots DISABLED); enable RTS 2–4 (SC loads every RTS disabled) |
| 2 | LC AP 0 | GNC ABORT |
| 3 | LC AP 1 | GNC HOLD (stop closing, keep station-keeping; the ground decides GO or ABORT) |
| 4 | Ground (`gnc_cmd.py rearm`, console **RE-ARM FDIR**) | LC SET_AP_STATE(all, ACTIVE) |

- **Re-arm after every response.** A fired actionpoint goes PASSIVE so a persisting fault can't restart its RTS every sample. That also means a second fault goes unanswered until you run RTS 4.
- **Link-loss latency is about 4 s**, not 2 s. The watchpoint trips at 2 s of silence, and the 1 Hz LC sample and the SC wakeup each add up to about 1 s. In lock-step this doesn't matter physically: a silent Unity is either paused (physics stopped) or free-running with the valves closed. The abort only keeps guidance from resuming on its own.
- **Files:** `lc_def_wdt.c` (watchpoints: packet, offset via `offsetof`, comparison), `lc_def_adt.c` (actionpoints: RPN, persistence, event text, RTS), `sc_rts001-004.c` (sequences; command checksums are computed at compile time because SC validates them). All of them are cpu1-only overrides: cpu2 shares the build but has no GNC.
- **HS** (`hs_amt.c`) watches the execution counters of GNC_APP, RCS, SIM_IO, LC, SC and DS, and sends an event if one stops for 10 s. That catches a hung app, not a paused sim, because every app still wakes on its 1 Hz HK.

### Add a new fault response

1. Publish the measurement from the owning app (HK if it must be evaluated while the sim is silent, STATE if it is per-cycle).
2. Add watchpoint(s) to `lc_def_wdt.c`, using `offsetof()` on the packet struct from the app's public `*_msg.h`.
3. Add an actionpoint to `lc_def_adt.c`, and put it in a sampled range: the SCH_LAB entry covers AP 0, and `GNC_APP_LC_CYCLE_AP_FIRST/LAST` covers the per-cycle group.
4. Add or reuse an RTS. There are 4 by default (`SC_NUMBER_OF_RTS`), and RTS 1 must enable any new one.

---

## Common Tasks

### Change the axial closure rate

Edit `AxialKp`, `MaxCloseSpeed` (outer cap), and/or `MaxCloseSpeed_Inner` (inner cap, active after `HoldPoint1_m` fires) in `gnc_param_tbl.c`. Rebuild, or uplink a new table image to a running system.

### Add a new autopilot phase

1. Add the enum value to `GNC_Phase_t` in `gnc_app.h`
2. Add transition logic in `GNC_APP_SelectPhase()` in `gnc_app.c`
3. Add a case in `GNC_APP_ComputeControl()` to define the control law for that phase
4. Add an EVS event ID and string for the phase transition in `gnc_app.h`
5. Update the `PHASE_NAMES[]` array in `GNC_APP_ProcessSimState()` and the `GncPhase` doc comment in `UdpCommandReceiver.cs`

### Add a new ground command

1. Add a `_CC` constant in `gnc_app_msg.h`
2. Add a case in `GNC_APP_ProcessCmd()` in `gnc_app.c`
3. Add a new EVS event ID and string
4. Add the command to `gnc_cmd.py` (Python CCSDS sender)

### Add a sensor reading to SIM_STATE

1. Add the field to `SIMLINK_SimState_t` in `sim_io/fsw/inc/simlink_icd.h` and update its size `_Static_assert`
2. Mirror it in `SimLinkProtocol.SimState` + `BuildSimStateFrame()` (same order), and produce it in `ChaserSensors.Sample()`
3. Update `SimStateBytes` in `SimLinkProtocol.cs`, the `STATE` format in `tools/simlink_smoke.py` and the table in `Docs/SIMLINK_ICD.md` — SIM_IO rejects any frame whose size or CRC does not match
4. Use it in `nav_filter.c` (`NAV_Step`); run the harness in `apps/nav/unit-test/` after changing the estimator

### Enable / retune autonomous hold waypoints

Set `HoldPoint1_m` and/or `HoldPoint2_m` in `gnc_param_tbl.c` (0 disables a waypoint). The GNC transitions to HOLD automatically when the brake-distance-adjusted range reaches the waypoint, and captures `HoldRange_m` for axial station-keep. Send `GO` to continue approach from each waypoint. If you move `HoldPoint1_m`, consider whether `MaxCloseSpeed_Inner` should also change — it applies for the entire remainder of the approach once the outer hold fires.

### Tune the CW feedforward

The feedforward is hardcoded in `GNC_APP_ComputeControl()` using `GNC_CW_MEAN_MOTION` (a `#define` in `gnc_app.h`, not yet in the parameter table). Add it to `GNC_ParamTbl_t` in `gnc_app_tbl.h` if you want to tune it at runtime.

### Check the thruster hardware / regenerate the RCS table

With cFS stopped, press **F8** in Unity to run `ThrusterDiagnostic.cs`. It fires each thruster group and each thruster alone, then logs measured vs geometry-predicted ΔV/Δω (`[DIAG]`, with a `*** MEASURED != PREDICTED ***` flag). After moving or re-canting thrusters, run RCSModel's **"Log cFS thruster table"** context-menu item and paste the rows into `cFS/apps/rcs/fsw/tables/rcs_thr_tbl.c`. There are no GNC acceleration constants to recalibrate any more.

---

## Scenario Reset (Unity)

Press **Backspace** to reset the scenario. `ScenarioReset.cs` returns the chaser to its initial position/rotation and zeroes all velocities (and tells `ChaserSensors` not to report the teleport as an acceleration). NAV notices the jump: its LIDAR fixes fail the innovation gate for 2 s, then it re-initialises (event NAV 10) — or send `python3 gnc_cmd.py nav-reset` to do it at once. The cFS GNC phase is not reset — send `python3 gnc_cmd.py abort` then `go` if you want to restart from IDLE (this also re-arms both autonomous hold points). If an FDIR response fired during the run, also send `python3 gnc_cmd.py rearm`.

---

## Ground Command Quick Reference

```bash
python3 gnc_cmd.py go      # release startup latch → begins CORRECT phase
python3 gnc_cmd.py hold    # station-keep at current range
python3 gnc_cmd.py go      # resume from HOLD
python3 gnc_cmd.py abort   # coast immediately; inhibit guidance
python3 gnc_cmd.py noop    # heartbeat (verifies command link)
python3 gnc_cmd.py reset   # zero HK counters
python3 gnc_cmd.py rearm   # SC RTS 4: re-arm the LC fault responses after one fired
python3 gnc_cmd.py trace-on   # per-cycle "GNC #" line in the cFS console (EVS DEBUG)
python3 gnc_cmd.py trace-off
```

Sent to CI_LAB on port 1234. Every command is acknowledged by an EVS event in the cFS console.

---

## EVS Event IDs (gnc_app.h)

| EID | Name | Meaning |
|-----|------|---------|
| 1 | INIT_INF | App initialized successfully |
| 2 | WAKEUP_INF | Per-cycle `GNC #` line (DEBUG — off unless `gnc_cmd.py trace-on`; the STATE packet is the data channel) |
| 11 | PHASE_INF | Phase transition (old→new) |
| 12 | NOOP_INF | NOOP command received |
| 13 | RST_INF | RESET_COUNTERS command received |
| 14 | HOLD_INF | HOLD command received; shows current range |
| 15 | GO_INF | GO command received |
| 16 | ABORT_CRIT | ABORT command received (CRITICAL severity) |
| 17 | CMD_LEN_ERR | Malformed command (wrong length) |
| 18 | CMD_CODE_ERR | Unknown command function code |
| 19 | TBL_UPD_INF | Parameter table image activated |
| 20 | TBL_ERR | Parameter table load/access error |
| 21 | HOLDPT1_INF | Autonomous hold point 1 triggered |
| 22 | HOLDPT2_INF | Autonomous hold point 2 triggered |
| 23 | TBL_VAL_ERR | Parameter table image rejected by the validator |
| 24 | BAD_CTRL | Non-finite control output, coasting |
| 25, 26 | — | Retired in phase 3: link-loss abort and actuator anomaly are now LC events 1000 / 1001 |
| 27 | BAD_DT | NAV_SOLUTION cycle length out of range |
| 28 | LINK_INF | Sim link restored after ≥ 2 s of silence (informational) |
| 29 | MANUAL_INF | Crew hand-controller takeover |
| 30 | NAV_INF | NAV relative solution became unusable (translation coasts) or usable again |

---

## Build & Run Cheatsheet

```bash
# Start Docker dev container (from project root on Mac)
./cfs-dev.sh

# Inside container — build
make native_std.install

# Inside container — run
cd /build-native_std/exe/cpu1 && ./core-cpu1

# Mac terminal — send ground commands
python3 gnc_cmd.py go

# Unity: open cFS_DockingSim/ → Play → Scene2
# Backspace: reset scenario
# H: toggle rate damping
# W/S/A/D/Space/Ctrl/R/F/E/Q/Z/X: manual thruster control
# 1/2/3/4: switch camera
# ` (backtick): single-thruster test mode
# F8: automated thruster calibration diagnostic
```
