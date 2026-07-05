# Developer Reference — GNC & Navigation Autopilot

Offline working guide for the navigation and autopilot subsystem. Covers every file you would touch when modifying guidance logic, phase transitions, or control laws. For build/run steps see [../README.md](../README.md) and [OPERATIONS.md](OPERATIONS.md); for the phase-by-phase build history see [PROJECT.md](PROJECT.md).

---

## Quick Orientation

The autopilot is split across two codebases that talk over UDP:

```
Unity (Mac)                                  cFS (Docker)
───────────────────────────────────          ─────────────────────────────────
RelativeNav.cs   → builds nav state          gnc_app.c    → SelectPhase()
VehicleState.cs  → Rigidbody wrapper                      → ComputeControl()
RCSModel.cs      → 16 physical thrusters     gnc_app_udp.c→ SendCommand()
ThrusterAllocator.cs → wrench → thrusters    gnc_app.h    → all types/constants
ClohessyWiltshire.cs → orbital drift         gnc_app_tbl.h→ tunable gains
                      ←──────────────────────────────────
                         telemetry (port 5005, 10 Hz, 72 bytes)
                      ──────────────────────────────────→
                         wrench command (port 5006, 1 Hz, 32 bytes)
```

**GNC runs at 1 Hz inside cFS. Unity runs physics at 50 Hz. `GNC_APP_ComputeControl()` outputs a body-frame wrench `[Fx,Fy,Fz,Tx,Ty,Tz]` plus one shared burn duration; Unity's `ThrusterAllocator` maps that wrench onto individual thrusters via a pseudo-inverse, and each fires for the exact duration cFS computed — cFS never sends a stop packet.**

---

## Source File Map

### cFS Side (C)

| File | What it does | Edit when |
|------|-------------|-----------|
| `cFS/apps/gnc_app/fsw/src/gnc_app.c` | Phase state machine, control law, wakeup handler | Adding phases, changing guidance logic |
| `cFS/apps/gnc_app/fsw/src/gnc_app_udp.c` | Background telemetry recv task; `SendCommand()` (packs the wrench) | Changing packet format or network topology |
| `cFS/apps/gnc_app/fsw/src/gnc_app.h` | All enums, structs, constants, function prototypes | Adding new state, message IDs, or event IDs |
| `cFS/apps/gnc_app/fsw/inc/gnc_app_tbl.h` | `GNC_ParamTbl_t` — 24 tunable gain/physical-constant fields | Adding new gains you want in the table |
| `cFS/apps/gnc_app/fsw/tables/gnc_param_tbl.c` | Default values for the gain table | Changing defaults at compile time |

### Unity Side (C#)

| File | What it does | Edit when |
|------|-------------|-----------|
| `cFS_DockingSim/Assets/RelativeNav.cs` | Computes range, closing speed, lateral offset, scalar + per-axis attitude error | Changing what navigation data is available |
| `cFS_DockingSim/Assets/RCSModel.cs` | 16-thruster geometry (position + direction); `SetWrenchCommand()` is the cFS hook | Changing thruster layout or burn execution |
| `cFS_DockingSim/Assets/ThrusterAllocator.cs` | Builds the 6×N effectiveness matrix and its pseudo-inverse; maps a wrench to per-thruster on/off | Changing the allocation algorithm or thruster count |
| `cFS_DockingSim/Assets/VehicleState.cs` | Rigidbody wrapper — mass set directly on Rigidbody; inertia auto-computed from mass + shape | Changing vehicle physical properties |
| `cFS_DockingSim/Assets/ClohessyWiltshire.cs` | Applies orbital differential gravity every FixedUpdate | Changing mean motion or orbital altitude |
| `cFS_DockingSim/Assets/ApproachCorridor.cs` | 15° cone; `inCorridor` flag, `corridorAngle` | Changing approach geometry |
| `cFS_DockingSim/Assets/DockingDetector.cs` | Latches `isDocked` when 4 thresholds met; fires `onDock` event | Changing docking contact thresholds |
| `cFS_DockingSim/Assets/RateDamping.cs` | Proportional rate-null controller (H key toggle); suppressed when cFS has authority | Changing attitude hold behavior |
| `cFS_DockingSim/Assets/ThrusterDiagnostic.cs` | F8 automated per-group calibration (delta-V/delta-omega logging) | Re-measuring `BrakeAccel_*_mss` / `ApproachAccel_mss` |
| `cFS_DockingSim/Assets/UdpTelemetrySender.cs` | Packs and sends 72-byte telemetry struct | Adding telemetry fields |
| `cFS_DockingSim/Assets/UdpCommandReceiver.cs` | Receives 32-byte wrench command; calls `SetWrenchCommand()` | Changing command format or timeout |

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
    │  Station-keep axially │  Drives Pos_X and Pos_Y to zero.
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
```

**Key transition constants** (live in `GNC_ParamTbl_t`, current defaults):

| Constant | Default | Role |
|----------|---------|------|
| `LatApproachGate` | 1.00 m | CORRECT→APPROACH trigger |
| `LatCorrectGate` | 1.50 m | APPROACH→CORRECT trigger (hysteresis) |
| `HoldPoint1_m` | 20.0 m | Outer autonomous HOLD range (0 = disabled) |
| `HoldPoint2_m` | 3.0 m | Inner autonomous HOLD range (0 = disabled) |

`SelectPhase()` also checks the Docked flag (bit 1 of telemetry Flags) and the HOLD command; those override gate logic. HOLD is sticky — only a ground `GO` or `ABORT` releases it; autonomous gate transitions never override a ground-commanded hold.

**Autonomous hold points:** each hold point fires at most once per approach sequence and is re-armed only by `ABORT`+`GO` (which implies a scenario reset). The trigger range is adjusted by a brake-distance lookahead (`v²/BrakeAccel_Hard_mss`) so the vehicle actually stops near the configured waypoint rather than overshooting it before the next 1 Hz cycle. When a hold point fires — or a ground `HOLD` command is received — `GNC_APP_Data.HoldRange_m` captures the current range; this becomes the axial position-hold target (see Channel 1 below).

---

## Control Law

Implemented in `GNC_APP_ComputeControl()` in `gnc_app.c`. Called once per 1 Hz wakeup. Returns a body-frame wrench `{Fx, Fy, Fz, Tx, Ty, Tz, duration_s}` — not a thruster bitmask.

### Fundamental math

```
thruster_accel = ThrusterForce / VehicleMass  = 400.0 N / 4500.0 kg ≈ 0.089 m/s²
burn_duration  = |velocity_error| / thruster_accel
```

Duration is capped: `MinBurnDuration (0.05 s)` ≤ duration ≤ `MaxBurnDuration (0.95 s)`. Commands below the minimum are discarded (this is the effective dead-band). Translational and attitude channels can each want a different duration; whichever is longer sets the shared `duration_s`, and the *other* group's force/torque components are scaled down proportionally so the delivered impulse stays correct.

### Clohessy-Wiltshire feedforward

Before computing velocity errors, the control law computes predicted CW differential acceleration and folds it into the velocity error on each axis, canceling the orbital drift `ClohessyWiltshire.cs` applies every Unity `FixedUpdate`:

```c
n = GNC_CW_MEAN_MOTION  // 0.00113 rad/s (ISS altitude)

ff_x = -(3*n²*Pos_X + 2*n*Vel_Y)    // radial feedforward
ff_y =  2*n*Vel_X                    // along-track feedforward
ff_z =  n²*Pos_Z                     // cross-track feedforward
```

Without this the vehicle accumulates a persistent drift that the proportional terms alone never fully cancel.

### Channel breakdown

**Channel 1 — Axial (Z, along docking axis)**

```
APPROACH:  v_target = clamp(AxialKp * range, MinCloseSpeed, cap)
             cap = MaxCloseSpeed (0.30 m/s) before HoldPoint1_m has fired,
                   MaxCloseSpeed_Inner (0.10 m/s) after it fires — models the
                   real-world outer/inner closing-rate profile.
HOLD:      v_target = clamp(AxialHoldKp * (Range_m - HoldRange_m), ±MaxHoldSpeed)
             Position + velocity feedback toward the range captured at HOLD entry —
             corrects accumulated range drift instead of just damping velocity.
CORRECT:   v_target = 0   (pure axial station-keep while driving onto the axis)

v_error  = v_target - ClosingSpeed_ms + ff_z

if v_error > 0:  fire +Z (toward target), Fz += ThrusterForce,     dur = v_error / ApproachAccel_mss
if v_error < 0:  fire −Z (away, braking), Fz -= brake_force,       dur = |v_error| / brake_accel
                   brake_accel = BrakeAccel_Hard_mss  if |v_error| > MaxCloseSpeed/2
                               = BrakeAccel_Light_mss otherwise
```

**Channel 2 — Lateral X / Y**

```
APPROACH:            v_target = 0                          (velocity-damp only; no position pull)
CORRECT / HOLD:      v_target = clamp(-LatKp * Pos_X, ±MaxLatSpeed)

v_error = v_target - Vel_X + ff_x
if |v_error| > LatVelDeadband_ms:  fire ±X,  Fx ±= ThrusterForce,  dur = (|v_error| - deadband) / accel
```

Same structure on Y with `Pos_Y`/`Vel_Y`/`ff_y`. The velocity deadband exists because without it, a 400 N impulse at 1 Hz overshoots the target velocity every cycle and the correction alternates sign (bang-bang chatter).

**Lateral→axial coupling feedforward (CORRECT only):** `Fz += 0.4 * (|Fx| + |Fy|)`. Firing a lateral burn disturbs attitude, and the resulting braking-thruster correction couples a stronger-than-expected `−Z` force back in; this proactive term fires compensating `+Z` in the same cycle rather than one cycle late.

**Channel 3 — Attitude PD (pitch/yaw/roll, all active phases)**

```
omega_target = clamp(AttKp * error_rad, ±MaxAttRate)
omega_error  = omega_target - AngVel

dur = |omega_error| / RotAccel   → fires the corresponding torque direction (Tx/Ty/Tz)
```

Skipped entirely when all three axis errors are within a deadband **and** the vehicle isn't spinning (>0.01 rad/s on any axis). The deadband is `AttDeadband_deg` scaled by phase: 0.5× in APPROACH (tighter, for port precision), 2× in CORRECT (wider, so small lateral-burn-induced tilts coast instead of triggering a fight between channels), 1× in HOLD.

---

## UDP Interface

### Telemetry packet — Unity → cFS (72 bytes, little-endian, port 5005, 10 Hz)

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0–3 | MET_s | float | Mission elapsed time |
| 4–7 | Range_m | float | Port-to-port distance |
| 8–11 | ClosingSpeed_ms | float | Positive = approaching |
| 12–15 | LateralOffset_m | float | Perpendicular from docking axis |
| 16–19 | AttitudeError_deg | float | Scalar port alignment angle |
| 20–31 | Pos_X/Y/Z | float×3 | Chaser position (LVLH) |
| 32–43 | Vel_X/Y/Z | float×3 | Chaser velocity (LVLH) |
| 44–55 | AngVel_X/Y/Z | float×3 | Chaser angular velocity (rad/s) |
| 56–59 | Flags | int32 | bit0=InCorridor, bit1=Docked |
| 60–63 | PitchError_deg | float | Per-axis attitude error, [-180, 180] |
| 64–67 | YawError_deg | float | |
| 68–71 | RollError_deg | float | |

### Wrench command packet — cFS → Unity (32 bytes, little-endian, port 5006, 1 Hz)

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0–3 | Fx | float | N, body-frame force |
| 4–7 | Fy | float | N |
| 8–11 | Fz | float | N |
| 12–15 | Tx | float | N·m, body-frame torque |
| 16–19 | Ty | float | N·m |
| 20–23 | Tz | float | N·m |
| 24–27 | Duration_s | float | Seconds each fired thruster fires |
| 28–31 | GncPhase | int32 | `GNC_Phase_t` value (0=IDLE, 1=CORRECT, 2=APPROACH, 3=DOCKED, 4=HOLD) |

A zero wrench (all six components 0.0) with any duration is a coast/heartbeat command. Unity's `ThrusterAllocator` pseudo-inverse maps the wrench to the 16 physical thrusters, binary on/off per thruster (Draco thrusters are full-thrust-or-off, not throttleable).

**Timeout:** If Unity receives no command for 1.5 s, `UdpCommandReceiver.cs` calls `ClearExternalControl()` and keyboard input resumes. This is a safety fallback, not a normal operating mode.

---

## Tunable Parameters (Parameter Table)

All gains live in `GNC_ParamTbl_t` (defined in `gnc_app_tbl.h`, 24 floats / 96 bytes). Defaults are in `gnc_param_tbl.c` and compiled to `/cf/gnc_param_tbl.tbl`. The `ProcessWakeup` loop calls `CFE_TBL_Manage` every second; you can uplink a new table image to a running cFS without restart.

| Field | Default | Units | Role |
|-------|---------|-------|------|
| `AxialKp` | 0.02 | 1/s | target closing speed = Kp × range |
| `MinCloseSpeed` | 0.10 | m/s | Floor on axial closure rate — holds a constant soft-capture speed instead of tapering to zero |
| `MaxCloseSpeed` | 0.30 | m/s | Outer axial closure cap, before HoldPoint1_m fires |
| `MaxCloseSpeed_Inner` | 0.10 | m/s | Inner axial closure cap, after HoldPoint1_m fires |
| `ThrusterForce` | 400.0 | N | Must match `RCSModel.thrusterForce` |
| `VehicleMass` | 4500.0 | kg | Must match Unity Rigidbody mass |
| `RotAccel` | 0.033 | rad/s² | Empirical rotation authority |
| `MinBurnDuration` | 0.050 | s | Discard burns shorter than this |
| `MaxBurnDuration` | 0.950 | s | Cap all burns at this |
| `LatKp` | 0.02 | 1/s | lateral speed = Kp × position error (CORRECT/HOLD) |
| `MaxLatSpeed` | 0.05 | m/s | Cap on lateral correction speed |
| `LatApproachGate` | 1.00 | m | CORRECT→APPROACH threshold |
| `LatCorrectGate` | 1.50 | m | APPROACH→CORRECT threshold (hysteresis) |
| `HoldPoint1_m` | 20.0 | m | Outer autonomous HOLD (0 = disabled) |
| `HoldPoint2_m` | 3.0 | m | Inner autonomous HOLD (0 = disabled) |
| `AttKp` | 0.25 | (rad/s)/rad | Attitude proportional gain |
| `MaxAttRate` | 0.20 | rad/s | Cap on commanded angular rate per axis |
| `AttDeadband_deg` | 2.0 | deg | Skip attitude correction below this (scaled ×0.5/×2 by phase) |
| `LatVelDeadband_ms` | 0.015 | m/s | Skip lateral correction below this velocity error |
| `BrakeAccel_Hard_mss` | 0.281 | m/s² | Empirical decel, T08–T15 (hard stop) |
| `BrakeAccel_Light_mss` | 0.136 | m/s² | Empirical decel, T08–T11 only (soft correct) |
| `ApproachAccel_mss` | 0.163 | m/s² | Empirical accel, T04–T07 approach group |
| `AxialHoldKp` | 0.02 | 1/s | HOLD axial position gain: target speed = Kp × (Range_m − HoldRange_m) |
| `MaxHoldSpeed` | 0.05 | m/s | Cap on HOLD-phase axial position-correction speed |

**Coupling warning:** `ThrusterForce` and `VehicleMass` must exactly match Unity's `RCSModel.thrusterForce` and the Rigidbody mass. A mismatch makes computed burn durations wrong, causing overshoot or drift accumulation. `BrakeAccel_*`/`ApproachAccel_mss` are empirical — re-measure with `ThrusterDiagnostic.cs` (F8) rather than recomputing from `ThrusterForce`/`VehicleMass` if thruster geometry changes, since the binary on/off thruster model at a 1 Hz discrete loop doesn't match the theoretical continuous-thrust value.

---

## Navigation State Available to the GNC

`RelativeNav.cs` computes these each `FixedUpdate` and packs them into the telemetry packet:

| Property | How computed | Available in telemetry as |
|----------|-------------|--------------------------|
| `range` | `(chaserPort.position - targetPort.position).magnitude` | `Range_m` |
| `closingSpeed` | `−dot(relativeVelocity, approachAxis)` | `ClosingSpeed_ms` |
| `lateralOffset` | perpendicular distance from docking axis | `LateralOffset_m` |
| `attitudeError` | `Quaternion.Angle` between port forward vectors | `AttitudeError_deg` |
| `pitchError` / `yawError` / `rollError` | per-axis decomposition of the port-to-port error quaternion | `PitchError_deg` / `YawError_deg` / `RollError_deg` |

Full 6-DOF state (position, velocity, angular velocity) is also in the packet. cFS currently uses:
- `Pos_X`, `Pos_Y` for lateral position control (CORRECT/HOLD)
- `Vel_X`, `Vel_Y`, `Vel_Z` for feedforward and error computation
- `AngVel_X/Y/Z` for the attitude D-term
- `ClosingSpeed_ms` for axial closure control (smoother than differencing range)
- `PitchError_deg`/`YawError_deg`/`RollError_deg` for the attitude P-term

`Range_m` and `LateralOffset_m` are used for phase gate logic and hold-point lookahead, not directly in the axial/lateral control law.

---

## Key Coupling Constraints

These values must be consistent across both codebases. A mismatch causes silent physics errors that are hard to debug:

| Value | cFS location | Unity location |
|-------|-------------|----------------|
| Thruster force 400 N | `ParamTbl.ThrusterForce` | `RCSModel.thrusterForce` |
| Vehicle mass 4500 kg | `ParamTbl.VehicleMass` | Rigidbody mass (Inspector) |
| Moment arm 1.5 m | `GNC_RCS_MOMENT_ARM` in `gnc_app.h` | `RCSModel` thruster position vectors |
| Mean motion 0.00113 rad/s | `GNC_CW_MEAN_MOTION` in `gnc_app.h` | `ClohessyWiltshire.meanMotion` |
| Brake threshold | `BrakeAccel_Hard_mss` / `BrakeAccel_Light_mss` × `VehicleMass` | `RCSModel.SoftBrakeThreshold_N` (938 N) |
| Telemetry packet layout | `GNC_APP_UnityTlm_t` struct in `gnc_app.h` | `UdpTelemetrySender.BuildPacket()` |
| Command packet layout | `GNC_APP_SendCommand()` in `gnc_app_udp.c` | `UdpCommandReceiver.Update()` |

---

## Common Tasks

### Change the axial closure rate

Edit `AxialKp`, `MaxCloseSpeed` (outer cap), and/or `MaxCloseSpeed_Inner` (inner cap, active after `HoldPoint1_m` fires) in `gnc_param_tbl.c`. Rebuild, or uplink a new table image to a running system.

### Add a new autopilot phase

1. Add the enum value to `GNC_Phase_t` in `gnc_app.h`
2. Add transition logic in `GNC_APP_SelectPhase()` in `gnc_app.c`
3. Add a case in `GNC_APP_ComputeControl()` to define the control law for that phase
4. Add an EVS event ID and string for the phase transition in `gnc_app.h`
5. Update the `PHASE_NAMES[]` array in `GNC_APP_ProcessWakeup()` and the `GncPhase` doc comment in `UdpCommandReceiver.cs`

### Add a new ground command

1. Add a `_CC` constant in `gnc_app.h`
2. Add a case in `GNC_APP_ProcessCmd()` in `gnc_app.c`
3. Add a new EVS event ID and string
4. Add the command to `gnc_cmd.py` (Python CCSDS sender)

### Add a new telemetry field to the nav packet

1. Add the field to `GNC_APP_UnityTlm_t` in `gnc_app.h` (keep it `__attribute__((packed))`)
2. Update `UdpTelemetrySender.BuildPacket()` in Unity to write the new field at the correct byte offset
3. Update the expected packet size check in `GNC_APP_UdpRecvTask()` (`gnc_app_udp.c`) if the total size changes
4. Update `TelemetryLogger.cs` CSV columns if you want it logged

### Enable / retune autonomous hold waypoints

Set `HoldPoint1_m` and/or `HoldPoint2_m` in `gnc_param_tbl.c` (0 disables a waypoint). The GNC transitions to HOLD automatically when the brake-distance-adjusted range reaches the waypoint, and captures `HoldRange_m` for axial station-keep. Send `GO` to continue approach from each waypoint. If you move `HoldPoint1_m`, consider whether `MaxCloseSpeed_Inner` should also change — it applies for the entire remainder of the approach once the outer hold fires.

### Tune the CW feedforward

The feedforward is hardcoded in `GNC_APP_ComputeControl()` using `GNC_CW_MEAN_MOTION` (a `#define` in `gnc_app.h`, not yet in the parameter table). Add it to `GNC_ParamTbl_t` in `gnc_app_tbl.h` if you want to tune it at runtime.

### Recalibrate empirical acceleration constants

Run cFS with guidance in ABORT/IDLE, press **F8** in Unity to run `ThrusterDiagnostic.cs`, and read the delta-V / delta-omega it logs (prefixed `[DIAG]`) for each thruster group. Update `ApproachAccel_mss`, `BrakeAccel_Hard_mss`, `BrakeAccel_Light_mss`, and `RotAccel` in `gnc_param_tbl.c` to match.

---

## Scenario Reset (Unity)

Press **Backspace** to reset the scenario. `ScenarioReset.cs` returns the chaser to its initial position/rotation and zeroes all velocities. The cFS GNC phase is not reset — send `python3 gnc_cmd.py abort` then `go` if you want to restart from IDLE (this also re-arms both autonomous hold points).

---

## Ground Command Quick Reference

```bash
python3 gnc_cmd.py go      # release startup latch → begins CORRECT phase
python3 gnc_cmd.py hold    # station-keep at current range
python3 gnc_cmd.py go      # resume from HOLD
python3 gnc_cmd.py abort   # coast immediately; inhibit guidance
python3 gnc_cmd.py noop    # heartbeat (verifies command link)
python3 gnc_cmd.py reset   # zero HK counters
```

Sent to CI_LAB on port 1234. Every command is acknowledged by an EVS event in the cFS console.

---

## EVS Event IDs (gnc_app.h)

| EID | Name | Meaning |
|-----|------|---------|
| 1 | INIT_INF | App initialized successfully |
| 2 | WAKEUP_INF | 1 Hz wakeup log (phase, range, speed, lateral, attitude errors, F/T, duration) |
| 5 | UDP_INIT_INF | Telemetry recv socket bound on port 5005 |
| 9 | CMD_INIT_INF | Command send socket ready; shows resolved Docker IP |
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
