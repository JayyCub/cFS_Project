# DockingSim — Project Reference

## Goal

Build a physically accurate spacecraft docking simulator in Unity as a learning foundation for NASA's Core Flight System (cFS). The end state is a closed-loop hardware-in-the-loop system: Unity runs the physics and renders the scene, cFS runs the GNC (Guidance, Navigation & Control) logic, and the two talk over UDP in real time.

This is a learning project. Every design decision is made to mirror how real flight software works, not to take shortcuts.

---

## Repository Layout

```
cFS_Project/
├── cfs-dev.sh                    ← start Docker dev container (ports 5005 + 1234)
├── gnc_cmd.py                    ← ground command sender (NOOP/HOLD/GO/ABORT/RESET)
├── OPERATIONS.md                 ← step-by-step run guide
├── cFS/                          ← nasa/cFS clone (git clone --recurse-submodules)
│   ├── apps/
│   │   └── gnc_app/              ← custom GNC application
│   │       ├── CMakeLists.txt
│   │       └── fsw/
│   │           ├── inc/
│   │           │   ├── gnc_app_msgids.h
│   │           │   └── gnc_app_tbl.h     ← GNC_ParamTbl_t struct
│   │           ├── src/
│   │           │   ├── gnc_app.h
│   │           │   ├── gnc_app.c
│   │           │   └── gnc_app_udp.c
│   │           └── tables/
│   │               └── gnc_param_tbl.c   ← default gain table
│   └── sample_defs/
│       └── tables/
│           ├── sch_lab_table.c   ← 10 Hz tick, GNC wakeup at 1 Hz
│           └── lc_def_adt-test.c ← LC actionpoints firing ABORT RTS
└── cFS_DockingSim/               ← Unity 6 project
    └── Assets/
        ├── VehicleState.cs
        ├── RCSModel.cs
        ├── ClohessyWiltshire.cs
        ├── RelativeNav.cs
        ├── ApproachCorridor.cs
        ├── DockingDetector.cs
        ├── DockingHUD.cs
        ├── DockingCamera.cs
        ├── RateDamping.cs
        ├── ScenarioReset.cs
        ├── TelemetryLogger.cs
        ├── UdpTelemetrySender.cs
        └── UdpCommandReceiver.cs
```

---

## Phases

### Phase 1 — Physics Foundation (complete)
Establish realistic 6-DOF spacecraft dynamics inside Unity.

- `VehicleState.cs` — central interface for all scripts; exposes position, velocity, attitude, angular velocity, mass, inertia tensor. Applies `massOverride` and `inertiaTensorOverride` at Start so the Rigidbody has realistic mass properties (200 kg, I = 80 kg·m² per axis).
- `RCSModel.cs` — 12-thruster model. Bits 0–5 = translation (+X/-X/+Y/-Y/+Z/-Z), bits 6–11 = rotation (+pitch/-pitch/+yaw/-yaw/+roll/-roll). Torques applied in **body frame** using `transform.right/up/forward`. Keyboard maps to bitmask; `SetThrusterCommand(int mask, float duration)` is the cFS integration hook. Unity auto-cuts the thruster when `Time.fixedTime >= burnEndTime`, so cFS never sends a stop packet.
- `ClohessyWiltshire.cs` — Clohessy-Wiltshire orbital mechanics equations apply a differential gravity force to the chaser each FixedUpdate, producing realistic relative-motion drift when no thrusters fire. World axes approximate LVLH because the target is kinematic/stationary. Mean motion n = 0.00113 rad/s (ISS LEO).
- `SimManager.cs` — throttled console logging of relative state.

Key physics decisions:
- Both vehicle BoxColliders set to `Is Trigger = true` — prevents bounce/tumble on contact; docking is detected in software, not by physics collision response.
- Rigidbody inertia set explicitly via `inertiaTensorOverride` on VehicleState; Unity's auto-computed inertia from collider geometry is not used.
- **Vehicle mass is 200 kg and thruster force is 10 N** — these values must match `GNC_VEHICLE_MASS` and `GNC_THRUSTER_FORCE` in `gnc_app.h` exactly, or the GNC will systematically under- or over-shoot every burn.

### Phase 2 — Docking Infrastructure (complete)
Define what "docked" means and visualize the approach.

- `RelativeNav.cs` — computes approach state every FixedUpdate from port Transform positions and Rigidbody velocities: `range`, `closingSpeed` (positive = approaching), `lateralOffset`, `attitudeError` (0° = ports perfectly anti-parallel/ready to dock). Also computes per-axis attitude errors `pitchError`, `yawError`, `rollError` by decomposing the error quaternion from chaserPort to the target docking orientation.
- `DockingDetector.cs` — monitors RelativeNav; fires `UnityEvent onDock` once when all thresholds are met (range ≤ 0.15 m, closing speed ≤ 0.30 m/s, lateral offset ≤ 0.10 m, attitude error ≤ 10°). `isDocked` bool is public.
- `ApproachCorridor.cs` — cone geometry (half-angle 15°, max range 20 m) from targetPort. `inCorridor` bool and `corridorAngle` float. Draws orange/green gizmo in Scene view.
- `TelemetryLogger.cs` — 10 Hz CSV log to project root.

### Phase 3 — GNC Prep (complete)
Instrumentation and controls that mirror real GNC software behavior.

- `RateDamping.cs` — proportional rate nulling controller. Directly writes `rb.angularVelocity` and `rb.linearVelocity` to avoid ForceMode ambiguity. Toggle with H key. Backs off on any axis where RCSModel reports active thrusters (translation bits 0–5 / rotation bits 6–11). **Auto-suppresses entirely when cFS has command authority** (`cfsReceiver.CfsActive`) so it never fights the attitude autopilot. Key inspector values: `inertiaTensor = 80`, `angularDeadband = 0.1 deg/s`, `linearDeadband = 0.005 m/s`.
- `DockingHUD.cs` — OnGUI HUD (no external packages). Top-left: docking metrics (range, closing speed, lateral offset, scalar attitude error, per-axis pitch/yaw/roll errors, corridor status, RDM state, GNC phase). Bottom-left: vehicle state panel (roll/pitch/yaw ±180°, angular rates deg/s, body-frame Vx/Vy/Vz). Top-right: controls legend.
- `DockingCamera.cs` — attaches to Main Camera, follows chaserPort transform with local offset. Arrow keys articulate pitch/yaw; Enter resets to center.
- `ScenarioReset.cs` — Backspace resets chaser to initial position/rotation and zeroes velocities. `DoReset()` is callable programmatically.

### Phase 4 — cFS Integration via UDP (complete)
Close the loop: Unity sends telemetry, cFS runs the GNC law, sends timed thruster commands back.

**Control architecture — Dragon-style proportional timed burns:**

Rather than binary on/off mode (thruster ON for a full 1 s cycle), the GNC computes a proportional burn duration each cycle:

```
duration = delta_v_needed / thruster_accel     (thruster_accel = F/m = 10/200 = 0.05 m/s²)
```

Unity fires each commanded thruster for exactly `duration` seconds, then auto-cuts. This eliminates the bang-bang limit cycling that binary mode causes.

**GNC phase state machine (modeled on Dragon RPOD):**

| Phase | Value | Trigger | Control law |
|-------|-------|---------|-------------|
| `GNC_PHASE_IDLE` | 0 | No telemetry / AbortLatch set | All thrust inhibited |
| `GNC_PHASE_CORRECT` | 1 | LateralOffset > 1.0 m | Station-keep axially; drive Pos_X/Y to zero |
| `GNC_PHASE_APPROACH` | 2 | LateralOffset < 0.5 m | Proportional axial closure + lateral position hold |
| `GNC_PHASE_DOCKED` | 3 | Flags bit 1 set | All thrust inhibited |
| `GNC_PHASE_HOLD` | 4 | Ground HOLD command | Station-keep; await GO |

Hysteresis pair (0.5 m / 1.0 m) prevents rapid phase toggling. Every transition generates a `GNC_APP_PHASE_INF_EID` EVS event. Current phase is published in HK telemetry and in the cFS→Unity command packet (see UDP format below).

**SCH_LAB scheduler:** TickRate = 10 (10 Hz wall clock). GNC wakeup MID 0x1894 fires at PacketRate = 10 → 1 Hz guidance loop rate.

### Phase 5A — GNC Command Interface (complete)

`GNC_APP_CMD_MID` (0x1893) subscribed. Full command dispatch in `GNC_APP_ProcessCmd()`. System starts **pre-latched** (guidance inhibited) and requires a ground GO before any thrust fires — matching Dragon-style positive authorization.

| FC | CC define | Name | Action |
|----|-----------|------|--------|
| 0x00 | `GNC_APP_NOOP_CC` | NOOP | Heartbeat; increments CmdCount |
| 0x01 | `GNC_APP_RESET_COUNTERS_CC` | RESET\_COUNTERS | Zero CmdCount, CmdErrCount, UdpPacketsReceived |
| 0x02 | `GNC_APP_HOLD_CC` | HOLD | Enter `GNC_PHASE_HOLD`; station-keep until GO |
| 0x03 | `GNC_APP_GO_CC` | GO | Release HOLD or AbortLatch; resume guidance from CORRECT |
| 0x04 | `GNC_APP_ABORT_CC` | ABORT | Immediate coast + AbortLatch set; guidance stays inhibited until GO |

`AbortLatch` prevents `SelectPhase` from auto-promoting IDLE → CORRECT until GO is received. ABORT sends an immediate coast packet to Unity so any active burn stops without waiting for the next 1 Hz wakeup.

### Phase 5B — cFS Parameter Table (complete)

All GNC gains moved from compiled `#define` constants into a `CFE_TBL`-managed struct. Gains can be changed by editing the table source and rebuilding — or uplinked over the ground command link to a running cFS instance without a restart.

**Files:**

| File | Purpose |
|------|---------|
| `fsw/inc/gnc_app_tbl.h` | `GNC_ParamTbl_t` struct (16 floats, 64 bytes) |
| `fsw/tables/gnc_param_tbl.c` | Default table values + `CFE_TBL_FILEDEF` macro |

**Fields in `GNC_ParamTbl_t`:**

| Field | Default | Meaning |
|-------|---------|---------|
| `AxialKp` | 0.02 | Target closing speed = KP × range (m/s per m) |
| `MinCloseSpeed` | 0.02 m/s | Floor on approach speed |
| `MaxCloseSpeed` | 0.30 m/s | Cap on approach speed |
| `ThrusterForce` | 10.0 N | Must match Unity RCSModel.thrusterForce |
| `VehicleMass` | 200.0 kg | Must match Unity Rigidbody mass |
| `RotAccel` | 0.1875 rad/s² | (10 N × 1.5 m moment arm) / 80 kg·m² — must match Unity geometry |
| `MinBurnDuration` | 0.050 s | Shorter pulses coast (dead-band) |
| `MaxBurnDuration` | 0.950 s | Cap so thruster stops before next 1 Hz tick |
| `LatKp` | 0.05 | Lateral speed = KP × position error (m/s per m) |
| `MaxLatSpeed` | 0.10 m/s | Lateral speed cap |
| `LatApproachGate` | 0.50 m | Enter APPROACH when lateral offset drops below this |
| `LatCorrectGate` | 1.00 m | Enter CORRECT when lateral offset rises above this |
| `HoldPoint1_m` | 10.0 m | Outer autonomous waypoint (0 = disabled) |
| `HoldPoint2_m` | 3.0 m | Inner autonomous waypoint (0 = disabled) |
| `AttKp` | 0.50 (rad/s)/rad | Attitude proportional gain |
| `MaxAttRate` | 0.20 rad/s | Cap on commanded angular rate per axis |

### Phase 5C — Autonomous Hold Points (complete)

Hold-point ranges in `GNC_ParamTbl_t` (`HoldPoint1_m`, `HoldPoint2_m`). `SelectPhase` transitions to `GNC_PHASE_HOLD` automatically when the chaser crosses a configured range threshold during APPROACH, accounting for brake distance lookahead (`v²/a`) so the vehicle can stop at the waypoint. GNC stays in HOLD until GO is received — modeling Dragon's manual authorization at each proximity-ops waypoint. Each hold-point fires at most once per approach sequence and is re-armed only on ABORT+GO. A threshold of 0.0 disables that waypoint.

### Phase 5D — LC Safety Monitoring (complete)

LC (Limit Checker) watchpoints monitor the HK telemetry packet each cycle:

| WP # | Field | Limit | Action |
|------|-------|-------|--------|
| 0 | `ClosingSpeed_ms` | > 0.35 m/s | AP #0 → RTS 1 → ABORT |
| 1 | `LateralOffset_m` | > 2.0 m | AP #1 → RTS 2 → ABORT |
| 2 | `TlmStaleSec` | > 3 s | AP #2 → RTS 1 → ABORT (telemetry loss) |
| 3 | `ClosingSpeed_ms` | > 0.35 m/s (paired) | Part of AP #0 compound expression |

`TlmStaleSec` increments each wakeup when no fresh Unity packet arrived; resets to 0 on fresh data. All three safety cases trigger an immediate ABORT via the SC (Stored Commands) app RTS mechanism.

### Phase 5E — Clohessy-Wiltshire Feedforward (complete)

`ComputeControl` computes CW feedforward delta-v each 1 Hz cycle and folds it into each channel's velocity error before computing burn duration. This pre-cancels the differential gravity that `ClohessyWiltshire.cs` applies every FixedUpdate:

```
ff_x = -(3n²·rx + 2n·vy)
ff_y =  +2n·vx
ff_z =  +n²·rz          (n = 0.00113 rad/s)
```

The feedforward is added directly to the velocity error so the existing burn duration formula absorbs it — no extra thruster logic needed.

### Phase 5F — Attitude Autopilot (complete)

Closed-loop attitude control: the GNC commands the chaser to autonomously align its docking port with the ISS port and hold that attitude throughout the approach.

**Unity side:**
- `RelativeNav.cs` — decomposes the error quaternion (`Quaternion.Inverse(chaserPort.rotation) * targetRot`) into per-axis `pitchError`, `yawError`, `rollError` (degrees, [-180, 180]).
- `UdpTelemetrySender.cs` — telemetry packet extended from 60 → **72 bytes**; per-axis errors appended at offsets 60/64/68.
- `RateDamping.cs` — fully suppressed when `cfsReceiver.CfsActive` is true; prevents rate damping from fighting the cFS attitude controller during autonomous flight.

**cFS side — Channel 3 (attitude PD, all active phases):**

```
omega_tgt = clamp(AttKp × error_rad, ±MaxAttRate)
omega_err = omega_tgt − AngVel
duration  = |omega_err| / RotAccel
```

Per-axis (pitch/yaw/roll) independently. Thruster bit map: +pitch=6, -pitch=7, +yaw=8, -yaw=9, +roll=10, -roll=11.

**Command packet extended from 8 → 12 bytes** — bytes 8–11 carry the current `GNC_Phase_t` value so Unity always knows the GNC state without a separate channel.

**HUD additions:**
- Per-axis PITCH / YAW / ROLL error rows (green ≤ ±5°, red > ±5°), indented under the scalar ATTITUDE row.
- GNC phase row — color-coded: gray=IDLE, yellow=CORRECT, cyan=APPROACH, green=DOCKED, orange=HOLD. Shows `---` when cFS is not connected.
- RDM row shows `SUPPRESSED` when cFS has authority.

---

## UDP Packet Format

### Telemetry (Unity → cFS) — port 5005 — 72 bytes, little-endian

| Offset | Type   | Field               |
|--------|--------|---------------------|
| 0      | float  | MET_s               |
| 4      | float  | Range_m             |
| 8      | float  | ClosingSpeed_ms     |
| 12     | float  | LateralOffset_m     |
| 16     | float  | AttitudeError_deg   |
| 20     | float  | Pos_X               |
| 24     | float  | Pos_Y               |
| 28     | float  | Pos_Z               |
| 32     | float  | Vel_X               |
| 36     | float  | Vel_Y               |
| 40     | float  | Vel_Z               |
| 44     | float  | AngVel_X            |
| 48     | float  | AngVel_Y            |
| 52     | float  | AngVel_Z            |
| 56     | int32  | Flags (bit0=InCorridor, bit1=Docked) |
| 60     | float  | PitchError_deg      |
| 64     | float  | YawError_deg        |
| 68     | float  | RollError_deg       |

Struct is `__attribute__((packed))` in C (`GNC_APP_UnityTlm_t`). `BuildPacket()` in `UdpTelemetrySender.cs` must write fields in this exact order.

### Command (cFS → Unity) — port 5006 — 32 bytes, little-endian (Phase 6-4+)

| Offset | Type  | Field       | Notes |
|--------|-------|-------------|-------|
| 0      | float | Fx          | N, body-frame force |
| 4      | float | Fy          | N |
| 8      | float | Fz          | N |
| 12     | float | Tx          | N·m, body-frame torque |
| 16     | float | Ty          | N·m |
| 20     | float | Tz          | N·m |
| 24     | float | Duration_s  | Seconds each fired thruster fires |
| 28     | int32 | Phase       | `GNC_Phase_t` value (0–4) |

A zero wrench (all forces and torques = 0.0) is a coast/heartbeat command.
Unity's `ThrusterAllocator` pseudo-inverse maps the wrench to physical thrusters.

**Axis convention (approach along world +Z, ship upright):**
- Approach axis: body +Z = world +Z. Bit 4 closes range; bit 5 brakes.
- Lateral X: body +X = world +X. Bit 0 = +X force; bit 1 = -X force.
- Lateral Y: body +Y = world +Y. Bit 2 = +Y force; bit 3 = -Y force.

---

## Phase 6 — RCS Physics Overhaul (planned)

**Goal:** Replace the current abstract thruster model (each bit = one isolated effect) with physically accurate thrusters that have real body-frame positions and orientations. A single thruster firing produces coupled force **and** torque via **r × F**. cFS outputs a 6-DOF wrench; Unity solves control allocation.

This mirrors how real spacecraft GNC actually works: the guidance law computes desired `[Fx, Fy, Fz, Tx, Ty, Tz]`, a separate control allocator maps that wrench to individual thruster on-times, and each thruster inherently couples translation and rotation.

### 6-1 — Thruster Geometry Definition
Define all 12 physical thrusters as position + direction vectors in Dragon's body frame. Validate with Scene-view gizmos before any behavior changes.

**Touches:** `RCSModel.cs` (thruster data array)

### 6-2 — Coupled Physics in Unity
Replace the switch-case force/torque block with a loop: for each active thruster bit, `AddForce(dir * F)` and `AddTorque(cross(pos, dir) * F)`. Keyboard control works immediately; you can already observe that translation thrusters slightly rotate the ship.

**Touches:** `RCSModel.cs`

### 6-3 — Effectiveness Matrix + Pseudo-Inverse (complete)
At startup, build the **6×N effectiveness matrix B** (columns = per-thruster `[direction; r × direction]`). Compute right pseudo-inverse **B† = B^T(BB^T)^-1** via Gauss-Jordan. This is the core of the allocator and is computed once, not per-frame. Keyboard allocation now uses the pseudo-inverse with a 20%-of-peak relative threshold.

**Touches:** New `ThrusterAllocator.cs`, `RCSModel.cs`

### 6-4 — New cFS Wrench Command Packet (complete)
Changed the cFS → Unity command packet from `mask(int32) + duration(float) + phase(int32)` (12 bytes) to `Fx, Fy, Fz, Tx, Ty, Tz (6 floats) + duration(float) + phase(int32)` = **32 bytes**. `gnc_app_udp.c` converts the legacy bitmask to a body-frame wrench before packing; Unity's `UdpCommandReceiver` receives the wrench and calls `RCSModel.SetWrenchCommand()`, which runs the pseudo-inverse allocator to map it to physical thrusters.

**Touches:** `gnc_app_udp.c`, `gnc_app.h` (`GNC_RCS_MOMENT_ARM` constant), `UdpCommandReceiver.cs`, `RCSModel.cs`

### 6-5 — GNC Wrench Output in cFS (complete)
`GNC_Control_t` changed from `{mask, duration}` to `{Fx, Fy, Fz, Tx, Ty, Tz, duration}`. `GNC_APP_ComputeControl()` now sets signed force/torque components directly instead of bit-flags. `GNC_APP_SendCommand()` packs the wrench struct directly into the 32-byte UDP packet — no mask→wrench conversion. The WAKEUP_INF log now shows `F=(x,y,z) T=(x,y,z) Dur=s` instead of `Cmd=0xXXX`.

**Known limitation:** Fx and Fy from the lateral correction channel are zeroed by Unity's `SetWrenchCommand` (the Dragon pod geometry maps lateral force requests to yaw thrusters, causing divergence). Lateral correction effectively coasts until the thruster geometry is fixed (Phase 7).

**Touches:** `gnc_app.c`, `gnc_app.h`, `gnc_app_udp.c`

### 6-6 — Stuck Thruster Fault Injection
Add a fault mode to `RCSModel.cs`: mark one or more thrusters as stuck-on. A stuck thruster fires every `FixedUpdate` regardless of commanded state, producing a genuine coupled disturbance (off-axis force + torque) that the GNC must sense and fight. This is the educational payoff of the full overhaul — a realistic fault the closed-loop system has to handle.

**Touches:** `RCSModel.cs` (fault flags, Inspector toggles)

---

## Key Design Principles

1. **Mirror real flight software** — proportional timed burns (Dragon RPOD model), phase state machine with hysteresis, rate damping, approach corridor, attitude PD, LC safety monitoring, and telemetry packet format are all modeled after actual GNC software.
2. **Physics accuracy over convenience** — inertia tensor set explicitly, torques applied in body frame, Clohessy-Wiltshire drift applied, trigger colliders used so contact doesn't corrupt dynamics.
3. **cFS integration points are isolated** — `RCSModel.SetThrusterCommand()`, `UdpTelemetrySender`, and `UdpCommandReceiver` are the only touch points between the sim and cFS. Everything else is self-contained.
4. **Coupling warning** — `ThrusterForce` and `VehicleMass` in `GNC_ParamTbl_t` must always match `RCSModel.thrusterForce` and the Rigidbody mass in the Unity Inspector. `RotAccel` must match `(thrusterForce × momentArm) / inertiaTensor`. If any of these drift the GNC will systematically under- or over-shoot every burn.
5. **No external Unity packages** — OnGUI for HUD, raw UDP sockets for networking. Keeps the project portable and dependency-free.
6. **Scene2.unity** is the working scene. SampleScene is unused.

---

## EVS Event IDs (gnc_app.h)

| EID | Name | Meaning |
|-----|------|---------|
| 1 | INIT\_INF | App initialized successfully |
| 2 | WAKEUP\_INF | 1 Hz wakeup log (phase, range, speed, lateral, attitude errors, mask, duration) |
| 5 | UDP\_INIT\_INF | Telemetry recv socket bound on port 5005 |
| 9 | CMD\_INIT\_INF | Command send socket ready |
| 11 | PHASE\_INF | Phase transition (old→new) |
| 12 | NOOP\_INF | NOOP command received |
| 13 | RST\_INF | RESET\_COUNTERS command received |
| 14 | HOLD\_INF | HOLD command received |
| 15 | GO\_INF | GO command received |
| 16 | ABORT\_CRIT | ABORT command received (CRITICAL severity) |
| 17 | CMD\_LEN\_ERR | Malformed command (wrong length) |
| 18 | CMD\_CODE\_ERR | Unknown command function code |
| 19 | TBL\_UPD\_INF | Parameter table image activated |
| 21 | HOLDPT1\_INF | Autonomous hold point 1 triggered |
| 22 | HOLDPT2\_INF | Autonomous hold point 2 triggered |

---

## Controls Reference

| Key        | Action              |
|------------|---------------------|
| W / S      | Forward / Back (+Z / -Z) |
| A / D      | Left / Right (-X / +X)   |
| Space      | Up (+Y)             |
| Ctrl       | Down (-Y)           |
| R / F      | Pitch up / down     |
| E / Q      | Yaw right / left    |
| Z / X      | Roll CW / CCW       |
| H          | Toggle Rate Damping (suppressed while cFS active) |
| Backspace  | Reset scenario      |
| Arrow keys | Articulate camera   |
| Enter      | Center camera       |

---

## Build and Run

See **OPERATIONS.md** for the full step-by-step guide including Docker setup, building, running, and sending ground commands.

Quick reference:
```bash
./cfs-dev.sh                                   # start container (ports 5005 + 1234 mapped)
make native_std.install                        # build inside container
cd /build-native_std/exe/cpu1 && ./core-cpu1  # run cFS inside container
python3 gnc_cmd.py go                          # from Mac — release pre-launch hold
```
