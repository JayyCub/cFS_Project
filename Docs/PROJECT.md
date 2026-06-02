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
│   │           │   └── gnc_app_tbl.h     ← GNC_ParamTbl_t struct (Phase 5B)
│   │           ├── src/
│   │           │   ├── gnc_app.h
│   │           │   ├── gnc_app.c
│   │           │   └── gnc_app_udp.c
│   │           └── tables/
│   │               └── gnc_param_tbl.c   ← default gain table (Phase 5B)
│   └── sample_defs/
│       └── tables/
│           ├── sch_lab_table.c   ← 10 Hz tick, GNC wakeup at 1 Hz
│           └── lc_def_adt-test.c ← LC actionpoints (not yet wired to GNC)
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

- `RelativeNav.cs` — computes approach state every FixedUpdate from port Transform positions and Rigidbody velocities: `range`, `closingSpeed` (positive = approaching), `lateralOffset`, `attitudeError` (0° = ports perfectly anti-parallel/ready to dock).
- `DockingDetector.cs` — monitors RelativeNav; fires `UnityEvent onDock` once when all thresholds are met (range ≤ 0.15 m, closing speed ≤ 0.30 m/s, lateral offset ≤ 0.10 m, attitude error ≤ 10°). `isDocked` bool is public.
- `ApproachCorridor.cs` — cone geometry (half-angle 15°, max range 20 m) from targetPort. `inCorridor` bool and `corridorAngle` float. Draws orange/green gizmo in Scene view.
- `TelemetryLogger.cs` — 10 Hz CSV log to project root. Columns define the Phase 4 UDP packet format: `MET_s, Range_m, ClosingSpeed_ms, LateralOffset_m, AttitudeError_deg, Pos_X/Y/Z, Vel_X/Y/Z, AngVel_X/Y/Z, InCorridor, Docked`.

### Phase 3 — GNC Prep (complete)
Instrumentation and controls that mirror real GNC software behavior.

- `RateDamping.cs` — proportional rate nulling controller. Directly writes `rb.angularVelocity` and `rb.linearVelocity` to avoid ForceMode ambiguity. Toggle with H key. Backs off completely when RCSModel reports active thrusters on that axis (translation bits 0–5 / rotation bits 6–11) so it does not fight intentional commands. Key inspector values: `inertiaTensor = 80`, `angularDeadband = 0.1 deg/s`, `linearDeadband = 0.005 m/s`.
- `DockingHUD.cs` — OnGUI HUD (no external packages). Top-left: docking metrics color-coded green/red. Middle-left: corridor status and RDM mode (cyan = ON). Bottom-left: vehicle state panel (roll/pitch/yaw ±180°, angular rates deg/s, body-frame Vx/Vy/Vz). Top-right: controls legend.
- `DockingCamera.cs` — attaches to Main Camera, follows chaserPort transform with local offset. Arrow keys articulate pitch/yaw; Enter resets to center.
- `ScenarioReset.cs` — Backspace resets chaser to initial position/rotation and zeroes velocities. `DoReset()` is callable programmatically.

### Phase 4 — cFS Integration via UDP (complete)
Close the loop: Unity sends telemetry, cFS runs the GNC law, sends timed thruster commands back.

**Control architecture — Dragon-style proportional timed burns:**

Rather than binary on/off mode (thruster ON for a full 1 s cycle), the GNC computes a proportional burn duration each cycle:

```
duration = delta_v_needed / thruster_accel     (thruster_accel = F/m = 10/200 = 0.05 m/s²)
```

Unity fires each commanded thruster for exactly `duration` seconds, then auto-cuts. This eliminates the bang-bang limit cycling that caused the 0.05 ↔ 0.10 m/s oscillation observed in the binary mode.

**GNC phase state machine (modeled on Dragon RPOD):**

| Phase | Value | Trigger | Control law |
|-------|-------|---------|-------------|
| `GNC_PHASE_IDLE` | 0 | No telemetry received yet | All thrust inhibited |
| `GNC_PHASE_CORRECT` | 1 | LateralOffset > 1.0 m | Station-keep axially; drive Pos_X/Y to zero |
| `GNC_PHASE_APPROACH` | 2 | LateralOffset < 0.5 m | Proportional axial closure + lateral position hold |
| `GNC_PHASE_DOCKED` | 3 | Flags bit 1 set | All thrust inhibited |

Hysteresis pair (0.5 m / 1.0 m) prevents rapid phase toggling. Every transition generates a `GNC_APP_PHASE_INF_EID` EVS event for ground visibility. Current phase is published in the HK telemetry packet.

**GNC gain summary:**

| Constant | Value | Meaning |
|----------|-------|---------|
| `GNC_AXIAL_KP` | 0.02 | Target closing speed = KP × range (m/s per m) |
| `GNC_MIN_CLOSE_SPEED` | 0.02 m/s | Floor on approach speed |
| `GNC_MAX_CLOSE_SPEED` | 0.30 m/s | Cap on approach speed |
| `GNC_LAT_KP` | 0.05 | Lateral speed = KP × lateral position error (m/s per m) |
| `GNC_MAX_LAT_SPEED` | 0.10 m/s | Cap on lateral correction speed |
| `GNC_MIN_BURN_DURATION` | 0.050 s | Minimum pulse — shorter burns coast |
| `GNC_MAX_BURN_DURATION` | 0.950 s | Cap — thruster always off before next 1 Hz tick |
| `GNC_THRUSTER_FORCE` | 10.0 N | Must match Unity Inspector RCSModel.thrusterForce |
| `GNC_VEHICLE_MASS` | 200.0 kg | Must match Unity Rigidbody mass |

**SCH_LAB scheduler:** TickRate = 10 (10 Hz wall clock). GNC wakeup MID 0x1894 fires at PacketRate = 10 → 1 Hz guidance loop rate.

**Unity side (complete):**
- `UdpTelemetrySender.cs` — sends 60-byte telemetry packet to cFS at 10 Hz (`127.0.0.1:5005`). Background send thread.
- `UdpCommandReceiver.cs` — listens on port 5006 for 8-byte command packet. Extracts mask (int32) and duration (float). Calls `rcsModel.SetThrusterCommand(mask, duration)`. Command timeout = 1.5 s before reverting to keyboard.

**cFS side (complete):**
- `gnc_app.c` — Main task loop, Init, ProcessWakeup. Subscribes to SCH_LAB wakeup MID. Runs `SelectPhase` then `ComputeControl` then `SendCommand` each wakeup. Publishes HK TLM packet to SB.
- `gnc_app_udp.c` — Background OSAL task (`GNC_UDP_RECV`, priority 80) blocks on `OS_SocketRecvFrom`. Locks mutex, copies packet to `GNC_APP_Data.LatestTlm`, sets `TlmFresh`. `GNC_APP_SendCommand()` sends 8-byte command packet to Unity (`host.docker.internal:5006`).
- `gnc_app.h` — All type definitions and constants. `GNC_Phase_t` enum must be declared before `GNC_APP_Data_t` (C forward-declaration rule). `GNC_APP_HkTlm_t` includes `uint8 Phase; uint8 Spare[3];` for word-alignment.

---

## UDP Packet Format

### Telemetry (Unity → cFS) — port 5005 — 60 bytes, little-endian

| Offset | Type   | Field             |
|--------|--------|-------------------|
| 0      | float  | MET_s             |
| 4      | float  | Range_m           |
| 8      | float  | ClosingSpeed_ms   |
| 12     | float  | LateralOffset_m   |
| 16     | float  | AttitudeError_deg |
| 20     | float  | Pos_X             |
| 24     | float  | Pos_Y             |
| 28     | float  | Pos_Z             |
| 32     | float  | Vel_X             |
| 36     | float  | Vel_Y             |
| 40     | float  | Vel_Z             |
| 44     | float  | AngVel_X          |
| 48     | float  | AngVel_Y          |
| 52     | float  | AngVel_Z          |
| 56     | int32  | Flags (bit0=InCorridor, bit1=Docked) |

Struct is `__attribute__((packed))` in C; `BuildPacket()` in `UdpTelemetrySender.cs` must write fields in this exact order.

### Command (cFS → Unity) — port 5006 — 8 bytes, little-endian

| Offset | Type  | Field        |
|--------|-------|--------------|
| 0      | int32 | ThrusterMask |
| 4      | float | BurnDuration_s |

ThrusterMask bit assignments match `RCSModel.cs`:
- Bits 0–5: translation (+X, -X, +Y, -Y, +Z, -Z)
- Bits 6–11: rotation (+pitch, -pitch, +yaw, -yaw, +roll, -roll)

A mask of 0 or duration of 0.0 is a coast command — no force applied.

**Axis convention (approach along world +Z, ship upright):**
- Approach axis: body +Z = world +Z. Bit 4 closes range; bit 5 brakes.
- Lateral X: body +X = world +X. Bit 0 = +X force; bit 1 = -X force.
- Lateral Y: body +Y = world +Y. Bit 2 = +Y force; bit 3 = -Y force.

---

## Phase 5 — Roadmap

Priority order reflects real flight software maturity model.

### 5A — GNC Command Interface (complete)

`GNC_APP_CMD_MID` (0x1893) is subscribed. Full command dispatch in `GNC_APP_ProcessCmd()`. System starts **pre-latched** (guidance inhibited) and requires a ground GO before any thrust fires — matching Dragon-style positive authorization.

| FC | CC define | Name | Action |
|----|-----------|------|--------|
| 0x00 | `GNC_APP_NOOP_CC` | NOOP | Heartbeat; increments CmdCount |
| 0x01 | `GNC_APP_RESET_COUNTERS_CC` | RESET\_COUNTERS | Zero CmdCount, CmdErrCount, UdpPacketsReceived |
| 0x02 | `GNC_APP_HOLD_CC` | HOLD | Enter `GNC_PHASE_HOLD`; station-keep until GO |
| 0x03 | `GNC_APP_GO_CC` | GO | Release HOLD or AbortLatch; resume guidance from CORRECT |
| 0x04 | `GNC_APP_ABORT_CC` | ABORT | Immediate coast + AbortLatch set; guidance stays inhibited until GO |

`AbortLatch` prevents `SelectPhase` from auto-promoting IDLE → CORRECT until a GO is received. ABORT sends an immediate coast packet to Unity so any active burn stops without waiting for the next 1 Hz wakeup.

Commands are sent via `gnc_cmd.py` (see OPERATIONS.md). Packets are CCSDS header-only (8 bytes) delivered to CI_LAB on port 1234.

**Key EVS events added:** NOOP (12), RST (13), HOLD (14), GO (15), ABORT (16, CRITICAL), CMD\_LEN\_ERR (17), CMD\_CODE\_ERR (18).

**Recv task robustness fix:** `OS_SocketRecvFrom` now receives into a real `OS_SockAddr_t` instead of NULL — some OSAL/Linux builds silently error on NULL and kill the recv task. Both error paths (fatal error and wrong packet size) now emit EVS events instead of discarding silently.

### 5B — cFS Parameter Table (complete)

All GNC gains moved from compiled `#define` constants into a `CFE_TBL`-managed struct. Gains can now be changed by editing the table source file and rebuilding — or uplinked over the ground command link to a running cFS instance without a restart.

**New files:**

| File | Purpose |
|------|---------|
| `fsw/inc/gnc_app_tbl.h` | `GNC_ParamTbl_t` struct (12 floats, 48 bytes) |
| `fsw/tables/gnc_param_tbl.c` | Default table values + `CFE_TBL_FILEDEF` macro; builds to `/cf/gnc_param_tbl.tbl` |

**Changes to existing files:**
- `CMakeLists.txt`: added `add_cfe_tables(gnc_app fsw/tables/gnc_param_tbl.c)`
- `gnc_app.h`: added `#include "gnc_app_tbl.h"`, EIDs 19/20, `ParamTblHandle` + `ParamTblPtr` in `GNC_APP_Data_t`
- `gnc_app.c` Init: `CFE_TBL_Register` → `CFE_TBL_Load` → `CFE_TBL_GetAddress` at startup
- `gnc_app.c` ProcessWakeup: `CFE_TBL_Manage` + `GetAddress` each 1 Hz cycle — picks up uplinked table images and logs EID 19 on activation
- `gnc_app.c` ComputeControl + SelectPhase: all `#define` constants replaced with `p->FieldName` reads through the live table pointer

**Fields in `GNC_ParamTbl_t`:** `AxialKp`, `MinCloseSpeed`, `MaxCloseSpeed`, `ThrusterForce`, `VehicleMass`, `RotAccel`, `MinBurnDuration`, `MaxBurnDuration`, `LatKp`, `MaxLatSpeed`, `LatApproachGate`, `LatCorrectGate`.

The old `#define` constants remain in `gnc_app.h` as documented defaults only — the control law no longer reads them at runtime.

### 5C — Autonomous Hold Points

Depends on 5A ✓ and 5B ✓.

Add hold-point ranges to `GNC_ParamTbl_t` (e.g., 10 m, 3 m). `SelectPhase` transitions to `GNC_PHASE_HOLD` automatically when the chaser crosses a configured range threshold during APPROACH. GNC stays in HOLD until the operator sends GO — modeling Dragon's manual authorization at each proximity-ops waypoint. Each hold-point arrival generates an EVS event.

Key additions:
- `HoldPoint1_m` / `HoldPoint2_m` float fields in `GNC_ParamTbl_t`
- `SelectPhase`: check range against hold-point thresholds during APPROACH
- Distinct EVS event per hold-point crossing (with range at time of entry)

### 5D — LC Safety Monitoring

The Limit Checker (LC) app is already running but has no GNC watchpoints. This phase wires it up.

- Add watchpoints to `lc_def_wdt.c`:
  - Range decreasing while closing speed > 0.35 m/s (overspeed)
  - LateralOffset > 2.0 m during APPROACH (off-corridor)
  - UdpPacketsReceived not incrementing (telemetry loss watchdog)
- Add actionpoints to `lc_def_adt-test.c`:
  - Overspeed → trigger RTS that sends ABORT command to GNC_APP
  - Telemetry loss → trigger RTS that sends ABORT command to GNC_APP
- Write abort RTS sequences in `sc_rts001.c` / `sc_rts002.c` for SC app

### 5E — Clohessy-Wiltshire Feedforward

`ClohessyWiltshire.cs` already applies CW differential gravity to Unity physics, but `gnc_app.c` ignores it. At ISS orbit (n = 0.00113 rad/s) the CW accelerations are small but accumulate over a long approach. A feedforward term would cancel them.

CW equations:
```
ax =  3n²rx + 2n·vy
ay = -2n·vx
az = -n²rz
```

Add to `ComputeControl`: compute CW acceleration from current position and velocity, convert to required delta-v per 1 Hz cycle, add to the velocity error before computing burn duration. This is the same feedforward structure used in real rendezvous GNC.

---

## Key Design Principles

1. **Mirror real flight software** — proportional timed burns (Dragon RPOD model), phase state machine with hysteresis, rate damping, approach corridor, and telemetry packet format are all modeled after actual GNC software.
2. **Physics accuracy over convenience** — inertia tensor set explicitly, torques applied in body frame, Clohessy-Wiltshire drift applied, trigger colliders used so contact doesn't corrupt dynamics.
3. **cFS integration points are isolated** — `RCSModel.SetThrusterCommand()`, `UdpTelemetrySender`, and `UdpCommandReceiver` are the only touch points between the sim and cFS. Everything else is self-contained.
4. **Coupling warning** — `GNC_THRUSTER_FORCE` (gnc_app.h) and `thrusterForce` (RCSModel Inspector) must always match. `GNC_VEHICLE_MASS` must always match the Rigidbody mass. If they drift the GNC will systematically under- or over-shoot every burn.
5. **No external Unity packages** — OnGUI for HUD, raw UDP sockets for networking. Keeps the project portable and dependency-free.
6. **Scene2.unity** is the working scene. SampleScene is unused.

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
| H          | Toggle Rate Damping |
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
