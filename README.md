# cFS Spacecraft Docking Simulator

*A project by Jacob Thomsen*

A hardware-in-the-loop spacecraft docking simulator built on NASA's Core Flight System (cFS). A Unity physics engine simulates a chaser vehicle in LEO; a real C flight software application running inside cFS computes the guidance, navigation, and control (GNC) logic; the two communicate over UDP in real time.

This is a learning project. Every design decision is made to mirror how real flight software works.  
*(Well also I don't have an orbital spacecraft available to me)*

**Technologies used:**
- **C** — GNC flight software application (`gnc_app`) running inside NASA cFS
- **C#** — Unity scripts for 6-DOF physics, RCS model, telemetry sender, and command receiver
- **Python** — ground command tool (`gnc_cmd.py`) that sends CCSDS packets to cFS
- **NASA cFS / cFE** — real flight software framework providing the scheduler, event system, table manager, and software bus
- **Unity 6** — physics simulation and scene rendering (plant model only; no guidance logic)
- **Docker** — containerized cFS build and runtime environment
- **Blender** — tweaking 3D models of the International Space Station and SpaceX Crew Dragon capsule

---

## Real-World Context

Spacecraft rendezvous and proximity operations (RPOD) require a flight computer to receive sensor telemetry, run a control law, and publish thruster commands on a schedule while being commanded and monitored from the ground. SpaceX Dragon, Boeing Starliner, and NASA Orion all follow this pattern on final approach to a station.

This project maps that architecture to accessible tools:

| Real Spacecraft | This Project |
|-----------------|--------------|
| Flight computer running VxWorks or similar RTOS | Docker container running NASA cFS on Linux |
| Inertial measurement unit, LIDAR range sensor | Unity physics engine streaming 10 Hz UDP telemetry |
| Thruster control electronics | Unity `RCSModel.cs` receiving timed-burn command packets |
| CCSDS ground command uplink | `gnc_cmd.py` sending CCSDS packets to cFS CI\_LAB |
| Onboard parameter tables (uplinked in-flight) | `CFE_TBL`-managed gain table, live-uplink capable |
| EVS event log (downlinked to ground) | cFS EVS messages printed to the operator console |
| Dragon hold-and-proceed waypoints | Abort latch + GO/HOLD/ABORT command interface |

Clohessy-Wiltshire differential gravity equations run continuously in Unity, producing the relative-motion drift a real chaser would experience at ISS altitude (n = 0.00113 rad/s). The GNC ignores this in the current implementation; a planned feedforward term would cancel it, as real rendezvous GNC does.

---

## System Architecture

```
┌─────────────────────────────────────┐        ┌───────────────────────────────────┐
│           Unity (Mac)               │        │       cFS (Docker container)      │
│                                     │        │                                   │
│  ┌──────────────────────────────┐   │        │  ┌──────────────────────────────┐ │
│  │  Physics Simulation          │   │  UDP   │  │  gnc_app (custom C app)      │ │
│  │  6-DOF dynamics              │   │ telemetry │  SelectPhase()               │ │
│  │  Clohessy-Wiltshire drift    │───────────>│  │  ComputeControl()            │ │
│  │  12-thruster RCS model       │   │  10 Hz │  │  SendCommand()               │ │
│  │  Docking corridor / detector │   │ port 5005 │  1 Hz wakeup (SCH_LAB)       │ │
│  └──────────────────────────────┘   │        │  └──────────────────────────────┘ │
│                                     │        │              │                    │
│  ┌──────────────────────────────┐   │  UDP   │  ┌──────────────────────────────┐ │
│  │  UdpCommandReceiver          │<───────────│  │  gnc_app_udp (OSAL task)     │ │
│  │  Applies thruster mask +     │   │ command│  │  Recv telemetry (port 5005)  │ │
│  │  burn duration to RCSModel   │   │ port 5006 │  Send commands (port 5006)   │ │
│  └──────────────────────────────┘   │        │  └──────────────────────────────┘ │
└─────────────────────────────────────┘        └───────────────────────────────────┘
                                                               ^
                                                               │ CCSDS commands
                                                          port 1234 (CI_LAB)
                                                               │
                                                    python3 gnc_cmd.py go/hold/abort
```

---

## Components

### GNC Application (cFS / C)

`gnc_app` is a standard cFS application written in C that runs inside NASA's Core Flight Executive (cFE). It subscribes to the SCH\_LAB scheduler wakeup message and runs its guidance loop at 1 Hz.

**Phase state machine** (modeled on Dragon RPOD):

| Phase | Trigger | Control law |
|-------|---------|-------------|
| `IDLE` | No telemetry received, or startup | All thrust inhibited |
| `CORRECT` | Lateral offset > 1.0 m | Station-keep axially; drive lateral position to zero |
| `APPROACH` | Lateral offset < 0.5 m | Proportional axial closure + lateral position hold |
| `HOLD` | Ground command | Station-keep at current range; await GO |
| `DOCKED` | Contact flags set | All thrust inhibited |

The 0.5 m / 1.0 m hysteresis band prevents rapid phase toggling. Every transition generates an EVS event visible in the operator console.

**Proportional timed-burn control law:**

Rather than a simple on/off thruster command, the GNC computes the exact burn duration needed to achieve a target velocity correction each cycle:

```
duration = Δv_needed / thruster_accel     (thruster_accel = F/m = 10 N / 200 kg = 0.05 m/s²)
```

Unity fires each commanded thruster for exactly `duration` seconds, then auto-cuts. This removes the limit-cycling oscillation that a bang-bang controller produces.

**Safety features:**

- **Abort latch**: the system starts guidance-inhibited and requires an explicit GO command before any thrust fires. An ABORT command sets the latch and sends an immediate coast packet to Unity, stopping any active burn without waiting for the next 1 Hz wakeup. Guidance stays inhibited until GO is sent again.
- **CCSDS command dispatch**: all ground commands arrive as properly-formatted CCSDS packets (NOOP, RESET\_COUNTERS, HOLD, GO, ABORT). Unknown function codes and malformed packet lengths generate EVS error events.
- **CFE\_TBL parameter management**: all GNC gains live in a `CFE_TBL`-managed struct (`GNC_ParamTbl_t`) rather than compiled `#define` constants. Parameters can be changed by uplink to a running cFS instance without recompile or restart. The `ProcessWakeup` cycle calls `CFE_TBL_Manage` every 1 Hz to pick up newly activated table images.

**Source files:**

| File | Purpose |
|------|---------|
| `cFS/apps/gnc_app/fsw/src/gnc_app.c` | Main task, Init, ProcessWakeup, SelectPhase, ComputeControl |
| `cFS/apps/gnc_app/fsw/src/gnc_app_udp.c` | Background OSAL recv task; `GNC_APP_SendCommand()` |
| `cFS/apps/gnc_app/fsw/inc/gnc_app.h` | All type definitions, constants, `GNC_APP_Data_t` |
| `cFS/apps/gnc_app/fsw/inc/gnc_app_tbl.h` | `GNC_ParamTbl_t` struct (12 gain fields) |
| `cFS/apps/gnc_app/fsw/tables/gnc_param_tbl.c` | Default gain values; builds to `/cf/gnc_param_tbl.tbl` |

---

### Unity Physics Simulation

Unity 6 runs the physics and renders the scene. It applies forces and integrates dynamics; all guidance logic is in cFS.

**6-DOF dynamics:**

- Both vehicles are Unity Rigidbodies. Chaser mass is 200 kg with inertia tensor set explicitly to 80 kg·m² per axis. Unity's auto-computed inertia from collider geometry is not used.
- `ClohessyWiltshire.cs` applies a differential gravity force each `FixedUpdate`, producing the relative-motion drift a real chaser experiences at ISS orbit. This runs whether or not cFS is connected.
- Colliders are set to `Is Trigger = true`. Contact does not generate collision response forces; docking is detected in software by `DockingDetector.cs` monitoring range, closing speed, lateral offset, and attitude error thresholds.

**RCS model:**

`RCSModel.cs` implements a 12-thruster layout controlled by a bitmask:

| Bits | Axis |
|------|------|
| 0–5 | Translation: +X, −X, +Y, −Y, +Z, −Z |
| 6–11 | Rotation: +pitch, −pitch, +yaw, −yaw, +roll, −roll |

Torques are applied in body frame using `transform.right/up/forward`. The `SetThrusterCommand(int mask, float duration)` method is the cFS integration hook. Unity auto-cuts the thruster at `burnEndTime`, so cFS never sends a stop packet.

**Telemetry and commands:**

`UdpTelemetrySender.cs` sends a 60-byte telemetry packet to cFS at 10 Hz on a background thread. `UdpCommandReceiver.cs` listens on port 5006 for 8-byte command packets, extracts the thruster mask and burn duration, and calls `SetThrusterCommand`. If no command arrives within 1.5 seconds, the keyboard regains control.

**UDP packet format:**

Telemetry (Unity to cFS, 60 bytes):

| Field | Type | Offset |
|-------|------|--------|
| MET\_s | float | 0 |
| Range\_m | float | 4 |
| ClosingSpeed\_ms | float | 8 |
| LateralOffset\_m | float | 12 |
| AttitudeError\_deg | float | 16 |
| Pos\_X/Y/Z | float×3 | 20 |
| Vel\_X/Y/Z | float×3 | 32 |
| AngVel\_X/Y/Z | float×3 | 44 |
| Flags (bit0=InCorridor, bit1=Docked) | int32 | 56 |

Command (cFS to Unity, 8 bytes):

| Field | Type | Offset |
|-------|------|--------|
| ThrusterMask | int32 | 0 |
| BurnDuration\_s | float | 4 |

---

### Ground Command Interface

`gnc_cmd.py` constructs CCSDS command packets and sends them to cFS's CI\_LAB uplink port (1234/udp) from the Mac host.

```bash
python3 gnc_cmd.py <command>
```

| Command | Effect |
|---------|--------|
| `noop` | Heartbeat; verifies the command link is alive. Increments CmdCount in HK telemetry. |
| `reset` | Zeros HK counters (CmdCount, CmdErrCount, UdpPacketsReceived). |
| `hold` | Freeze at current range. GNC station-keeps with no axial closure. |
| `go` | Release a hold or the startup pre-latch. Resumes guidance from CORRECT phase. |
| `abort` | Emergency stop. Sends immediate coast to Unity; inhibits guidance until GO. |

Every command is acknowledged by an EVS event in the cFS console. GO is rejected if guidance is already active.

---

## Getting Started

### Prerequisites

| Tool | Purpose |
|------|---------|
| Docker Desktop | Runs the cFS build and runtime environment |
| Unity 6 | Runs the physics simulation |
| Python 3 | Sends ground commands |

No external Unity packages. No other dependencies.

### 1. Clone with submodules

```bash
git clone --recurse-submodules <repo-url>
cd cFS_Project
```

### 2. Start the Docker container

```bash
./cfs-dev.sh
```

This builds the image on first run (~1 minute) and drops you into a shell inside the container. The `cFS/` directory is bind-mounted at `/cfs`; edits on the host are immediately visible inside the container.

### 3. Build cFS (inside the container)

```bash
make native_std.install
```

First build takes 2-3 minutes. Incremental rebuilds of `gnc_app` alone take a few seconds.

### 4. Run cFS (inside the container)

```bash
cd /build-native_std/exe/cpu1
./core-cpu1
```

Watch for these lines confirming `gnc_app` is healthy:

```
GNC_APP initialized. Recv port 5005, Cmd port 5006. Guidance INHIBITED — send GO to start.
GNC_APP UDP: listening on port 5005
```

### 5. Start Unity

Open `cFS_DockingSim/` in the Unity Editor and press **Play** on **Scene2**. Within a second or two the cFS console switches from "waiting for Unity telemetry" to live wakeup logs showing range, phase, and commanded thruster mask.

### 6. Send GO

In a new Mac terminal (not inside the container):

```bash
python3 gnc_cmd.py go
```

The GNC transitions from IDLE to CORRECT to APPROACH and autonomously docks.

For the full troubleshooting guide and EVS log reference, see [OPERATIONS.md](OPERATIONS.md).

---

## Repository Layout

```
cFS_Project/
├── cfs-dev.sh                        ← start Docker dev container
├── gnc_cmd.py                        ← ground command sender
├── Dockerfile                        ← cFS build environment
├── console.html                      ← browser-based telemetry viewer
├── PROJECT.md                        ← architecture and design reference
├── OPERATIONS.md                     ← step-by-step run guide and troubleshooting
├── cFS/                              ← NASA cFS (git submodule)
│   ├── apps/
│   │   └── gnc_app/                  ← custom GNC flight software application
│   │       └── fsw/
│   │           ├── src/              ← gnc_app.c, gnc_app_udp.c
│   │           ├── inc/              ← gnc_app.h, gnc_app_tbl.h
│   │           └── tables/           ← gnc_param_tbl.c (default gain table)
│   └── sample_defs/
│       └── tables/
│           └── sch_lab_table.c       ← 10 Hz tick; GNC wakeup at 1 Hz
└── cFS_DockingSim/                   ← Unity 6 project
    └── Assets/
        ├── VehicleState.cs
        ├── RCSModel.cs               ← 12-thruster model; cFS integration hook
        ├── ClohessyWiltshire.cs      ← orbital differential gravity
        ├── RelativeNav.cs            ← approach state (range, closing speed, etc.)
        ├── DockingDetector.cs        ← contact detection
        ├── ApproachCorridor.cs       ← 15° cone geometry and corridor check
        ├── DockingHUD.cs             ← OnGUI operator display
        ├── DockingCamera.cs          ← articulated chase camera
        ├── RateDamping.cs            ← proportional rate-nulling controller
        ├── ScenarioReset.cs          ← Backspace reset
        ├── TelemetryLogger.cs        ← 10 Hz CSV log to project root
        ├── UdpTelemetrySender.cs     ← 10 Hz telemetry to cFS (port 5005)
        └── UdpCommandReceiver.cs     ← timed-burn commands from cFS (port 5006)
```

---

## Journal

Progress snapshots as the project develops.

**Unity scene with Crew Dragon and ISS models**

![Unity scene — front view](Docs/Unity_Scene_img1.png)

![Unity scene — approach view](Docs/Unity_Scene_img2.png)

**Blender shading work on the vehicle models**
I was able to import Kerbal Space Program models into Blender instead of creating 3D models from scratch. However, there was plenty of issues to still fix. For example, it took a while to figure out why part of the ship was completely chrome. Turns out a value in the shading configuration was set to max.

![Blender shading](Docs/Blender_Shading_Struggles.png)
Also next I need to figure out why my shading and surfaces didn't transfer to Unity.

**June 8th Update**
I think I finally fixed my git history and am now using Git LFS for large files.

The scene in Unity looks a lot better now with materials and textures (mostly) correctly added in

![Unity scene — close view](Docs/Unity_Scene_img3.png)

![Unity scene — far view](Docs/Unity_Scene_img4.png)

**June 23rd Update: RCS Plumes and Multi-Camera System**

I was getting tired of math and deep issues with GNC so I just went for two big visual and usability improvements.

First, proper RCS thruster plumes. Each of the 16 Draco thrusters now emits a particle-based exhaust plume driven live from the throttle state, plus a mesh-based glow cone that fades in and out smoothly when a thruster fires or cuts off. The thruster model was also updated to be physically accurate — Draco thrusters are binary (on/off at full thrust), so the pseudo-inverse allocator now snaps outputs to either full power or zero rather than fractional levels. Torque cancellation that was previously handled by proportional throttles is now left to the rate damping controller, which is more representative of how real Dragon GNC works.

Second, a multi-camera system. The sim now has four switchable cameras: the original nose/docking cam (key `1`), two manually-placed ISS robotic cameras that can pan and zoom (keys `2` and `3`), and a third-person chase cam that smoothly follows Dragon from behind (key `4`). A small HUD in the corner always shows which camera is active. The screenshot below is the ISS approach camera looking out at Dragon on final approach with plumes firing.

![ISS approach camera — Dragon on approach with RCS plumes](Docs/Unity_Scene_img5.png)
