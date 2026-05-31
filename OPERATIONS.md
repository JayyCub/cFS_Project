# DockingSim — Operations Guide

How to build, run, and operate the cFS + Unity docking simulation.

---

## Prerequisites

| Tool | Purpose |
|------|---------|
| Docker Desktop | Runs the cFS build and runtime environment |
| Unity 6 | Runs the physics simulation and renders the scene |
| Python 3 | Sends ground commands to cFS via `gnc_cmd.py` |

No other dependencies. No external Unity packages.

---

## Directory Layout

```
cFS_Project/
├── cfs-dev.sh          ← start the Docker container
├── gnc_cmd.py          ← send ground commands to cFS
├── PROJECT.md          ← architecture and design reference
├── OPERATIONS.md       ← this file
├── cFS/                ← NASA cFS source (mounted into container at /cfs)
└── cFS_DockingSim/     ← Unity project (open in Unity Editor)
```

---

## Step 1 — Start the Docker Container

```bash
cd cFS_Project
./cfs-dev.sh
```

This builds the Docker image on first run (takes ~1 minute), then drops you into a bash shell inside the container. The `cFS/` directory is mounted at `/cfs` so any edits you make on the Mac are immediately visible inside the container without rebuilding the image.

**Ports exposed by the container:**

| Port | Direction | Purpose |
|------|-----------|---------|
| 5005/udp | Mac → container | Unity telemetry → cFS |
| 1234/udp | Mac → container | Ground commands (CI_LAB uplink) |
| *(internal)* | container → Mac | cFS commands → Unity via `host.docker.internal:5006` |

> If the container fails to start because port 5005 or 1234 is already in use, a previous container is still running. Kill it with `docker rm -f cfs-dev`.

---

## Step 2 — Build cFS

Run this **inside the container** (the shell you got from `./cfs-dev.sh`):

```bash
make native_std.install
```

This compiles all cFS apps including `gnc_app` and installs the binaries to `/build-native_std/exe/cpu1/`. The first build takes 2–3 minutes; incremental rebuilds of just `gnc_app` take a few seconds.

**You only need to rebuild when you change C source files.** Unity changes, Python scripts, and table files do not require a rebuild.

---

## Step 3 — Run cFS

Still inside the container:

```bash
cd /build-native_std/exe/cpu1
./core-cpu1
```

You should see cFS boot messages followed by app initialization events. Watch for these lines confirming `gnc_app` is healthy:

```
GNC_APP initialized. Recv port 5005, Cmd port 5006. Guidance INHIBITED — send GO to start.
GNC_APP UDP: listening on port 5005
GNC_APP: command socket ready → 192.168.x.x:5006
```

cFS then prints a wakeup log line every second:

```
GNC #1 | waiting for Unity telemetry...
GNC #2 | waiting for Unity telemetry...
```

This is normal — guidance is inhibited and Unity is not playing yet. **Leave cFS running and move to Step 4.**

---

## Step 4 — Start Unity

Open `cFS_DockingSim/` in the Unity Editor and press **Play** on **Scene2**.

Within 1–2 seconds cFS should start receiving telemetry. The wakeup log changes from "waiting for Unity telemetry" to:

```
GNC #3 [IDLE] | Rng=15.23 Spd=0.000 Lat=2.41 Att=0.0 | Cor=0 Dkd=0 | Cmd=0x000 Dur=0.000s
```

The phase shows `[IDLE]` because guidance is pre-latched. No thrusters fire yet. The vehicle drifts only under Clohessy-Wiltshire differential gravity.

---

## Step 5 — Send the GO Command

Open a **new terminal** on your Mac (not inside the container):

```bash
cd cFS_Project
python3 gnc_cmd.py go
```

cFS immediately logs:

```
GNC_APP: GO — guidance ENABLED, starting from CORRECT
GNC mode: IDLE → LAT_CORR (lat=2.41m rng=15.23m)
```

Thrusters start firing. The GNC will:
1. Enter **LATERAL_CORRECT** if lateral offset > 0.5 m — drives the vehicle onto the docking axis
2. Transition to **APPROACH** once lateral offset < 0.5 m — proportional axial closure begins
3. Enter **DOCKED** when contact thresholds are met

---

## Ground Commands

All commands are sent from the Mac terminal using `gnc_cmd.py`. cFS must be running.

```bash
python3 gnc_cmd.py <command>
```

| Command | Effect |
|---------|--------|
| `noop` | Heartbeat. Verifies the command link is alive. Increments CmdCount in HK. |
| `reset` | Zeros HK counters (CmdCount, CmdErrCount, UdpPacketsReceived). |
| `hold` | Immediately freeze at current range. GNC station-keeps — no axial closure. |
| `go` | Release a `hold` or the startup pre-latch. Resumes guidance. |
| `abort` | **Emergency stop.** Sends immediate coast to Unity. All thrust inhibited until you send `go`. |

### Typical Sequence

```bash
# Confirm command link
python3 gnc_cmd.py noop

# Start guidance (required after every cFS restart)
python3 gnc_cmd.py go

# Pause at any point during approach
python3 gnc_cmd.py hold

# Resume
python3 gnc_cmd.py go

# Emergency stop
python3 gnc_cmd.py abort

# Resume after abort
python3 gnc_cmd.py go
```

### What GO Accepts / Rejects

GO is only valid when the GNC is in HOLD or when the AbortLatch is set (startup or post-ABORT). Sending GO while guidance is already running returns an error in EVS:

```
GNC_APP: GO rejected — guidance already active (phase=2)
```

---

## EVS Log Reference

The cFS EVS log (printed to the terminal running `./core-cpu1`) is the primary diagnostic tool.

| Event | EID | Meaning |
|-------|-----|---------|
| `GNC_APP initialized...` | 1 | App started OK. Guidance is pre-latched. |
| `GNC_APP UDP: listening on port 5005` | 5 | Recv socket bound. Packets can now arrive from Unity. |
| `GNC_APP: command socket ready` | 9 | cFS can send commands to Unity. |
| `GNC #N [PHASE] \| Rng=... Cmd=0x... Dur=...` | 2 | 1 Hz wakeup log. Shows phase, nav state, and what was commanded. |
| `GNC mode: X → Y` | 11 | Phase transition. Includes lateral offset and range at the moment of transition. |
| `GNC_APP: NOOP` | 12 | NOOP received and accepted. |
| `GNC_APP: counters reset` | 13 | RESET_COUNTERS accepted. |
| `GNC_APP: HOLD — station-keep at X m` | 14 | HOLD accepted. Range shown for reference. |
| `GNC_APP: GO — guidance ENABLED` | 15 | GO accepted. |
| `GNC_APP: *** ABORT ***` | 16 | ABORT accepted. Severity = CRITICAL. |
| `GNC_APP: cmd len err` | 17 | Command packet had wrong byte count. |
| `GNC_APP: invalid command code` | 18 | Unknown function code received. |
| `GNC_APP UDP: recv error RC=...` | 6 | Recv task got a fatal socket error and exited. Restart cFS. |
| `GNC_APP UDP: wrong packet size` | 6 | Unity sent a packet with the wrong byte count (packet format mismatch). |

---

## Keyboard Controls (Unity)

Controls are active when cFS is **not** in control (before GO, or after ABORT).

| Key | Action |
|-----|--------|
| W / S | Forward / Back (+Z / -Z) |
| A / D | Left / Right (−X / +X) |
| Space | Up (+Y) |
| Ctrl | Down (−Y) |
| R / F | Pitch up / down |
| E / Q | Yaw right / left |
| Z / X | Roll CW / CCW |
| H | Toggle Rate Damping |
| Backspace | Reset scenario (zeroes velocities, returns to start position) |
| Arrow keys | Articulate camera |
| Enter | Center camera |

---

## Troubleshooting

### "Waiting for Unity telemetry" after pressing Play

1. Check that Unity is playing **Scene2** (not SampleScene).
2. Check that port 5005 is not blocked — another cFS instance or process may hold the port. Run `docker rm -f cfs-dev` and restart.
3. Look for `GNC_APP UDP: recv error RC=...` in the EVS log — this means the recv task exited. Restart cFS.
4. Check that `UdpTelemetrySender.cs` has the correct destination IP (`127.0.0.1`) and port (`5005`) in the Unity Inspector.

### Commands not reaching cFS

1. Confirm the container is running: `docker ps` should show `cfs-dev`.
2. Confirm port 1234 is mapped: `docker port cfs-dev` should show `1234/udp -> 0.0.0.0:1234`.
3. Send a NOOP and check EVS: `python3 gnc_cmd.py noop`. If you see EID 12 in the log, the link is up.
4. If using a firewall, allow UDP on port 1234 from localhost.

### cFS → Unity commands not reaching Unity

1. Check for `GNC_APP: command socket ready → ...` in the EVS log. If missing, DNS resolution of `host.docker.internal` failed — restart Docker Desktop.
2. Check that `UdpCommandReceiver.cs` is attached to a GameObject in the scene and has `rcsModel` assigned in the Inspector.
3. Confirm port 5006 is open in the Unity script (`listenPort = 5006`).

### Vehicle oscillates / overshoots after GO

The GNC gains (`GNC_AXIAL_KP`, `GNC_LAT_KP`, etc.) and the physical parameters (`GNC_THRUSTER_FORCE`, `GNC_VEHICLE_MASS`) must match the Unity Inspector values. If `RCSModel.thrusterForce` or the Rigidbody mass has been changed in the Inspector, update the matching constants in `gnc_app.h` and rebuild.

### Rebuilding after a code change

Inside the container:
```bash
make native_std.install
cd /build-native_std/exe/cpu1
./core-cpu1
```

You do not need to restart the container or re-run `./cfs-dev.sh` between rebuilds.
