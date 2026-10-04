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
├── Docs/
│   ├── PROJECT.md      ← architecture and design reference
│   ├── OPERATIONS.md   ← this file
│   ├── DEV_REFERENCE.md
│   ├── RCS_THRUSTER_REFERENCE.md
│   └── ATTITUDE_AUTOPILOT_GUIDE.md
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

You should see cFS boot messages followed by app initialization events. Watch for these lines confirming the three mission apps are healthy:

```
SIM_IO initialized: SimLink v3, rx port 5005, tx host.docker.internal:5006
RCS initialized: NNLS allocation + PWM over table /cf/rcs_thr_tbl.tbl
GNC_APP initialized (lock-step on SIM_STATE). Guidance INHIBITED — send GO to start.
SIM_IO: listening for SimLink frames on port 5005
```

Nothing else happens until Unity is playing. GNC is lock-stepped to the simulation, so with no Unity frames it simply waits. **Leave cFS running and move to Step 4.**

---

## Step 4 — Start Unity

Open `cFS_DockingSim/` in the Unity Editor and press **Play** on **Scene2**.

Within a cycle or two the Unity console shows `[UdpTelemetrySender] cFS answering — lock-step ENGAGED`. The cFS console stays quiet: per-cycle GNC state goes out as the STATE telemetry packet (ground console, flight data recorder), not as an event line. To see the old one-line-per-cycle trace, send `python3 gnc_cmd.py trace-on`:

```
GNC #3 [IDLE] Rng=15.23 Spd=0.000 Lat=2.410 PYR=0.0/0.0/0.0 W=0.000/0.000/0.000 C0D0 P=0/0/0 L=0/0/0
```

`P` and `L` are the linear and angular impulse GNC requested from the RCS app. The phase shows `[IDLE]` because guidance is pre-latched, so no thrusters fire yet. The vehicle drifts only under Clohessy-Wiltshire differential gravity.

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
1. Enter **CORRECT** if lateral offset > 1.5 m — drives the vehicle onto the docking axis
2. Transition to **APPROACH** once lateral offset < 1.0 m — proportional axial closure begins, capped at 0.3 m/s
3. Enter **HOLD** automatically at 20 m and 3 m range (autonomous hold points) — station-keeps and waits for `GO`; the axial closing-speed cap tightens to 0.1 m/s after the 20 m hold
4. Enter **DOCKED** when contact thresholds are met

---

## Ground Commands

All commands are sent from the Mac terminal using `gnc_cmd.py`. cFS must be running.

```bash
python3 gnc_cmd.py <command>
```

| Command | Effect |
|---------|--------|
| `noop` | Heartbeat. Verifies the command link is alive. Increments CmdCount in HK. |
| `reset` | Zeros HK counters (CmdCount, CmdErrCount, SimStateCount). |
| `hold` | Immediately freeze at current range. GNC station-keeps — no axial closure. |
| `go` | Release a `hold` or the startup pre-latch. Resumes guidance. |
| `abort` | **Emergency stop.** Sends immediate coast to Unity. All thrust inhibited until you send `go`. |
| `rearm` | Re-arm fault protection (SC RTS 4) after an automatic ABORT or HOLD. See [Fault Protection](#fault-protection). |
| `trace-on` / `trace-off` | Show / hide the per-cycle `GNC #` line in the cFS console (EVS DEBUG events for GNC_APP). |
| `nav-reset` | Drop NAV's translation filter; it re-initialises from the next LIDAR fix. NAV also does this by itself after 2 s of LIDAR fixes it disagrees with (for example after a scenario reset). |

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
| `SIM_IO: listening for SimLink frames on port 5005` | SIM_IO 5 | Recv socket bound. Frames can now arrive from Unity. |
| `RCS initialized: ...` | RCS 1 | Thruster table loaded; allocation ready. |
| `GNC #N [PHASE] Rng=... P=... L=...` | 2 | One per lock-step cycle, only after `gnc_cmd.py trace-on` (DEBUG). Phase, nav state, and the impulse requested from RCS. |
| `GNC mode: X → Y` | 11 | Phase transition. Includes lateral offset and range at the moment of transition. |
| `GNC_APP: NOOP` | 12 | NOOP received and accepted. |
| `GNC_APP: counters reset` | 13 | RESET_COUNTERS accepted. |
| `GNC_APP: HOLD — station-keep at X m` | 14 | HOLD accepted. Range shown for reference. |
| `GNC_APP: GO — guidance ENABLED` | 15 | GO accepted. |
| `GNC_APP: *** ABORT ***` | 16 | ABORT accepted. Severity = CRITICAL. |
| `GNC_APP: cmd len err` | 17 | Command packet had wrong byte count. |
| `GNC_APP: invalid command code` | 18 | Unknown function code received. |
| `GNC_APP: parameter table updated` | 19 | An uplinked table image was activated — new gains are live. |
| `GNC_APP: HOLD POINT 1 — braking to ...` | 21 | Autonomous outer hold point (20 m default) reached during APPROACH. |
| `GNC_APP: HOLD POINT 2 — braking to ...` | 22 | Autonomous inner hold point (3 m default) reached during APPROACH. |
| `SIM_IO: rejected N-byte frame (bad ...)` | 6 | Unity and cFS disagree on the SimLink format — rebuild both from the same commit. |
| `NAV: translation filter initialised from LIDAR at range X m` | NAV 9 | First LIDAR fix (or after `nav-reset`). Relative nav becomes VALID a few seconds later, once the velocity estimate has converged. |
| `NAV: relative nav VALID / INVALID …` | NAV 11 | Whether GNC may steer on NAV's position and velocity. INVALID = translation coasts (attitude is still held). |
| `NAV: filter disagreed with LIDAR … re-initialised` | NAV 10 | 10 LIDAR fixes in a row failed the innovation gate, so NAV trusted the sensor again. Expected once after a scenario reset (Backspace). |
| `NAV: LIDAR tracking / lost the target port` | NAV 13 | Target port entered or left the LIDAR's field of view or range (0.3–250 m). Inside 0.3 m NAV dead-reckons on the IMU. |
| `GNC_APP: NAV relative solution NOT usable — translation coasts` | 30 | GNC's side of NAV 11. Its `usable` partner follows when NAV recovers. |
| `GNC_APP: CREW MANUAL TAKEOVER (hand controller)` | 29 | A hand-controller key was pressed; GNC is flying the sticks (MANUAL) until GO. |
| `GNC_APP: sim link restored at Seq N after N s` | 28 | SIM_STATEs are flowing again after a gap. Guidance stays however FDIR left it. |
| `SC 73: RTS Number 001 Started` … `LC 28: Set LC state command: new state = 1` | — | Boot: fault protection armed. If these are missing, FDIR is off. |
| `LC 1000: GNC sim link loss: ABORT : AP = 0 …` | LC 1000 | Fault protection saw ≥ 2 s of sim silence and started RTS 2 (GNC ABORT). |
| `LC 1001: Axial under-delivery: HOLD : AP = 1 …` | LC 1001 | Axial burns delivered < 50 % of predicted Δv for 3 cycles on approach; RTS 3 (GNC HOLD). |
| `LC 60: AP failed while passive` | LC 60 | The fault is still present but the response already ran. Run `rearm` once recovered. |
| `HS …: App Monitor Failure: APP:(NAME): Action: Event Only` | HS | A flight app stopped running (hung), not just a paused sim. |

---

## Fault Protection

cFS watches for two faults on its own. The response is the same command you would send, and it arrives as an event in the log and on the ground console's FDIR panel:

| Fault | Detected when | Automatic response |
|-------|---------------|--------------------|
| Sim link loss | No SIM_STATE for 2 s while not docked and not already aborted (fires about 4 s after Unity stops) | **ABORT** |
| Axial thruster under-delivery | During APPROACH, 3 cycles in a row where the accelerometer measures less than half the Δv the axial burns should have given | **HOLD** |

After an automatic response, recover the way you would from your own ABORT or HOLD (fix the cause, then `go`), **and** send `python3 gnc_cmd.py rearm`. A response only fires once until it is re-armed. The ground console's FDIR panel shows each response as `ACTIVE` (armed) or `PASSIVE` (fired, needs re-arm). Pausing Unity for more than a few seconds while guidance is active counts as link loss by design. Expect an ABORT when you unpause.

How it works and how to change it: Docs/DEV_REFERENCE.md, *Fault Protection*.

---

## Navigation

Since realism phase 4, cFS does its own navigation. Unity sends raw sensor readings (gyro, accelerometer Δv, star tracker, LIDAR range and bearing to the target port, capture switch) and the cFS **NAV** app estimates where the docking port is. Two consequences for flying:

- **After pressing Play, GNC needs a few seconds.** NAV initialises from the first LIDAR fix and reports relative nav VALID once its velocity estimate has converged (about 2–6 s, longer from far away). Until then the ground console's NAV panel reads `CONVERGING`, and a GO only holds attitude.
- **The HUD and the ground console can differ slightly.** The Unity HUD shows truth. The ground console shows what flight software believes: millimetres apart up close, a centimetre or two at 40 m.

The ground console's NAV panel shows whether relative nav is valid, the position and velocity uncertainty (σ), how many LIDAR fixes were used or rejected, and how far off the last one was (innovation, in σ). Values under about 3 σ are normal. To test degraded modes, tick the *Fault injection* boxes on the `ChaserSensors` component (added to the UdpTelemetrySender object at runtime) in Play mode.

---

## Flight Data

- **Ground console** (`python3 ground_console.py`, http://localhost:8080) writes every per-cycle STATE packet to `run_logs/gnc_state_<time>.csv`.
- **Onboard recorder (DS)** writes `cf/fdr_gnc<seq>.dat` (STATE, NAV solution, valve commands, GNC/NAV/RCS/SIM_IO/LC/SC HK) and `cf/fdr_evs<seq>.dat` (all events) next to `core-cpu1`. Each run overwrites the previous one, so copy the files off first to keep them. Decode them with:

```bash
python3 tools/fdr_decode.py build-native_std/exe/cpu1/cf/fdr_gnc00000001.dat
# -> .gnc_state.csv, .nav_solution.csv, .thruster_cmd.csv, .gnc_hk.csv, .nav_hk.csv  (fdr_evs -> .events.txt)
```

---

## Keyboard Controls (Unity)

The flight keys are the **crew hand controllers**. They don't fire thrusters directly: every GNC cycle Unity sends the stick state to cFS, and pressing any of them hands control to the crew (GNC → **MANUAL**) from any undocked phase, even before GO. Translation keys are an acceleration command. Rotation keys are a rate command with **rate hold**: when you let go, flight software damps the rotation to zero. Send `GO` to hand control back to the autopilot. Manual flight requires cFS to be running.

| Key | Action |
|-----|--------|
| W / S | THC forward / back (+Z / −Z) |
| A / D | THC left / right (−X / +X) |
| Space | THC up (+Y) |
| Ctrl | THC down (−Y) |
| R / F | RHC pitch up / down |
| E / Q | RHC yaw right / left |
| Z / X | RHC roll CW / CCW |
| T | Toggle force suppression (debug) |
| H | Toggle legacy Unity rate damping (inactive while cFS is connected) |
| Backspace | Reset scenario (zeroes velocities, returns to start position) |
| 1 / 2 / 3 / 4 | Switch camera (nose/docking, ISS cam A/B, chase cam) |
| Arrow keys | Articulate active camera |
| Enter | Center camera |
| `` ` `` (backtick) | Toggle single-thruster test mode (bench test, bypasses cFS) |
| F8 | Run the thruster hardware bench test (run with cFS stopped) |

---

## Troubleshooting

### Unity never logs "lock-step ENGAGED" after pressing Play

1. Check that Unity is playing **Scene2** (not SampleScene).
2. Check that port 5005 is not blocked — another cFS instance or process may hold the port. Run `docker rm -f cfs-dev` and restart.
3. Look for `SIM_IO: rejected ... frame` or `rx error` in the EVS log. A rejected frame means the Unity and cFS builds disagree on the SimLink version.
4. Check that `UdpTelemetrySender.cs` has the correct destination IP (`127.0.0.1`) and port (`5005`) in the Unity Inspector.

### Commands not reaching cFS

1. Confirm the container is running: `docker ps` should show `cfs-dev`.
2. Confirm port 1234 is mapped: `docker port cfs-dev` should show `1234/udp -> 0.0.0.0:1234`.
3. Send a NOOP and check EVS: `python3 gnc_cmd.py noop`. If you see EID 12 in the log, the link is up.
4. If using a firewall, allow UDP on port 1234 from localhost.

### cFS → Unity commands not reaching Unity

1. Check the `SIM_IO initialized ... tx host.docker.internal:5006` event. A `cannot resolve` error means DNS resolution of `host.docker.internal` failed — restart Docker Desktop.
2. Check that `UdpCommandReceiver.cs` is attached to a GameObject in the scene and has `rcsModel` assigned in the Inspector.
3. Confirm port 5006 is open in the Unity script (`listenPort = 5006`).

### Vehicle oscillates / overshoots after GO

The GNC gains (`AxialKp`, `LatKp`, etc.) and mass properties (`VehicleMass`, `Inertia_kgm2`) live in `GNC_ParamTbl_t` (`gnc_param_tbl.c`). Thrust and thruster geometry live in the RCS table (`rcs_thr_tbl.c`). They are flight software's *model* of the vehicle. Small differences from the Unity scene are absorbed by the closed loop, but large ones (for example, after moving thrusters) should be fixed: regenerate the RCS table with RCSModel's "Log cFS thruster table" context menu and rebuild, or uplink a corrected table image to a running system.

### Rebuilding after a code change

Inside the container:
```bash
make native_std.install
cd /build-native_std/exe/cpu1
./core-cpu1
```

You do not need to restart the container or re-run `./cfs-dev.sh` between rebuilds.
