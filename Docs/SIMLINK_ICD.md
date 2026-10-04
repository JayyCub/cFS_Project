# SimLink ICD — Unity ⇄ cFS interface (v4)

SimLink is the interface between the Unity simulation (the physical world and vehicle hardware) and cFS (the flight software). On the cFS side, only the **SIM_IO** app talks to it. Every other app sees ordinary Software Bus messages, the same way flight apps see device data that a hardware I/O app publishes.

| Item | Source of truth |
|---|---|
| C definition | `cFS/apps/sim_io/fsw/inc/simlink_icd.h` |
| C# definition | `cFS_DockingSim/Assets/SimLinkProtocol.cs` |
| cFS endpoint | `cFS/apps/sim_io` (config table `SIM_IO.CfgTbl`) |
| Unity endpoints | `UdpTelemetrySender.cs` (lock-step master), `UdpCommandReceiver.cs` |
| Unity sensor models | `ChaserSensors.cs` (IMU, star tracker, LIDAR, capture switch) |

Keep the two definitions identical. The C header has `_Static_assert`s on every struct size, so a layout change that isn't mirrored fails the cFS build instead of corrupting data at runtime.

## Transport

| Direction | Frame | UDP port | Rate |
|---|---|---|---|
| Unity → cFS | `SIM_STATE` | 5005 (`SIM_IO.CfgTbl.ListenPort`) | once per GNC cycle (0.2 s sim time) |
| cFS → Unity | `THRUSTER_CMD` | 5006 (`SIM_IO.CfgTbl.DestPort`, host `DestHost`) | one per `SIM_STATE` |

## Framing

All fields are little-endian with no padding.

```
Header (24 B) | payload | Trailer (4 B)
```

**Header**

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 | Sync | `0x324B4C53`, the ASCII bytes `SLK2` on the wire |
| 4 | u16 | Version | `4` |
| 6 | u16 | Type | `1` = SIM_STATE, `3` = THRUSTER_CMD (`2`, the v2 WRENCH_CMD, is retired) |
| 8 | u32 | Seq | Unity's GNC-cycle counter, starting at 1. A THRUSTER_CMD echoes the Seq of the SIM_STATE it answers |
| 12 | u32 | Length | Total frame length in bytes, including header and trailer |
| 16 | f64 | SimTime_s | Simulation time at the cycle boundary. A THRUSTER_CMD echoes it |

**Trailer**

| Offset | Type | Field |
|---|---|---|
| Length−4 | u16 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) over every preceding byte |
| Length−2 | u16 | spare (0) |

The receiver drops any frame whose sync word, version, type, length or CRC is wrong. It counts the drop (SIM_IO HK `RxBadFrame`/`RxBadCrc`; Unity `UdpCommandReceiver.FramesBad`) and never applies the frame.

## SIM_STATE payload (88 B, frame 116 B)

Raw sensor readings, sampled at the cycle boundary by `ChaserSensors.cs` (realism phase 4). Nothing computed from truth goes to flight software: the cFS **NAV** app turns these readings into the navigation solution GNC flies on. Unity's `RelativeNav` still computes the truth values, but only for the HUD, the docking mechanism and the truth log.

| Offset | Type | Field | Units / frame |
|---|---|---|---|
| 0 | f32 | CycleDt_s | Sim seconds one GNC cycle covers |
| 4 | u32 | SensorValid | bit 0 IMU, bit 1 star tracker, bit 2 LIDAR: the device produced a reading this cycle |
| 8 | f32×3 | GyroRate_B | Body angular rate (rad/s) |
| 20 | f32×3 | DeltaV_B | Non-gravitational Δv over the cycle that just ended, body (m/s). Thrust and contact; free fall reads zero |
| 32 | f32×4 | StQuat | Star tracker attitude q_L_B (x, y, z, w): rotates body vectors into LVLH |
| 48 | f32 | RpsRange_m | LIDAR range from its mount to the target port reflector |
| 52 | f32 | RpsAz_rad | Azimuth, atan2(x, z) in the sensor frame (boresight +Z) |
| 56 | f32 | RpsEl_rad | Elevation, asin(y / range) |
| 60 | u32 | MechFlags | bit 0 soft-capture latches engaged |
| 64 | f32×3 | Thc | Crew translation hand controller, body X/Y/Z, −1..+1 |
| 76 | f32×3 | Rhc | Crew rotation hand controller, pitch/yaw/roll, −1..+1 |

**Frames.** Body = the chaser vehicle transform's axes with the origin at the centre of mass, the same frame as the RCS thruster table. LVLH = Unity world axes (mapping below). The LIDAR frame is its mount's transform, the chaser docking port by default.

**Sensor models** (`ChaserSensors` Inspector defaults; cFS `nav_cfg_tbl.c` holds FSW's model of the same sensors):

| Sensor | Noise, 1-sigma | Limits |
|---|---|---|
| Gyro | 1e-5 rad/s per sample, optional bias | none |
| Accelerometer Δv | 1e-5 m/s per cycle, optional bias | CW gravity taken out (`ClohessyWiltshire.LastAccel`) |
| Star tracker | 5e-5 rad (~10 arcsec) per axis | none |
| LIDAR | range 5 mm + 0.1 % of range, az/el 0.5 mrad | 0.3–250 m, 35° half-angle field of view |

Noise comes from a seeded generator, so lock-stepped runs stay repeatable. The Inspector's *Fault injection* switches drop a sensor's valid bit to test NAV's degraded modes.

**World ↔ LVLH mapping** (the same in `ClohessyWiltshire.cs` and the GNC CW feed-forward):

| LVLH axis | World axis |
|---|---|
| Radial (up) | +Y |
| Along-track (velocity direction) | −Z |
| Cross-track | +X |

The chaser starts at −Z and closes toward +Z. That places it on +V-bar, ahead of the station, approaching the forward port like a real Dragon approach to IDA-2.

## THRUSTER_CMD payload (72 B, frame 100 B)

Unchanged since v3.

Valve on-times computed by the cFS **RCS** app. RCS turns GNC's impulse request into on-times: it solves a non-negative least-squares allocation over its thruster table, then applies pulse-width modulation with a minimum impulse bit.

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | NumThrusters (16) |
| 4 | i32 | GncPhase (0 IDLE, 1 CORRECT, 2 APPROACH, 3 DOCKED, 4 HOLD, 5 MANUAL) |
| 8 | f32×16 | OnTime_s[i]: how long valve i stays open from the start of this cycle (0 = closed) |

Unity opens every commanded valve when the frame is applied and closes each one after its own on-time, with full rated thrust in between. Draco thrusters don't throttle. Unity also rejects a negative or non-finite on-time.

Type 2 (the v2 `WRENCH_CMD`, body wrench plus duration) is retired.

## Lock-step protocol

```
Unity FixedUpdate (cycle boundary)            cFS
─────────────────────────────────             ───────────────────────────────────────
Seq++ ; send SIM_STATE(Seq) ───────────────▶ SIM_IO rx task: validate, publish SIM_STATE (SB)
block in WaitForCommand(Seq)                  NAV: wakes on SIM_STATE, runs the filter,
                                                   publishes NAV_SOLUTION(Seq) (SB)
                                              GNC: wakes on NAV_SOLUTION, runs the cycle,
                                                   publishes ACT_REQ(Seq) = impulse wanted (SB)
                                              RCS: allocation + PWM, publishes THRUSTER_CMD(Seq) (SB)
open valves in the same physics step ◀─────── SIM_IO main task: frame and send THRUSTER_CMD(Seq)
run gncCycleSec of physics, repeat
```

- **The sim clock is the master.** cFS never schedules the control cycle from a wall-clock timer: `SCH_LAB` only sends the 1 Hz HK requests. Given the same initial conditions, every run produces the same result.
- **Engage and disengage.** Unity starts free-running and engages lock-step as soon as cFS answers the previous cycle. If an answer takes longer than `lockStepTimeoutMs` (500 ms real time), Unity logs a warning, drops to free-running and re-engages on its own once cFS catches up.
- **Link loss in cFS.** If GNC sees no SIM_STATE for `TlmLossTimeoutSec` of wall time, measured on the 1 Hz HK tick, it latches ABORT. When the link returns, GNC reports it but stays inhibited until the operator sends GO.
- **Sequence accounting.** SIM_IO HK counts `RxSeqGaps` (Unity ran free or a datagram was lost), `RxSeqRestarts` (Unity play-mode restarted) and `TxStaleSeq` (flight software answered an older cycle).

## cFS Software Bus messages

| MID | Name | Producer → consumer |
|---|---|---|
| 0x1896 | SIM_IO_CMD | ground → SIM_IO (NOOP=0, RESET_COUNTERS=1) |
| 0x1897 | SIM_IO_SEND_HK | SCH_LAB (1 Hz) → SIM_IO |
| 0x0896 | SIM_IO_HK_TLM | SIM_IO → TO_LAB |
| 0x0897 | SIM_IO_SIM_STATE | SIM_IO → NAV (raw sensors; the NAV cycle trigger) and → GNC (hand controllers, capture switch, link watchdog) |
| 0x08D1 | NAV_SOLUTION | NAV → GNC (the GNC cycle trigger) and → DS recorder |
| 0x18D0 | NAV_CMD | ground → NAV (NOOP=0, RESET_COUNTERS=1, RESET_FILTER=2) |
| 0x18D1 | NAV_SEND_HK | SCH_LAB (1 Hz) → NAV (also where the NAV table is managed) |
| 0x08D0 | NAV_HK_TLM | NAV → TO_LAB (LIDAR fixes used / rejected / missing, filter inits, sigmas) |
| 0x08A2 | RCS_ACT_REQ | GNC → RCS: linear/angular impulse wanted this cycle (body frame) |
| 0x0898 | SIM_IO_THRUSTER_CMD | RCS → SIM_IO (valve on-times) and → GNC (achieved impulse, for the actuator-health check) |
| 0x1899 | RCS_CMD | ground → RCS (NOOP=0, RESET_COUNTERS=1) |
| 0x18A2 | RCS_SEND_HK | SCH_LAB (1 Hz) → RCS (also where the RCS table is managed) |
| 0x0899 | RCS_HK_TLM | RCS → TO_LAB (requests, saturations, minimum-impulse drops/rounds, residual, fired mask) |
