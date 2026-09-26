# SimLink ICD — Unity ⇄ cFS interface (v2)

SimLink is the interface between the Unity simulation (the physical world and vehicle hardware) and cFS (the flight software). On the cFS side, only the **SIM_IO** app talks to it. Every other app sees ordinary Software Bus messages, the same way flight apps see device data that a hardware I/O app publishes.

| Item | Source of truth |
|---|---|
| C definition | `cFS/apps/sim_io/fsw/inc/simlink_icd.h` |
| C# definition | `cFS_DockingSim/Assets/SimLinkProtocol.cs` |
| cFS endpoint | `cFS/apps/sim_io` (config table `SIM_IO.CfgTbl`) |
| Unity endpoints | `UdpTelemetrySender.cs` (lock-step master), `UdpCommandReceiver.cs` |

Keep the two definitions identical. The C header has `_Static_assert`s on every struct size, so a layout change that isn't mirrored fails the cFS build instead of corrupting data at runtime.

## Transport

| Direction | Frame | UDP port | Rate |
|---|---|---|---|
| Unity → cFS | `SIM_STATE` | 5005 (`SIM_IO.CfgTbl.ListenPort`) | once per GNC cycle (0.2 s sim time) |
| cFS → Unity | `WRENCH_CMD` | 5006 (`SIM_IO.CfgTbl.DestPort`, host `DestHost`) | one per `SIM_STATE` |

## Framing

All fields are little-endian with no padding.

```
Header (24 B) | payload | Trailer (4 B)
```

**Header**

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 | Sync | `0x324B4C53`, the ASCII bytes `SLK2` on the wire |
| 4 | u16 | Version | `2` |
| 6 | u16 | Type | `1` = SIM_STATE, `2` = WRENCH_CMD |
| 8 | u32 | Seq | Unity's GNC-cycle counter, starting at 1. A WRENCH_CMD echoes the Seq of the SIM_STATE it answers |
| 12 | u32 | Length | Total frame length in bytes, including header and trailer |
| 16 | f64 | SimTime_s | Simulation time at the cycle boundary. A WRENCH_CMD echoes it |

**Trailer**

| Offset | Type | Field |
|---|---|---|
| Length−4 | u16 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection) over every preceding byte |
| Length−2 | u16 | spare (0) |

The receiver drops any frame whose sync word, version, type, length or CRC is wrong. It counts the drop (SIM_IO HK `RxBadFrame`/`RxBadCrc`; Unity `UdpCommandReceiver.FramesBad`) and never applies the frame.

## SIM_STATE payload (80 B, frame 108 B)

Phase 1 of the realism roadmap still sends *truth-derived* relative navigation. Phase 4 replaces this payload with raw sensor measurements (IMU, star tracker, relative sensor) and moves navigation into a cFS NAV app.

| Offset | Type | Field | Units / frame |
|---|---|---|---|
| 0 | f32 | CycleDt_s | Sim seconds one GNC cycle covers |
| 4 | f32 | Range_m | Distance between port faces |
| 8 | f32 | ClosingSpeed_ms | Positive when closing |
| 12 | f32 | LateralOffset_m | Port distance from the docking axis |
| 16 | f32 | AttitudeError_deg | Cone angle between the port axes |
| 20 | f32×3 | RelPos | Chaser origin minus target origin, world axes (m) |
| 32 | f32×3 | RelVel | Chaser velocity minus target velocity, world axes (m/s) |
| 44 | f32×3 | AngVel | Chaser rate in the chaser docking-port frame (rad/s) |
| 56 | u32 | Flags | bit 0 in-corridor, bit 1 docked |
| 60 | f32×3 | Pitch/Yaw/RollError_deg | Per-axis error in [−180, 180]; 0 = aligned |
| 72 | f32×2 | LatOffset_X/Y | Signed port lateral offset, world X/Y (m) |

**World ↔ LVLH mapping** (the same in `ClohessyWiltshire.cs` and the GNC CW feed-forward):

| LVLH axis | World axis |
|---|---|
| Radial (up) | +Y |
| Along-track (velocity direction) | −Z |
| Cross-track | +X |

The chaser starts at −Z and closes toward +Z. That places it on +V-bar, ahead of the station, approaching the forward port like a real Dragon approach to IDA-2.

## WRENCH_CMD payload (32 B, frame 60 B)

Phase 2 of the realism roadmap replaces this with per-thruster on-times computed in cFS.

| Offset | Type | Field |
|---|---|---|
| 0 | f32×3 | Fx, Fy, Fz: body-frame force (N) |
| 12 | f32×3 | Tx, Ty, Tz: body-frame torque (N·m) |
| 24 | f32 | Duration_s (0 = coast) |
| 28 | i32 | GncPhase (0 IDLE, 1 CORRECT, 2 APPROACH, 3 DOCKED, 4 HOLD) |

## Lock-step protocol

```
Unity FixedUpdate (cycle boundary)            cFS
─────────────────────────────────             ───────────────────────────────────────
Seq++ ; send SIM_STATE(Seq) ───────────────▶ SIM_IO rx task: validate, publish SIM_STATE (SB)
block in WaitForCommand(Seq)                  GNC: wakes on SIM_STATE, runs the cycle,
                                                   publishes WRENCH_CMD(Seq) (SB)
apply command in the same physics step ◀───── SIM_IO main task: frame and send WRENCH_CMD(Seq)
run gncCycleSec of physics, repeat
```

- **The sim clock is the master.** cFS never schedules the control cycle from a wall-clock timer: `SCH_LAB` only sends the 1 Hz HK requests. Given the same initial conditions, every run produces the same result.
- **Engage and disengage.** Unity starts free-running and engages lock-step as soon as cFS answers the previous cycle. If an answer takes longer than `lockStepTimeoutMs` (500 ms real time), Unity logs a warning, drops to free-running and re-engages on its own once cFS catches up.
- **Link loss in cFS.** If GNC sees no SIM_STATE for `TlmLossTimeoutSec` of wall time, measured on the 1 Hz HK tick, it latches ABORT. When the link returns, GNC reports it but stays inhibited until the operator sends GO.
- **Sequence accounting.** SIM_IO HK counts `RxSeqGaps` (Unity ran free or a datagram was lost), `RxSeqRestarts` (Unity play-mode restarted) and `TxStaleSeq` (GNC answered an older cycle).

## cFS Software Bus messages (SIM_IO)

| MID | Name | Producer → consumer |
|---|---|---|
| 0x1896 | SIM_IO_CMD | ground → SIM_IO (NOOP=0, RESET_COUNTERS=1) |
| 0x1897 | SIM_IO_SEND_HK | SCH_LAB (1 Hz) → SIM_IO |
| 0x0896 | SIM_IO_HK_TLM | SIM_IO → TO_LAB |
| 0x0897 | SIM_IO_SIM_STATE | SIM_IO → GNC (the GNC cycle trigger) |
| 0x0898 | SIM_IO_WRENCH_CMD | GNC → SIM_IO |
