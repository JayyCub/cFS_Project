#!/usr/bin/env python3
"""
SimLink smoke test — stands in for Unity to verify the cFS side end to end:
SIM_IO (framing, lock-step) -> NAV (sensors -> navigation) -> GNC (phases, control law)
-> RCS (allocation, PWM) -> SIM_IO.

Sends SIM_STATE frames (Seq 1..N) exactly the way UdpTelemetrySender.cs does and checks
each is answered by a CRC-valid THRUSTER_CMD echoing the same Seq. Since SimLink v4 the
frames carry raw sensor readings, so a small truth model (class Truth) flies the chaser —
docking port on the approach axis, upright, CW gravity — and produces consistent IMU,
star tracker and LIDAR readings for NAV. It translates (no rotation) under the thruster
commands, from the RCS table directions; responds=False freezes the "hardware". Scenario:

  1. IDLE (guidance inhibited at boot)       -> all valves closed while NAV converges
  2. crew THC +Z (hand controller)           -> GNC MANUAL, approach group T04-T07 firing
  3. sticks released, still MANUAL           -> rate hold only (no translation)
  4. ground GO, chaser drifting in 5 cm/s    -> GNC CORRECT, braking pulses
  5. ground ABORT                            -> DEPART: retreat burn (brake group), then
                                                IDLE once the free drift is passively safe
  6. corrupt frame                           -> dropped, not answered

FDIR (realism phase 3 — LC watchpoints/actionpoints -> SC RTSs, wall-clock paced):
  7. GO, then sim silent 5 s                 -> LC AP 0 -> SC RTS 2 -> GNC ABORT (DEPART)
  8. re-arm (SC RTS 4), GO, silent again     -> aborts again (AP 0 was re-armed)
  9. re-arm, GO, reach APPROACH, then burns
     that change nothing (thrusters dead)    -> LC AP 1 -> SC RTS 3 -> GNC HOLD

NAV fault protection (realism phase 4b):
 10. GO, reach APPROACH, IMU fails           -> NAV invalid 2 s -> LC AP 2 -> RTS 2 ABORT
 11. GO, reach APPROACH, chaser jumps 2 m    -> LIDAR fixes rejected -> LC AP 3 -> RTS 3 HOLD

Approach envelope (realism phase 5):
 12. GO, reach APPROACH, closing at 0.45 m/s
     with the thrusters dead                 -> overspeed -> LC AP 4 -> RTS 3 HOLD
 13. re-arm; chaser 6 m off-axis at 15 m     -> out of corridor -> LC AP 5 -> RTS 2 ABORT

Usage (cFS running, SIM_IO DestHost resolving to this machine, CI_LAB on --ci-port):
    python3 tools/simlink_smoke.py [--host 127.0.0.1]

Wire format: Docs/SIMLINK_ICD.md (mirrors simlink_icd.h / SimLinkProtocol.cs).
"""
import argparse
import math
import socket
import struct
import sys
import time

SYNC = 0x324B4C53
VERSION = 5
TYPE_SIM_STATE = 1
TYPE_THRUSTER_CMD = 3
N_THR = 16

HDR = struct.Struct("<IHHIId")                 # 24 B
STATE = struct.Struct("<fI3f3f4f3f4fI3f3f")    # 104 B
THR = struct.Struct(f"<Ii{N_THR}f")            # 72 B
TRL = struct.Struct("<HH")                     # 4 B
assert HDR.size == 24 and STATE.size == 104 and THR.size == 72

SENSOR_IMU, SENSOR_ST, SENSOR_RPS, SENSOR_RPS_POSE = 0x1, 0x2, 0x4, 0x8
POSE_MAX_RANGE = 30.0
CYCLE_DT = 0.2

# Geometry, as in cFS nav_cfg_tbl.c: target port (= LIDAR reflector) relative to the ISS
# reference point; the chaser docking port / LIDAR sit on body +Z ahead of the CoM.
TARGET_PORT = (0.000005485832, -5.5613275, -16.092)
TARGET_PORT_QUAT = (-0.70668393, 0.7075295, -0.00000087704393, -0.00000014301622)  # q_T_TP

# Thruster push directions and enables from cFS rcs_thr_tbl.c; rated 400 N, 12000 kg
THRUST_N, MASS_KG = 400.0, 12000.0
THR_DIR = [(0, 0, -1)] * 4 + [
    (-0.2241, -0.5, 0.8365), (0.2241, -0.5, 0.8365), (0.2241, 0.5, 0.8365), (-0.2241, 0.5, 0.8365),
    (-0.7071, 0, -0.7071), (0.7071, 0, -0.7071), (0.7071, 0, -0.7071), (-0.7071, 0, -0.7071),
    (0.4330, -0.5, -0.75), (-0.4330, -0.5, -0.75), (-0.4330, 0.5, -0.75), (0.4330, 0.5, -0.75),
]
THR_ENABLED = [False] * 4 + [True] * 12
MEAN_MOTION = 0.00113

PHASES = {0: "IDLE", 1: "CORRECT", 2: "APPROACH", 3: "DOCKED", 4: "HOLD", 5: "MANUAL", 6: "DEPART"}

GNC_CMD_MID = 0x1893
GNC_GO, GNC_ABORT = 3, 4
NAV_CMD_MID = 0x18D0
GNC_NAV_RESET = 2  # NAV RESET_FILTER

SC_CMD_MID = 0x18A9
SC_START_RTS = 4
RTS_FDIR_REARM = 4


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Truth:
    """Chaser held upright on the approach axis. pos = chaser docking port minus target
    port, world axes (z < 0 while short of the port). Velocity changes through the
    thruster commands (thrust(), unless responds is False) and set_vel(); the IMU
    reports the sum as that cycle's Δv."""

    def __init__(self, gap=40.0, lat=(0.0, 0.0)):
        self.pos = [lat[0], lat[1], -gap]
        self.vel = [0.0, 0.0, 0.0]
        self.pending = None
        self.dv_cmd = [0.0, 0.0, 0.0]
        self.imu_ok = True
        self.responds = True

    def thrust(self, on_times):
        """Valve on-times for the coming cycle -> Δv (upright: body = world, no rotation)."""
        if not self.responds:
            return
        for t, d, en in zip(on_times, THR_DIR, THR_ENABLED):
            if en and t > 0:
                for i in range(3):
                    self.dv_cmd[i] += d[i] * THRUST_N * t / MASS_KG

    def set_vel(self, vx=None, vy=None, vz=None):
        v = list(self.vel)
        for i, x in enumerate((vx, vy, vz)):
            if x is not None:
                v[i] = x
        self.pending = v

    def step(self):
        """Advance one cycle; return the non-gravitational Δv (world = body: upright)."""
        new = self.pending or list(self.vel)
        self.pending = None
        new = [new[i] + self.dv_cmd[i] for i in range(3)]
        self.dv_cmd = [0.0, 0.0, 0.0]
        dv = [new[i] - self.vel[i] for i in range(3)]
        # impulse at mid-interval (NAV's model): average old/new velocity over the cycle
        v = [(self.vel[i] + new[i]) / 2 for i in range(3)]
        n, sub = MEAN_MOTION, 10
        h = CYCLE_DT / sub
        for _ in range(sub):
            # CW gravity, relative to the ISS reference (ClohessyWiltshire.cs); the
            # along-track offset of the CoM from the port doesn't enter the equations
            rx = TARGET_PORT[0] + self.pos[0]
            ry = TARGET_PORT[1] + self.pos[1]
            a = (-n * n * rx, 3 * n * n * ry - 2 * n * v[2], 2 * n * v[1])
            for i in range(3):
                v[i] += a[i] * h
                new[i] += a[i] * h
                self.pos[i] += v[i] * h
        self.vel = new
        return dv

    def lidar(self):
        """Range / az / el from the chaser port to the target port; sensor axes = world."""
        p = [-x for x in self.pos]
        rng = math.sqrt(sum(x * x for x in p))
        return rng, math.atan2(p[0], p[2]), math.asin(p[1] / rng)


def sim_state_frame(seq, sim_time, truth, thc=(0, 0, 0), rhc=(0, 0, 0)):
    length = HDR.size + STATE.size + TRL.size
    dv = truth.step()
    rng, az, el = truth.lidar()
    # LIDAR pose: chaser upright with the sensor on world axes, ISS at identity, so the
    # target port's orientation in the sensor frame is just its table quaternion
    pose = SENSOR_RPS_POSE if rng <= POSE_MAX_RANGE else 0
    imu = SENSOR_IMU if truth.imu_ok else 0
    body = HDR.pack(SYNC, VERSION, TYPE_SIM_STATE, seq, length, sim_time) + STATE.pack(
        CYCLE_DT,
        imu | SENSOR_ST | SENSOR_RPS | pose,
        0.0, 0.0, 0.0,                         # gyro: not rotating
        *dv,                                   # IMU Δv over the cycle
        0.0, 0.0, 0.0, 1.0,                    # star tracker: upright (identity)
        rng, az, el,                           # LIDAR
        *TARGET_PORT_QUAT,                     # LIDAR pose (used only when flagged)
        0,                                     # mechanism: not captured
        *thc, *rhc,
    )
    return body + TRL.pack(crc16(body), 0)


def parse_thruster_cmd(data):
    if len(data) != HDR.size + THR.size + TRL.size:
        return None, f"length {len(data)}"
    sync, ver, typ, seq, length, sim_time = HDR.unpack_from(data, 0)
    if sync != SYNC or ver != VERSION or typ != TYPE_THRUSTER_CMD or length != len(data):
        return None, f"header sync={sync:#x} ver={ver} type={typ} len={length}"
    crc, _ = TRL.unpack_from(data, len(data) - TRL.size)
    if crc16(data[: len(data) - TRL.size]) != crc:
        return None, "CRC"
    n, phase, *on = THR.unpack_from(data, HDR.size)
    return {"seq": seq, "sim_time": sim_time, "phase": phase, "on": on, "n": n}, None


def ccsds_cmd(mid, fc, payload=b""):
    body = struct.pack(">HHHBB", mid, 0xC000, 1 + len(payload), fc & 0x7F, 0) + payload
    cksum = 0xFF
    for b in body:
        cksum ^= b
    return body[:7] + bytes([cksum]) + body[8:]


class Link:
    def __init__(self, a):
        self.a = a
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("0.0.0.0", a.listen))
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.rtts = []
        self.failures = 0
        self.truth = Truth()

    def fail(self, msg):
        print("FAIL " + msg)
        self.failures += 1

    def cycle(self, **sticks):
        self.seq += 1
        sim_time = (self.seq - 1) * CYCLE_DT
        t0 = time.perf_counter()
        self.tx.sendto(sim_state_frame(self.seq, sim_time, self.truth, **sticks), (self.a.host, self.a.port))
        self.rx.settimeout(self.a.timeout)
        try:
            while True:
                data, _ = self.rx.recvfrom(512)
                cmd, why = parse_thruster_cmd(data)
                if cmd is None:
                    self.fail(f"Seq {self.seq}: bad THRUSTER_CMD ({why})")
                    continue
                if cmd["seq"] < self.seq:
                    continue
                break
        except socket.timeout:
            self.fail(f"Seq {self.seq}: no answer within {self.a.timeout}s")
            return None
        self.rtts.append((time.perf_counter() - t0) * 1000)
        self.truth.thrust(cmd["on"])
        if cmd["seq"] != self.seq or abs(cmd["sim_time"] - sim_time) > 1e-9 or cmd["n"] != N_THR:
            self.fail(f"Seq {self.seq}: answered Seq {cmd['seq']} simTime {cmd['sim_time']} n {cmd['n']}")
        return cmd

    def ground(self, fc, mid=GNC_CMD_MID, payload=b""):
        """Uplink a command, then keep the lock-step cycling for 1.2 s while CI_LAB
        (which polls its socket) delivers it — a silent gap that long would itself
        count toward the LC link-loss watchpoint."""
        self.tx.sendto(ccsds_cmd(mid, fc, payload), (self.a.host, self.a.ci_port))
        return self.paced(6)

    def paced(self, n):
        """n cycles at the real-time 5 Hz cadence (LC/SC run on wall-clock rate groups)."""
        c = None
        for _ in range(n):
            c = self.cycle() or c
            time.sleep(0.2)
        return c

    def rearm_fdir(self):
        return self.ground(SC_START_RTS, mid=SC_CMD_MID, payload=struct.pack("<HH", RTS_FDIR_REARM, 0))

    def silent(self, sec):
        """Stop the sim (as a paused/crashed Unity would) and wait."""
        time.sleep(sec)


def wait_phase(L, phase, cycles, pace=0.0):
    """Cycle until GNC reports phase (or give up); return the last command."""
    c = None
    for _ in range(cycles):
        c = L.cycle() or c
        if pace:
            time.sleep(pace)
        if c and c["phase"] == phase:
            break
    return c


def approach(L):
    """Cycle until GNC reaches APPROACH (settle gate: 15 steady cycles)."""
    c = None
    for _ in range(40):
        c = L.cycle()
        if c and c["phase"] == 2:
            return c
    L.fail("never reached APPROACH (settle gate)")
    return None


def fired(cmd):
    return [i for i, t in enumerate(cmd["on"]) if t > 0]


def show(label, cmd):
    f = fired(cmd)
    ons = " ".join(f"T{i:02d}={cmd['on'][i]:.3f}" for i in f) or "all closed"
    print(f"  {label:<34} phase={PHASES.get(cmd['phase'], cmd['phase']):<8} {ons}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--listen", type=int, default=5006)
    ap.add_argument("--ci-port", type=int, default=1234)
    ap.add_argument("--timeout", type=float, default=1.0)
    a = ap.parse_args()
    L = Link(a)

    # 1. IDLE — long enough for NAV to initialise from the LIDAR and its velocity
    #    uncertainty to converge (relative nav VALID), so GO in step 4 can steer
    for _ in range(15):
        c = L.cycle()
    if c:
        show("1 IDLE (NAV converging)", c)
        if c["phase"] != 0 or fired(c):
            L.fail("expected IDLE with all valves closed")

    # 2. Crew THC +Z (forward)
    for _ in range(3):
        c = L.cycle(thc=(0, 0, 1))
    if c:
        show("2 THC +Z (crew takeover)", c)
        if c["phase"] != 5:
            L.fail("expected MANUAL after hand-controller deflection")
        if not fired(c) or any(i not in (4, 5, 6, 7) for i in fired(c)):
            L.fail("expected only the approach group T04-T07 for pure +Z")
        if c["on"][4] < 0.02 or c["on"][4] > 0.19:
            L.fail(f"T04 on-time {c['on'][4]:.3f} outside [MinOnTime, MaxOnTime]")

    # 3. Sticks released, zero rates -> nothing to do
    for _ in range(3):
        c = L.cycle()
    if c:
        show("3 sticks released (rate hold)", c)
        if c["phase"] != 5 or fired(c):
            L.fail("expected MANUAL holding with valves closed (vehicle at rest)")

    # 4. GO while drifting in at 5 cm/s -> CORRECT station-keeps axially: braking pulses
    L.truth.set_vel(vz=0.05)
    L.ground(GNC_GO)
    for _ in range(3):
        c = L.cycle()
    if c:
        show("4 GO, drifting in at 0.05 m/s", c)
        if c["phase"] not in (1, 2):
            L.fail("expected CORRECT/APPROACH after GO")
        if not fired(c):
            L.fail("expected corrective pulses")

    # 5. ABORT -> DEPART (retreat on the brake group), then IDLE when passively safe
    L.tx.sendto(ccsds_cmd(GNC_CMD_MID, GNC_ABORT), (a.host, a.ci_port))
    c = wait_phase(L, 6, 10, pace=0.2)
    if c:
        show("5 ABORT (collision avoidance)", c)
        if c["phase"] != 6 or not fired(c) or any(i < 8 for i in fired(c)):
            L.fail("expected DEPART firing only the brake group T08-T15 (retreat)")
    c = wait_phase(L, 0, 40)
    if c:
        show("5b manoeuvre complete", c)
        if c["phase"] != 0 or fired(c):
            L.fail("expected IDLE with valves closed once the retreat was done")

    # 6. Corrupt frame must be dropped, not answered
    bad = bytearray(sim_state_frame(L.seq + 1, L.seq * CYCLE_DT, Truth()))
    bad[40] ^= 0xFF
    L.tx.sendto(bytes(bad), (a.host, a.port))
    L.rx.settimeout(0.5)
    try:
        data, _ = L.rx.recvfrom(512)
        L.fail(f"corrupt frame was answered ({len(data)} bytes)")
    except socket.timeout:
        print("  6 corrupt frame                     dropped (OK)")

    # Stop the retreat drift so later steps start from rest
    L.truth.set_vel(0.0, 0.0, 0.0)
    L.paced(2)

    # 7. Link loss while guidance is active -> LC/SC abort
    L.ground(GNC_GO)
    c = L.paced(3)
    if c and c["phase"] not in (1, 2):
        L.fail("expected guidance active after GO")
    L.silent(5.0)
    c = L.cycle()
    if c:
        show("7 link loss 5 s (LC AP0 -> RTS 2)", c)
        if c["phase"] != 6:
            L.fail("expected the FDIR ABORT's retreat (DEPART) when the link came back")
    wait_phase(L, 0, 40)
    L.truth.set_vel(0.0, 0.0, 0.0)

    # 8. Re-arm, fly, lose the link again -> aborts again only if AP 0 was re-armed
    L.rearm_fdir()
    L.ground(GNC_GO)
    L.paced(3)
    L.silent(5.0)
    c = L.cycle()
    if c:
        show("8 re-armed, link loss again", c)
        if c["phase"] != 6:
            L.fail("expected a second FDIR ABORT — RTS 4 did not re-arm AP 0")
    wait_phase(L, 0, 40)
    L.truth.set_vel(0.0, 0.0, 0.0)

    # 9. Actuator anomaly on approach -> LC AP1 -> RTS 3 HOLD
    L.rearm_fdir()
    L.ground(GNC_GO)
    c = approach(L)
    if c:
        show("9a APPROACH (burning to close)", c)
        L.truth.responds = False                # thrusters dead: burns change nothing
        c = wait_phase(L, 4, 20, pace=0.2)
        if c:
            show("9b no IMU dv (LC AP1 -> RTS 3)", c)
        if not c or c["phase"] != 4:
            L.fail("expected FDIR HOLD after axial burns stopped delivering")
        L.truth.responds = True

    # 10. NAV lost on approach -> LC AP 2 -> RTS 2 ABORT. AP 1 fired in step 9 and is
    #     not re-armed (PASSIVE).
    L.ground(GNC_GO)
    c = approach(L)
    if c:
        L.truth.imu_ok = False                  # RELNAV needs the IMU: NAV goes invalid
        c = wait_phase(L, 6, 20, pace=0.2)
        if c:
            show("10 IMU lost on approach (LC AP2)", c)
        if not c or c["phase"] != 6:
            L.fail("expected FDIR ABORT (DEPART) after NAV went invalid on approach")
        c = wait_phase(L, 0, 40)
        if not c or c["phase"] != 0:
            L.fail("expected the open-loop retreat to finish without the IMU")
        L.truth.imu_ok = True
        L.truth.set_vel(0.0, 0.0, 0.0)

    # 11. LIDAR fixes rejected on approach -> LC AP 3 -> RTS 3 HOLD
    L.paced(5)                                  # IMU back: NAV valid again
    L.ground(GNC_GO)
    c = approach(L)
    if c:
        L.truth.pos[2] -= 2.0                   # chaser "teleports" 2 m back
        c = wait_phase(L, 4, 20, pace=0.2)
        if c:
            show("11 LIDAR jump on approach (LC AP3)", c)
        if not c or c["phase"] != 4:
            L.fail("expected FDIR HOLD after consecutive LIDAR rejections on approach")
    L.paced(15)                                 # NAV re-initialises and converges

    # 12. Overspeed on approach -> LC AP 4 -> RTS 3 HOLD (AP 1 and 3 are PASSIVE)
    L.ground(GNC_GO)
    c = approach(L)
    if c:
        L.truth.responds = False
        L.truth.set_vel(vz=0.45)                # above the 0.36 m/s envelope
        c = wait_phase(L, 4, 20, pace=0.2)
        if c:
            show("12 overspeed on approach (LC AP4)", c)
        if not c or c["phase"] != 4:
            L.fail("expected FDIR HOLD after closing above the approach envelope")
        L.truth.responds = True
        L.truth.set_vel(0.0, 0.0, 0.0)

    # 13. Out of the corridor close in -> LC AP 5 -> RTS 2 ABORT
    L.rearm_fdir()
    L.truth.pos = [6.0, 0.0, -15.0]             # 6 m off-axis at 15 m: cone radius is 4 m
    L.ground(GNC_NAV_RESET, mid=NAV_CMD_MID)    # re-initialise NAV on the new position
    c = wait_phase(L, 6, 60, pace=0.2)
    if c:
        show("13 out of corridor at 15 m (LC AP5)", c)
    if not c or c["phase"] != 6:
        L.fail("expected FDIR ABORT (DEPART) outside the approach corridor")
    wait_phase(L, 0, 40)

    if L.rtts:
        r = sorted(L.rtts)
        print(f"round trip ms: median {r[len(r) // 2]:.2f}, max {r[-1]:.2f} over {len(r)} cycles")
    print("PASS" if L.failures == 0 else f"FAILED ({L.failures})")
    return 0 if L.failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
