#!/usr/bin/env python3
"""
SimLink smoke test — stands in for Unity to verify the cFS side end to end:
SIM_IO (framing, lock-step) -> GNC (phases, control law) -> RCS (allocation, PWM) -> SIM_IO.

Sends SIM_STATE frames (Seq 1..N) exactly the way UdpTelemetrySender.cs does and checks
each is answered by a CRC-valid THRUSTER_CMD echoing the same Seq. Scenario:

  1. IDLE (guidance inhibited at boot)       -> all valves closed
  2. crew THC +Z (hand controller)           -> GNC MANUAL, approach group T04-T07 firing
  3. sticks released, still MANUAL           -> rate hold only (no translation)
  4. ground GO, 0.2 m lateral offset         -> GNC CORRECT, lateral + braking pulses
  5. ground ABORT                            -> IDLE, all valves closed
  6. corrupt frame                           -> dropped, not answered

FDIR (realism phase 3 — LC watchpoints/actionpoints -> SC RTSs, wall-clock paced):
  7. GO, then sim silent 5 s                 -> LC AP 0 -> SC RTS 2 -> GNC ABORT (IDLE)
  8. re-arm (SC RTS 4), GO, silent again     -> aborts again (AP 0 was re-armed)
  9. re-arm, GO, reach APPROACH, then burns
     that change nothing (frozen closing)    -> LC AP 1 -> SC RTS 3 -> GNC HOLD

Usage (cFS running, SIM_IO DestHost resolving to this machine, CI_LAB on --ci-port):
    python3 tools/simlink_smoke.py [--host 127.0.0.1]

Wire format: Docs/SIMLINK_ICD.md (mirrors simlink_icd.h / SimLinkProtocol.cs).
"""
import argparse
import socket
import struct
import sys
import time

SYNC = 0x324B4C53
VERSION = 3
TYPE_SIM_STATE = 1
TYPE_THRUSTER_CMD = 3
N_THR = 16

HDR = struct.Struct("<IHHIId")                 # 24 B
STATE = struct.Struct("<5f3f3f3fI3f2f3f3f")    # 104 B
THR = struct.Struct(f"<Ii{N_THR}f")            # 72 B
TRL = struct.Struct("<HH")                     # 4 B
assert HDR.size == 24 and STATE.size == 104 and THR.size == 72

PHASES = {0: "IDLE", 1: "CORRECT", 2: "APPROACH", 3: "DOCKED", 4: "HOLD", 5: "MANUAL"}

GNC_CMD_MID = 0x1893
GNC_GO, GNC_ABORT = 3, 4

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


def sim_state_frame(seq, sim_time, rng=30.0, lat=(0.0, 0.0), closing=0.0, thc=(0, 0, 0), rhc=(0, 0, 0)):
    length = HDR.size + STATE.size + TRL.size
    body = HDR.pack(SYNC, VERSION, TYPE_SIM_STATE, seq, length, sim_time) + STATE.pack(
        0.2,                                   # CycleDt_s
        rng, closing, (lat[0] ** 2 + lat[1] ** 2) ** 0.5, 0.0,
        lat[0], lat[1], -rng,                  # RelPos
        0.0, 0.0, closing,                     # RelVel
        0.0, 0.0, 0.0,                         # AngVel
        1,                                     # Flags: in corridor
        0.0, 0.0, 0.0,                         # Pitch/Yaw/Roll error
        lat[0], lat[1],                        # LatOffset X/Y
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

    def fail(self, msg):
        print("FAIL " + msg)
        self.failures += 1

    def cycle(self, **state):
        self.seq += 1
        sim_time = (self.seq - 1) * 0.2
        t0 = time.perf_counter()
        self.tx.sendto(sim_state_frame(self.seq, sim_time, **state), (self.a.host, self.a.port))
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
        if cmd["seq"] != self.seq or abs(cmd["sim_time"] - sim_time) > 1e-9 or cmd["n"] != N_THR:
            self.fail(f"Seq {self.seq}: answered Seq {cmd['seq']} simTime {cmd['sim_time']} n {cmd['n']}")
        return cmd

    def ground(self, fc, mid=GNC_CMD_MID, payload=b"", **state):
        """Uplink a command, then keep the lock-step cycling for 1.2 s while CI_LAB
        (which polls its socket) delivers it — a silent gap that long would itself
        count toward the LC link-loss watchpoint."""
        self.tx.sendto(ccsds_cmd(mid, fc, payload), (self.a.host, self.a.ci_port))
        return self.paced(6, **state)

    def paced(self, n, **state):
        """n cycles at the real-time 5 Hz cadence (LC/SC run on wall-clock rate groups)."""
        c = None
        for _ in range(n):
            c = self.cycle(**state) or c
            time.sleep(0.2)
        return c

    def rearm_fdir(self, **state):
        return self.ground(SC_START_RTS, mid=SC_CMD_MID, payload=struct.pack("<HH", RTS_FDIR_REARM, 0), **state)

    def silent(self, sec):
        """Stop the sim (as a paused/crashed Unity would) and wait."""
        time.sleep(sec)


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

    # 1. IDLE
    for _ in range(5):
        c = L.cycle()
    if c:
        show("1 IDLE", c)
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

    # 4. GO with a lateral offset and some closing speed -> autopilot corrects
    L.ground(GNC_GO)
    for _ in range(3):
        c = L.cycle(lat=(0.2, -0.2), closing=0.05)
    if c:
        show("4 GO, lat (+0.2,-0.2), closing 0.05", c)
        if c["phase"] not in (1, 2):
            L.fail("expected CORRECT/APPROACH after GO")
        if not fired(c):
            L.fail("expected corrective pulses")

    # 5. ABORT -> IDLE, valves closed
    L.ground(GNC_ABORT)
    for _ in range(2):
        c = L.cycle(lat=(0.2, -0.2), closing=0.05)
    if c:
        show("5 ABORT", c)
        if c["phase"] != 0 or fired(c):
            L.fail("expected IDLE with all valves closed after ABORT")

    # 6. Corrupt frame must be dropped, not answered
    bad = bytearray(sim_state_frame(L.seq + 1, L.seq * 0.2))
    bad[40] ^= 0xFF
    L.tx.sendto(bytes(bad), (a.host, a.port))
    L.rx.settimeout(0.5)
    try:
        data, _ = L.rx.recvfrom(512)
        L.fail(f"corrupt frame was answered ({len(data)} bytes)")
    except socket.timeout:
        print("  6 corrupt frame                     dropped (OK)")

    # 7. Link loss while guidance is active -> LC/SC abort
    L.ground(GNC_GO, rng=40.0)
    c = L.paced(3, rng=40.0)
    if c and c["phase"] not in (1, 2):
        L.fail("expected guidance active after GO")
    L.silent(5.0)
    c = L.cycle(rng=40.0)
    if c:
        show("7 link loss 5 s (LC AP0 -> RTS 2)", c)
        if c["phase"] != 0 or fired(c):
            L.fail("expected IDLE (FDIR ABORT) with valves closed after link loss")

    # 8. Re-arm, fly, lose the link again -> aborts again only if AP 0 was re-armed
    L.rearm_fdir(rng=40.0)
    L.ground(GNC_GO, rng=40.0)
    L.paced(3, rng=40.0)
    L.silent(5.0)
    c = L.cycle(rng=40.0)
    if c:
        show("8 re-armed, link loss again", c)
        if c["phase"] != 0:
            L.fail("expected a second FDIR ABORT — RTS 4 did not re-arm AP 0")

    # 9. Actuator anomaly on approach -> LC AP1 -> RTS 3 HOLD
    L.rearm_fdir(rng=40.0)
    L.ground(GNC_GO, rng=40.0)
    for _ in range(40):                         # settle gate: 15 steady cycles
        c = L.cycle(rng=40.0)
        if c and c["phase"] == 2:
            break
    if not c or c["phase"] != 2:
        L.fail("never reached APPROACH (settle gate)")
    else:
        show("9a APPROACH (burning to close)", c)
        held = False
        for _ in range(20):                      # burns commanded, closing speed stays 0
            c = L.cycle(rng=40.0)
            time.sleep(0.2)
            if c and c["phase"] == 4:
                held = True
                break
        if c:
            show("9b frozen closing (LC AP1 -> RTS 3)", c)
        if not held:
            L.fail("expected FDIR HOLD after axial burns stopped delivering")

    if L.rtts:
        r = sorted(L.rtts)
        print(f"round trip ms: median {r[len(r) // 2]:.2f}, max {r[-1]:.2f} over {len(r)} cycles")
    print("PASS" if L.failures == 0 else f"FAILED ({L.failures})")
    return 0 if L.failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
