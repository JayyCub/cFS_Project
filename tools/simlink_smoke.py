#!/usr/bin/env python3
"""
SimLink smoke test — stands in for Unity to verify the cFS side of the lock-step link.

Sends SIM_STATE frames (Seq 1..N) to SIM_IO exactly the way UdpTelemetrySender.cs does,
and checks each one is answered by a CRC-valid WRENCH_CMD echoing the same Seq. Also
sends one corrupt frame and checks cFS does NOT answer it.

Usage (cFS must be running, with SIM_IO's DestHost resolving to this machine):
    python3 tools/simlink_smoke.py [--cycles 25] [--host 127.0.0.1]

Wire format: Docs/SIMLINK_ICD.md (mirrors simlink_icd.h / SimLinkProtocol.cs).
"""
import argparse
import socket
import struct
import sys
import time

SYNC = 0x324B4C53
VERSION = 2
TYPE_SIM_STATE = 1
TYPE_WRENCH_CMD = 2

HDR = struct.Struct("<IHHIId")        # 24 B
STATE = struct.Struct("<5f3f3f3fI3f2f")  # 80 B
WRENCH = struct.Struct("<7fi")        # 32 B
TRL = struct.Struct("<HH")            # 4 B

assert HDR.size == 24 and STATE.size == 80 and WRENCH.size == 32


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def sim_state_frame(seq: int, sim_time: float, rng: float) -> bytes:
    length = HDR.size + STATE.size + TRL.size
    body = HDR.pack(SYNC, VERSION, TYPE_SIM_STATE, seq, length, sim_time) + STATE.pack(
        0.2,                 # CycleDt_s
        rng, 0.05, 0.3, 1.0,  # Range, ClosingSpeed, LateralOffset, AttitudeError
        0.0, 0.0, -rng,       # RelPos
        0.0, 0.0, 0.05,       # RelVel
        0.0, 0.0, 0.0,        # AngVel
        1,                    # Flags: in corridor
        0.5, -0.5, 0.2,       # Pitch/Yaw/Roll error
        0.2, -0.2,            # LatOffset X/Y
    )
    return body + TRL.pack(crc16(body), 0)


def parse_wrench(data: bytes):
    if len(data) != HDR.size + WRENCH.size + TRL.size:
        return None, f"length {len(data)}"
    sync, ver, typ, seq, length, sim_time = HDR.unpack_from(data, 0)
    if sync != SYNC or ver != VERSION or typ != TYPE_WRENCH_CMD or length != len(data):
        return None, f"header sync={sync:#x} ver={ver} type={typ} len={length}"
    crc, _ = TRL.unpack_from(data, len(data) - TRL.size)
    if crc16(data[: len(data) - TRL.size]) != crc:
        return None, "CRC"
    fields = WRENCH.unpack_from(data, HDR.size)
    return {"seq": seq, "sim_time": sim_time, "wrench": fields[:6], "dur": fields[6], "phase": fields[7]}, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="cFS host (SIM_IO listen address)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--listen", type=int, default=5006)
    ap.add_argument("--cycles", type=int, default=25)
    ap.add_argument("--timeout", type=float, default=1.0)
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", args.listen))
    rx.settimeout(args.timeout)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    failures = 0
    rtts = []
    for seq in range(1, args.cycles + 1):
        sim_time = (seq - 1) * 0.2
        t0 = time.perf_counter()
        tx.sendto(sim_state_frame(seq, sim_time, 30.0 - 0.01 * seq), (args.host, args.port))
        try:
            while True:
                data, _ = rx.recvfrom(512)
                cmd, why = parse_wrench(data)
                if cmd is None:
                    print(f"  Seq {seq}: bad WRENCH_CMD frame ({why})")
                    failures += 1
                    continue
                if cmd["seq"] < seq:
                    continue  # late answer to an earlier cycle
                break
        except socket.timeout:
            print(f"FAIL Seq {seq}: no answer within {args.timeout}s")
            failures += 1
            continue
        rtts.append((time.perf_counter() - t0) * 1000)
        if cmd["seq"] != seq or abs(cmd["sim_time"] - sim_time) > 1e-9:
            print(f"FAIL Seq {seq}: answered Seq {cmd['seq']} simTime {cmd['sim_time']}")
            failures += 1
        elif seq == 1 or seq == args.cycles:
            print(f"  Seq {seq}: OK phase={cmd['phase']} dur={cmd['dur']:.3f} F/T={['%.0f' % v for v in cmd['wrench']]}")

    # Corrupt frame (flip one payload byte, keep old CRC) must be dropped, not answered.
    bad = bytearray(sim_state_frame(args.cycles + 1, args.cycles * 0.2, 20.0))
    bad[40] ^= 0xFF
    tx.sendto(bytes(bad), (args.host, args.port))
    rx.settimeout(0.5)
    try:
        data, _ = rx.recvfrom(512)
        print(f"FAIL corrupt frame was answered ({len(data)} bytes)")
        failures += 1
    except socket.timeout:
        print("  corrupt frame: dropped (OK)")

    if rtts:
        rtts.sort()
        print(f"round trip ms: median {rtts[len(rtts) // 2]:.2f}, max {rtts[-1]:.2f} over {len(rtts)} cycles")
    print("PASS" if failures == 0 else f"FAILED ({failures})")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
