#!/usr/bin/env python3
"""
GNC ground command sender — builds a minimal CCSDS command packet and
sends it to the CI_LAB uplink port on the running cFS instance.

Usage:
    python3 gnc_cmd.py noop
    python3 gnc_cmd.py reset
    python3 gnc_cmd.py hold
    python3 gnc_cmd.py go
    python3 gnc_cmd.py abort
    python3 gnc_cmd.py rearm       # SC RTS 4: re-arm the LC FDIR actionpoints
    python3 gnc_cmd.py trace-on    # show the per-cycle "GNC #" line in the cFS console
    python3 gnc_cmd.py trace-off
    python3 gnc_cmd.py nav-reset   # NAV: drop the filter, re-initialise on the next LIDAR fix

CI_LAB listens on 127.0.0.1:1234 by default (cpu1, no port offset).

CCSDS packet layout (big-endian primary header):
  [0-1]  StreamId  = command MID (type=cmd, secondary-hdr=present)
  [2-3]  Sequence  = 0xC000  (standalone packet, count=0)
  [4-5]  PDLength  = total length - 7
  [6]    FcnCode   = command function code (bits 6-0)
  [7]    Checksum  = XOR checksum (all bytes XOR to 0xFF); CI_LAB doesn't check
                     it, but commands relayed by SC (RTSs) must carry a valid one
  [8-]   payload (little-endian, as the flight software reads it)
"""

import socket
import struct
import sys

GNC_APP_CMD_MID = 0x1893   # gnc_app_msgids.h
NAV_CMD_MID = 0x18D0       # nav_msgids.h
SC_CMD_MID = 0x18A9        # SC command MID (topic 0xA9)
CFE_EVS_CMD_MID = 0x1801   # cFE EVS command MID (topic 0x01)

SC_START_RTS_CC = 4
RTS_FDIR_REARM = 4         # cFS/sample_defs/cpu1/tables/sc_rts004.c

EVS_ENABLE_APP_EVENT_TYPE_CC = 5
EVS_DISABLE_APP_EVENT_TYPE_CC = 6
EVS_DEBUG_BIT = 0x01

def _evs_app_type(app: str, bitmask: int) -> bytes:
    # CFE_EVS_AppNameBitMaskCmd_Payload_t: AppName[20], BitMask, Spare
    return struct.pack("<20sBB", app.encode("ascii"), bitmask, 0)


# name: (MID, function code, payload, what it does) — GNC codes match gnc_app_msg.h
COMMANDS = {
    "noop":      (GNC_APP_CMD_MID, 0, b"", "GNC NOOP"),
    "reset":     (GNC_APP_CMD_MID, 1, b"", "GNC RESET_COUNTERS"),
    "hold":      (GNC_APP_CMD_MID, 2, b"", "GNC HOLD"),
    "go":        (GNC_APP_CMD_MID, 3, b"", "GNC GO"),
    "abort":     (GNC_APP_CMD_MID, 4, b"", "GNC ABORT"),
    "rearm":     (SC_CMD_MID, SC_START_RTS_CC, struct.pack("<HH", RTS_FDIR_REARM, 0),
                  "SC START_RTS 4 (re-arm LC FDIR actionpoints)"),
    "trace-on":  (CFE_EVS_CMD_MID, EVS_ENABLE_APP_EVENT_TYPE_CC, _evs_app_type("GNC_APP", EVS_DEBUG_BIT),
                  "EVS enable GNC_APP DEBUG events (per-cycle GNC # line)"),
    "trace-off": (CFE_EVS_CMD_MID, EVS_DISABLE_APP_EVENT_TYPE_CC, _evs_app_type("GNC_APP", EVS_DEBUG_BIT),
                  "EVS disable GNC_APP DEBUG events"),
    "nav-reset": (NAV_CMD_MID, 2, b"", "NAV RESET_FILTER (re-initialise from the next LIDAR fix)"),
}

CI_LAB_HOST = "127.0.0.1"
CI_LAB_PORT = 1234


def build_ccsds_cmd(mid: int, fc: int, payload: bytes = b"") -> bytes:
    pkt = bytearray(struct.pack(">HHHBB", mid, 0xC000, 1 + len(payload), fc & 0x7F, 0) + payload)
    cksum = 0xFF
    for b in pkt:
        cksum ^= b
    pkt[7] = cksum
    return bytes(pkt)


def send_cmd(name: str) -> None:
    entry = COMMANDS.get(name.lower())
    if entry is None:
        print(f"Unknown command '{name}'. Valid: {', '.join(COMMANDS)}")
        sys.exit(1)

    mid, fc, payload, what = entry
    pkt = build_ccsds_cmd(mid, fc, payload)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(pkt, (CI_LAB_HOST, CI_LAB_PORT))

    print(f"Sent {what} (MID=0x{mid:04X} FC={fc}) -> {CI_LAB_HOST}:{CI_LAB_PORT} ({len(pkt)} bytes)")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <{'|'.join(COMMANDS)}>")
        sys.exit(1)
    send_cmd(sys.argv[1])
