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

CI_LAB listens on 127.0.0.1:1234 by default (cpu1, no port offset).
cFS does not enforce CCSDS checksum in lab builds so the checksum byte is 0x00.

CCSDS packet layout (8 bytes total, big-endian primary header):
  [0-1]  StreamId  = 0x1893  (GNC_APP_CMD_MID — type=cmd, secondary-hdr=present)
  [2-3]  Sequence  = 0xC000  (standalone packet, count=0)
  [4-5]  PDLength  = 0x0001  (data field is 2 bytes: FC + checksum; PDLen = 2-1)
  [6]    FcnCode   = command function code (bits 6-0)
  [7]    Checksum  = 0x00
"""

import socket
import struct
import sys

# GNC_APP_CMD_MID and function codes — must match gnc_app.h exactly
GNC_APP_CMD_MID = 0x1893

COMMANDS = {
    "noop":  0,   # GNC_APP_NOOP_CC
    "reset": 1,   # GNC_APP_RESET_COUNTERS_CC
    "hold":  2,   # GNC_APP_HOLD_CC
    "go":    3,   # GNC_APP_GO_CC
    "abort": 4,   # GNC_APP_ABORT_CC
}

CI_LAB_HOST = "127.0.0.1"
CI_LAB_PORT = 1234


def build_ccsds_cmd(apid: int, fc: int) -> bytes:
    stream_id = apid          # CCSDS type=1 (cmd), sec-hdr=1 already encoded in MID
    sequence  = 0xC000        # standalone packet, count 0
    pdlength  = 0x0001        # secondary header is 2 bytes; PDLen = 2 - 1
    fc_byte   = fc & 0x7F     # function code occupies bits 6-0
    checksum  = 0x00
    return struct.pack(">HHHBB", stream_id, sequence, pdlength, fc_byte, checksum)


def send_cmd(name: str) -> None:
    fc = COMMANDS.get(name.lower())
    if fc is None:
        print(f"Unknown command '{name}'. Valid: {', '.join(COMMANDS)}")
        sys.exit(1)

    pkt = build_ccsds_cmd(GNC_APP_CMD_MID, fc)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(pkt, (CI_LAB_HOST, CI_LAB_PORT))

    print(f"Sent GNC {name.upper()} (FC={fc}) -> {CI_LAB_HOST}:{CI_LAB_PORT} ({len(pkt)} bytes)")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <{'|'.join(COMMANDS)}>")
        sys.exit(1)
    send_cmd(sys.argv[1])
