#!/usr/bin/env python3
"""
Ground Console — cFS / GNC Docking Mission Controller
"""

import csv
import json
import queue
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

TLM_LISTEN_PORT = 2234
HTTP_PORT = 8080

CI_HOST = "localhost"
CI_PORT = 1234

TO_LAB_CMD_MID = 0x1880
TO_LAB_ENABLE_CC = 6

GNC_CMD_MID = 0x1893
GNC_NOOP_CC = 0x00
GNC_RESET_CC = 0x01
GNC_HOLD_CC = 0x02
GNC_GO_CC = 0x03
GNC_ABORT_CC = 0x04

SC_CMD_MID = 0x18A9
SC_START_RTS_CC = 4
RTS_FDIR_REARM = 4          # sample_defs/cpu1/tables/sc_rts004.c

GNC_HK_MID = 0x0893
GNC_STATE_MID = 0x0895
LC_HK_MID = 0x08A7
EVS_LONG_MID = 0x0808

# ---------------------------------------------------------------------------
# Packet layouts (little-endian payloads after the 16-byte CCSDS tlm header)
# Mirror cFS/apps/gnc_app/fsw/inc/gnc_app_msg.h — its _Static_asserts pin the
# sizes these formats assume.
# ---------------------------------------------------------------------------

TLM_HDR_LEN = 16

# GNC_APP_HkTlm_t: CmdCount CmdErrCount CycleCount SimStateCount Phase Flags Spare TlmStaleSec
GNC_HK = struct.Struct("<IIIIBBHI")
# GNC_APP_StateTlm_t: SimTime Seq Phase Flags UnderDeliveryStreak SettleCounter Spare + 23 floats
GNC_STATE = struct.Struct("<dIBBHHH23f")
assert TLM_HDR_LEN + GNC_HK.size == 40 and TLM_HDR_LEN + GNC_STATE.size == 128

GNC_STATE_FIELDS = (
    ["sim_time", "seq", "phase", "flags", "under_delivery_streak", "settle_counter", "spare",
     "range_m", "closing_ms", "lateral_m", "lat_x_m", "lat_y_m",
     "pitch_err_deg", "yaw_err_deg", "roll_err_deg",
     "angvel_x", "angvel_y", "angvel_z",
     "relpos_x", "relpos_y", "relpos_z",
     "relvel_x", "relvel_y", "relvel_z",
     "px_ns", "py_ns", "pz_ns", "lx_nms", "ly_nms", "lz_nms"]
)

GNC_FLAGS = {
    0x01: "ABORT_LATCH",
    0x02: "HOLDPT1_ARMED",
    0x04: "HOLDPT2_ARMED",
    0x08: "IN_CORRIDOR",
    0x10: "CONTACT",
    0x20: "BAD_CTRL",
    0x40: "LINK_UP",
}

# LC_HkTlm_t: CmdCount CmdErrCount CurrentLCState Pad8, WPResults[44], APResults[88]
LC_OFF_STATE = TLM_HDR_LEN + 2
LC_OFF_APRESULTS = TLM_HDR_LEN + 4 + 44
LC_STATES = {1: "ACTIVE", 2: "PASSIVE", 3: "DISABLED"}
LC_AP_STATES = {0: "UNUSED", 1: "ACTIVE", 2: "PASSIVE", 3: "DISABLED"}
LC_AP_RESULTS = {0: "PASS", 1: "FAIL", 2: "ERROR", 3: "STALE"}
FDIR_APS = {0: "Link loss -> ABORT", 1: "Axial anomaly -> HOLD"}  # lc_def_adt.c

RUN_LOG_DIR = Path(__file__).parent / "run_logs"

# CFE_EVS_LongEventTlm_t after the 16-byte tlm header: AppName[20], EventID,
# EventType (native little-endian uint16s), SpacecraftID, ProcessorID, Message[122]
EVS_OFF_APPNAME = TLM_HDR_LEN
EVS_OFF_EVENTID = TLM_HDR_LEN + 20
EVS_OFF_EVTTYPE = TLM_HDR_LEN + 22
EVS_OFF_MESSAGE = TLM_HDR_LEN + 32
EVS_LONG_MIN_LEN = EVS_OFF_MESSAGE + 122

PHASE_NAMES = {
    0: "IDLE",
    1: "CORRECT",
    2: "APPROACH",
    3: "DOCKED",
    4: "HOLD",
    5: "MANUAL",
}

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

_sse_clients = []
_sse_lock = threading.Lock()

_gnc_state = {
    "phase": "---",
    "wakeup": 0,
    "cmd_cnt": 0,
    "cmd_err": 0,
    "udp_pkts": 0,
    "flags": [],
    "closing_ms": 0.0,
    "lateral_m": 0.0,
    "range_m": 0.0,
    "att_err_deg": [0.0, 0.0, 0.0],
    "stale_sec": 0,
    "streak": 0,
    "lc_state": "---",
    "fdir": [],
    "last_rx": None,
}

_gnc_lock = threading.Lock()

# ---------------------------------------------------------------------------
# CCSDS helpers
# ---------------------------------------------------------------------------

def _mid(data: bytes) -> int:
    if len(data) < 2:
        return 0
    return struct.unpack_from(">H", data, 0)[0]


def _checksum(mid: int, cc: int, length_word: int) -> int:
    xor_rest = (
        ((mid >> 8) & 0xFF)
        ^ (mid & 0xFF)
        ^ 0xC0
        ^ 0x00
        ^ ((length_word >> 8) & 0xFF)
        ^ (length_word & 0xFF)
        ^ (cc & 0xFF)
    )
    return (0xFF ^ xor_rest) & 0xFF


def _build_cmd_header(mid: int, cc: int, total_len: int) -> bytes:
    length_word = total_len - 7
    cksum = _checksum(mid, cc, length_word)

    return struct.pack(
        ">HHHBB",
        mid,
        0xC000,
        length_word,
        cc,
        cksum,
    )


def _build_no_payload_cmd(mid: int, cc: int) -> bytes:
    return _build_cmd_header(mid, cc, 8)


def _payload_checksum(pkt: bytes) -> int:
    """Checksum over the whole packet (header + payload), checksum byte taken as 0."""
    x = 0xFF
    for i, b in enumerate(pkt):
        if i != 7:
            x ^= b
    return x


def _build_to_enable_cmd(dest_ip: str) -> bytes:
    total_len = 24

    hdr = _build_cmd_header(
        TO_LAB_CMD_MID,
        TO_LAB_ENABLE_CC,
        total_len,
    )

    enc = dest_ip.encode("ascii")
    ip_bytes = enc[:15] + b"\x00" * (16 - min(len(enc), 15))

    return hdr + ip_bytes

# ---------------------------------------------------------------------------
# Telemetry decoding
# ---------------------------------------------------------------------------

def _flag_names(flags: int):
    return [name for bit, name in GNC_FLAGS.items() if flags & bit]


def _decode_gnc_hk(data: bytes):
    if len(data) < TLM_HDR_LEN + GNC_HK.size:
        return None

    cmd_cnt, cmd_err, cycles, sim_states, phase, flags, _, stale = GNC_HK.unpack_from(data, TLM_HDR_LEN)

    return {
        "phase": PHASE_NAMES.get(phase, f"UNK({phase})"),
        "wakeup": cycles,
        "cmd_cnt": cmd_cnt,
        "cmd_err": cmd_err,
        "udp_pkts": sim_states,
        "flags": _flag_names(flags),
        "stale_sec": stale,
    }


def _decode_gnc_state(data: bytes):
    if len(data) < TLM_HDR_LEN + GNC_STATE.size:
        return None
    return dict(zip(GNC_STATE_FIELDS, GNC_STATE.unpack_from(data, TLM_HDR_LEN)))


def _decode_lc_hk(data: bytes):
    if len(data) < LC_OFF_APRESULTS + 1:
        return None
    fdir = []
    for ap, label in FDIR_APS.items():
        byte = data[LC_OFF_APRESULTS + ap // 2]
        shift = 0 if ap % 2 == 0 else 4           # even AP in the low nibble
        fdir.append({
            "ap": ap,
            "label": label,
            "state": LC_AP_STATES[(byte >> (shift + 2)) & 0x3],
            "result": LC_AP_RESULTS[(byte >> shift) & 0x3],
        })
    return {"lc_state": LC_STATES.get(data[LC_OFF_STATE], "?"), "fdir": fdir}


class StateRecorder:
    """Writes every GNC STATE packet to run_logs/gnc_state_<time>.csv — the
    ground-side copy of what DS records onboard (tools/fdr_decode.py reads that)."""

    def __init__(self):
        self._writer = None
        self._file = None

    def write(self, row: dict):
        if self._writer is None:
            RUN_LOG_DIR.mkdir(exist_ok=True)
            path = RUN_LOG_DIR / f"gnc_state_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self._file = open(path, "w", newline="")
            self._writer = csv.DictWriter(self._file, fieldnames=["rx_time"] + GNC_STATE_FIELDS)
            self._writer.writeheader()
            print(f"[LOG] GNC STATE -> {path}")
        self._writer.writerow({"rx_time": f"{time.time():.3f}", **row})
        self._file.flush()


_recorder = StateRecorder()


def _decode_evs_long(data: bytes):
    if len(data) < EVS_LONG_MIN_LEN:
        return None

    app_raw = data[EVS_OFF_APPNAME : EVS_OFF_APPNAME + 20]
    app_name = app_raw.rstrip(b"\x00").decode("ascii", errors="replace")

    evt_id, = struct.unpack_from("<H", data, EVS_OFF_EVENTID)
    evt_type, = struct.unpack_from("<H", data, EVS_OFF_EVTTYPE)

    msg_raw = data[EVS_OFF_MESSAGE : EVS_OFF_MESSAGE + 122]
    message = msg_raw.rstrip(b"\x00").decode("ascii", errors="replace")

    type_str = {
        1: "DBG",
        2: "INF",
        3: "ERR",
        4: "CRIT",
    }.get(evt_type, f"T{evt_type}")

    return {
        "app": app_name,
        "id": evt_id,
        "type": type_str,
        "msg": message,
    }

# ---------------------------------------------------------------------------
# SSE
# ---------------------------------------------------------------------------

def _broadcast(event_type: str, payload: dict):
    msg = f"event: {event_type}\ndata: {json.dumps(payload)}\n\n"

    with _sse_lock:
        dead = []

        for q in _sse_clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)

        for q in dead:
            _sse_clients.remove(q)

# ---------------------------------------------------------------------------
# UDP receiver
# ---------------------------------------------------------------------------

_seen_mids = set()

def udp_recv_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sock.bind(("0.0.0.0", TLM_LISTEN_PORT))
    sock.settimeout(2.0)

    print(f"[UDP] Listening on :{TLM_LISTEN_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(4096)

        except socket.timeout:
            continue

        except Exception as e:
            print(f"[UDP] recv error: {e}")
            continue

        if len(data) < 6:
            continue

        mid = _mid(data)

        if mid not in _seen_mids:
            _seen_mids.add(mid)

            print(
                f"[UDP] first MID seen: 0x{mid:04X} "
                f"(len={len(data)})"
            )

        if mid == GNC_HK_MID:
            decoded = _decode_gnc_hk(data)

            if decoded:
                decoded["last_rx"] = time.strftime("%H:%M:%S")

                with _gnc_lock:
                    _gnc_state.update(decoded)

                _broadcast("gnc", decoded)

        elif mid == GNC_STATE_MID:
            st = _decode_gnc_state(data)

            if st:
                _recorder.write(st)
                decoded = {
                    "phase": PHASE_NAMES.get(st["phase"], f"UNK({st['phase']})"),
                    "flags": _flag_names(st["flags"]),
                    "range_m": st["range_m"],
                    "closing_ms": st["closing_ms"],
                    "lateral_m": st["lateral_m"],
                    "att_err_deg": [st["pitch_err_deg"], st["yaw_err_deg"], st["roll_err_deg"]],
                    "streak": st["under_delivery_streak"],
                    "sim_time": st["sim_time"],
                    "last_rx": time.strftime("%H:%M:%S"),
                }

                with _gnc_lock:
                    _gnc_state.update(decoded)

                _broadcast("gnc", decoded)

        elif mid == LC_HK_MID:
            decoded = _decode_lc_hk(data)

            if decoded:
                with _gnc_lock:
                    _gnc_state.update(decoded)

                _broadcast("gnc", decoded)

        elif mid == EVS_LONG_MID:
            decoded = _decode_evs_long(data)

            if decoded:
                decoded["ts"] = time.strftime("%H:%M:%S")
                _broadcast("evs", decoded)

# ---------------------------------------------------------------------------
# Command sender
# ---------------------------------------------------------------------------

_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(mid: int, cc: int, payload: bytes = b""):
    pkt = _build_cmd_header(mid, cc, 8 + len(payload)) + payload
    pkt = pkt[:7] + bytes([_payload_checksum(pkt)]) + pkt[8:]

    try:
        _cmd_sock.sendto(pkt, (CI_HOST, CI_PORT))
        return "OK"

    except Exception as e:
        return f"ERROR: {e}"


def send_to_enable(dest_ip: str | None = None) -> None:
    """
    Send TO_LAB OUTPUT_ENABLE command.

    TO_LAB only supports a 16-byte char array for dest_IP,
    so we MUST send a literal IPv4 string and NOT a hostname.
    """
    if dest_ip is None:
        dest_ip = "192.168.65.254"

    # Resolve hostname → IPv4 string; TO_LAB cannot do DNS resolution.
    try:
        resolved = socket.gethostbyname(dest_ip)
        if resolved != dest_ip:
            print(f"[CMD] Resolved {dest_ip!r} → {resolved}")
            dest_ip = resolved
    except socket.gaierror as e:
        print(f"[CMD] WARNING: could not resolve {dest_ip!r}: {e} — sending as-is")

    pkt = _build_to_enable_cmd(dest_ip)

    try:
        _cmd_sock.sendto(pkt, (CI_HOST, CI_PORT))
        print(
            f"[CMD] TO_LAB OUTPUT_ENABLE sent → "
            f"{CI_HOST}:{CI_PORT} "
            f"dest_IP={dest_ip} "
            f"pkt_len={len(pkt)}"
        )
    except Exception as e:
        print(f"[CMD] TO_LAB enable failed: {e}")

# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

CONSOLE_HTML = Path(__file__).parent / "console.html"

_CMD_MAP = {
    "gnc_noop": (GNC_CMD_MID, GNC_NOOP_CC),
    "gnc_reset": (GNC_CMD_MID, GNC_RESET_CC),
    "gnc_hold": (GNC_CMD_MID, GNC_HOLD_CC),
    "gnc_go": (GNC_CMD_MID, GNC_GO_CC),
    "gnc_abort": (GNC_CMD_MID, GNC_ABORT_CC),
    # SC START_RTS 4: set every LC actionpoint ACTIVE again after an FDIR response
    "fdir_rearm": (SC_CMD_MID, SC_START_RTS_CC, struct.pack("<HH", RTS_FDIR_REARM, 0)),
}

class ConsoleHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):

        if self.path in ("/", "/index.html"):
            self._serve_html()

        elif self.path == "/events":
            self._serve_sse()

        elif self.path == "/state":
            self._serve_state()

        else:
            self.send_error(404)

    def do_POST(self):

        if self.path != "/cmd":
            self.send_error(404)
            return

        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)

        try:
            payload = json.loads(body)
            cmd = payload.get("cmd", "")

            if cmd == "to_enable":
                send_to_enable()
                self._json({"status": "OK"})

            elif cmd in _CMD_MAP:
                result = send_cmd(*_CMD_MAP[cmd])
                self._json({"status": result})

            else:
                self._json({"status": "ERROR: unknown command"})

        except Exception as e:
            self._json({"status": f"ERROR: {e}"})

    def _serve_html(self):

        if CONSOLE_HTML.exists():
            html = CONSOLE_HTML.read_bytes()
        else:
            html = b"<html><body>console.html missing</body></html>"

        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()

        self.wfile.write(html)

    def _serve_sse(self):

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        q = queue.Queue(maxsize=50)

        with _sse_lock:
            _sse_clients.append(q)

        with _gnc_lock:
            init = dict(_gnc_state)

        try:
            self.wfile.write(
                f"event: gnc\ndata: {json.dumps(init)}\n\n".encode()
            )
            self.wfile.flush()

            while True:
                try:
                    msg = q.get(timeout=15)

                    self.wfile.write(msg.encode())
                    self.wfile.flush()

                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()

        except Exception:
            pass

    def _serve_state(self):

        with _gnc_lock:
            state = dict(_gnc_state)

        self._json(state)

    def _json(self, obj):

        body = json.dumps(obj).encode()

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()

        self.wfile.write(body)

# ---------------------------------------------------------------------------
# Threaded server
# ---------------------------------------------------------------------------

class ThreadedHTTPServer(HTTPServer):

    def process_request(self, request, client_address):

        t = threading.Thread(
            target=self._handle,
            args=(request, client_address),
        )

        t.daemon = True
        t.start()

    def _handle(self, request, client_address):

        try:
            self.finish_request(request, client_address)

        except Exception:
            pass

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():

    t_udp = threading.Thread(
        target=udp_recv_thread,
        daemon=True,
    )

    t_udp.start()

    def _enable():
        time.sleep(1.5)

        send_to_enable()

    threading.Thread(
        target=_enable,
        daemon=True,
    ).start()

    server = ThreadedHTTPServer(
        ("0.0.0.0", HTTP_PORT),
        ConsoleHandler,
    )

    print(f"[HTTP] Ground console at http://localhost:{HTTP_PORT}")
    print("[HTTP] Press Ctrl-C to stop")

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        print("\n[HTTP] Shutting down.")

if __name__ == "__main__":
    main()