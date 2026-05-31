#!/usr/bin/env python3
"""
Ground Console — cFS / GNC Docking Mission Controller
"""

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

GNC_HK_MID = 0x0893
EVS_LONG_MID = 0x0808

# ---------------------------------------------------------------------------
# GNC HK layout assumptions
# ---------------------------------------------------------------------------

GNC_OFF_WAKEUP = 16
GNC_OFF_CMDCNT = 20
GNC_OFF_CMDERR = 24
GNC_OFF_UDPPKTS = 28
GNC_OFF_PHASE = 32
GNC_OFF_CLOSING = 36
GNC_OFF_LATERAL = 40
GNC_OFF_STALE = 44
GNC_HK_MIN_LEN = 48

EVS_OFF_APPNAME = 12
EVS_OFF_EVENTID = 32
EVS_OFF_EVTTYPE = 34
EVS_OFF_MESSAGE = 44
EVS_LONG_MIN_LEN = 166

PHASE_NAMES = {
    0: "IDLE",
    1: "CORRECT",
    2: "APPROACH",
    3: "DOCKED",
    4: "HOLD",
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
    "closing_ms": 0.0,
    "lateral_m": 0.0,
    "stale_sec": 0,
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

def _decode_gnc_hk(data: bytes):
    if len(data) < GNC_HK_MIN_LEN:
        return None

    phase_byte = data[GNC_OFF_PHASE]

    wakeup, = struct.unpack_from("<I", data, GNC_OFF_WAKEUP)
    cmd_cnt, = struct.unpack_from("<I", data, GNC_OFF_CMDCNT)
    cmd_err, = struct.unpack_from("<I", data, GNC_OFF_CMDERR)
    udp_pkts, = struct.unpack_from("<I", data, GNC_OFF_UDPPKTS)

    closing, = struct.unpack_from("<f", data, GNC_OFF_CLOSING)
    lateral, = struct.unpack_from("<f", data, GNC_OFF_LATERAL)

    stale, = struct.unpack_from("<I", data, GNC_OFF_STALE)

    return {
        "phase": PHASE_NAMES.get(phase_byte, f"UNK({phase_byte})"),
        "wakeup": wakeup,
        "cmd_cnt": cmd_cnt,
        "cmd_err": cmd_err,
        "udp_pkts": udp_pkts,
        "closing_ms": closing,
        "lateral_m": lateral,
        "stale_sec": stale,
    }


def _decode_evs_long(data: bytes):
    if len(data) < EVS_LONG_MIN_LEN:
        return None

    app_raw = data[EVS_OFF_APPNAME : EVS_OFF_APPNAME + 20]
    app_name = app_raw.rstrip(b"\x00").decode("ascii", errors="replace")

    evt_id, = struct.unpack_from(">H", data, EVS_OFF_EVENTID)
    evt_type, = struct.unpack_from(">H", data, EVS_OFF_EVTTYPE)

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

        print(
            f"[UDP] RX from {addr[0]}:{addr[1]} "
            f"len={len(data)} "
            f"raw={data[:32].hex()}"
        )

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

        elif mid == EVS_LONG_MID:
            decoded = _decode_evs_long(data)

            if decoded:
                decoded["ts"] = time.strftime("%H:%M:%S")
                _broadcast("evs", decoded)

# ---------------------------------------------------------------------------
# Command sender
# ---------------------------------------------------------------------------

_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(mid: int, cc: int):
    pkt = _build_no_payload_cmd(mid, cc)

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
                mid, cc = _CMD_MAP[cmd]
                result = send_cmd(mid, cc)
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