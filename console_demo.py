#!/usr/bin/env python3
"""
Console demo — serves console.html fed by a canned 30-second telemetry loop, for
working on the UI without cFS or Unity running.

    python3 console_demo.py        then open http://localhost:8081

console.html is re-read on every request, so edit it and refresh the browser.
The loop is compressed to touch the UI's states, so its numbers aren't physical.
Reuses ground_console's HTTP handler and SSE broadcast, but opens no telemetry
socket, sends no commands, and writes nothing to run_logs.
"""

import json
import math
import threading
import time

import ground_console as gc

DEMO_PORT = 8081
LOOP_S = 30.0
FRAME_S = 0.2       # STATE packets at 5 Hz, like the real GNC cycle
FRAMES = int(LOOP_S / FRAME_S)
HK_EVERY = 5        # HK / LC / NAV at 1 Hz
HK_STAGE_S = 6      # the HK / LC / NAV scripts below change every 6 s

# time_s, range_m, closing_ms, lateral_m — interpolated between rows
KEYFRAMES = [
    (0.0,  30.0, 0.05, 0.75),
    (9.0,  22.0, 0.12, 0.30),
    (18.0, 10.0, 0.28, 0.08),
    (21.0,  8.0, 0.34, 0.12),
    (24.0,  3.0, 0.05, 0.06),
    (27.0,  0.0, 0.00, 0.02),
    (30.0,  0.0, 0.00, 0.02),
]

# frame -> (type, app, message)
EVENTS = {
    0:   ("INF", "GNC_APP", "Phase -> CORRECT: lateral correction started"),
    24:  ("DBG", "NAV", "Filter initialised from first LIDAR fix"),
    45:  ("INF", "GNC_APP", "Phase -> APPROACH: on centerline"),
    78:  ("INF", "LC", "A deliberately long event message to check how the log row handles "
                       "text that runs up to the EVS limit, wrapping included"),
    90:  ("ERR", "NAV", "LIDAR measurement rejected: innovation 5.2 sigma"),
    102: ("CRIT", "LC", "AP4 FAIL: approach overspeed -> HOLD"),
    108: ("INF", "GNC_APP", "Phase -> HOLD"),
    120: ("INF", "GNC_APP", "Phase -> APPROACH: resumed"),
    135: ("INF", "GNC_APP", "Phase -> DOCKED: contact confirmed"),
}


def lerp_keyframes(t):
    for (t0, *a), (t1, *b) in zip(KEYFRAMES, KEYFRAMES[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return [x + (y - x) * f for x, y in zip(a, b)]
    return KEYFRAMES[-1][1:]


def phase_at(t):
    if t < 9.0:
        return "CORRECT"
    if t < 21.6:
        return "APPROACH"
    if t < 24.0:
        return "HOLD"
    return "APPROACH" if t < 27.0 else "DOCKED"


def stamp():
    return time.strftime("%H:%M:%S")


def publish(partial):
    with gc._gnc_lock:
        gc._gnc_state.update(partial)
    gc._broadcast("gnc", partial)


def publish_event(evt_type, app, msg):
    gc._broadcast("evs", {"app": app, "id": 0, "type": evt_type, "msg": msg, "ts": stamp()})


def attitude_at(t):
    """Pitch, yaw, roll error in degrees: decays from red through yellow to green."""
    amp = 0.0 if phase_at(t) == "DOCKED" else 7.0 * math.exp(-t * 0.12)
    return [amp * math.sin(t * 0.9 + 0.4), amp * 0.7 * math.cos(t * 0.7), amp * 0.4 * math.sin(t * 1.3)]


def position_at(t):
    """Relative position: Z is the approach axis, the chaser closes from -Z."""
    rng, _, lateral = lerp_keyframes(t)
    theta = t * 0.35
    return [lateral * math.cos(theta), lateral * math.sin(theta), -rng]


def state_frame(frame, sim_time):
    t = frame * FRAME_S
    rng, closing, lateral = lerp_keyframes(t)
    phase = phase_at(t)
    overspeed_active = 19.2 <= t < 24.0

    flags = ["LINK_UP"]
    if lateral < 0.5:
        flags.append("IN_CORRIDOR")
    if t < 21.6:
        flags.append("HOLDPT1_ARMED")
    if t < 24.0:
        flags.append("HOLDPT2_ARMED")
    if phase == "DOCKED":
        flags.append("CONTACT")

    att = attitude_at(t)
    pos = position_at(t)
    ang_vel = [math.radians(b - a) / FRAME_S for a, b in zip(att, attitude_at(t + FRAME_S))]
    lat_vel = [(b - a) / FRAME_S for a, b in zip(pos[:2], position_at(t + FRAME_S)[:2])]

    return {
        "phase": phase,
        "flags": flags,
        "range_m": rng,
        "closing_ms": closing,
        "lateral_m": lateral,
        "att_err_deg": att,
        "ang_vel": ang_vel,
        "rel_pos": pos,
        "rel_vel": [*lat_vel, closing],
        "streak": frame - 95 if overspeed_active else 0,
        "overspeed": frame - 95 if overspeed_active else 0,
        "corridor": frame + 1 if t < 4.8 else 0,
        "drift_safe_m": rng * 0.8,
        "cam_dv": 0.0,
        "sim_time": sim_time,
        "last_rx": stamp(),
    }


def nav_hk(stage, cycle):
    # status bits per stage of the loop (see gc.NAV_STATUS): converging on gyro,
    # star tracker, adding LIDAR pose, dropout, recovered
    status = [0x0B, 0x9F, 0x19F, 0x22, 0x19F][stage]
    return {
        "nav_status": [name for bit, name in gc.NAV_STATUS.items() if status & bit],
        "nav_relnav": bool(status & 0x04),
        "nav_filter": bool(status & 0x08),
        "nav_att": bool(status & 0x01),
        "nav_st": bool(status & 0x80),
        "nav_pose": bool(status & 0x100),
        "nav_rps": [cycle, stage // 3, 0],
        "nav_inits": 1,
        "nav_sensor_rej": 2 if stage >= 3 else 0,
        "nav_innov": [1.1, 0.8, 1.6, 5.2, 0.9][stage],
        "nav_sigma": [[0.40, 0.021], [0.12, 0.008], [0.05, 0.004], [0.30, 0.015], [0.04, 0.003]][stage],
        "nav_att_sigma": 0.0009 - stage * 0.0001,
        "nav_att_rej": 0,
        "nav_gyro_bias": [1.2e-5, -0.8e-5, 0.3e-5],
        "nav_accel_bias": [2.1e-4, -1.3e-4, 0.6e-4],
    }


def lc_hk(stage):
    overspeed_failed = stage == 3
    tripped = stage == 4        # actionpoint goes PASSIVE after it fires
    fdir = []
    for ap, label in gc.FDIR_APS.items():
        is_overspeed = ap == 4
        fdir.append({
            "ap": ap,
            "label": label,
            "state": "PASSIVE" if is_overspeed and tripped else "ACTIVE",
            "result": "FAIL" if is_overspeed and overspeed_failed else "PASS",
        })
    return {"lc_state": "ACTIVE", "fdir": fdir}


def gnc_hk(stage, cycle):
    return {
        "wakeup": cycle,
        "cmd_cnt": 3 + stage,
        "cmd_err": 1 if stage >= 3 else 0,
        "udp_pkts": cycle,
        "stale_sec": [0, 0, 2, 6, 0][stage],   # green, yellow, red
        "last_rx": stamp(),
    }


def demo_thread():
    cycle = 0
    frame = 0
    while True:
        sim_time = cycle * FRAME_S
        publish(state_frame(frame, sim_time))

        if frame % HK_EVERY == 0:
            stage = int(frame * FRAME_S) // HK_STAGE_S
            publish(gnc_hk(stage, cycle))
            publish(lc_hk(stage))
            publish(nav_hk(stage, cycle))

        if frame in EVENTS:
            publish_event(*EVENTS[frame])

        time.sleep(FRAME_S)
        cycle += 1
        frame = (frame + 1) % FRAMES


class DemoHandler(gc.ConsoleHandler):

    def do_POST(self):
        if self.path != "/cmd":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        cmd = json.loads(self.rfile.read(length)).get("cmd", "")
        publish_event("INF", "DEMO", f"{cmd} acknowledged (demo: nothing sent)")
        self._json({"status": "OK"})


def main():
    threading.Thread(target=demo_thread, daemon=True).start()
    server = gc.ThreadedHTTPServer(("127.0.0.1", DEMO_PORT), DemoHandler)
    print(f"[DEMO] Console demo at http://localhost:{DEMO_PORT} (Ctrl-C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[DEMO] Shutting down.")


if __name__ == "__main__":
    main()
