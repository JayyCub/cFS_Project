#!/usr/bin/env python3
"""
Decode a DS flight-data-recorder file (realism phase 3) into CSV / text.

DS writes each recorder file as: cFE file header (64 B, big-endian) + DS file
header + the recorded CCSDS packets back to back. This splits the packets by
their CCSDS length field and writes one output per packet type:

  <file>.gnc_state.csv     GNC STATE, one row per lock-step cycle
  <file>.nav_solution.csv  NAV solution (estimate, sigmas, status), one row per cycle
  <file>.nav_hk.csv        NAV HK (1 Hz): LIDAR accept/reject counts, filter inits
  <file>.thruster_cmd.csv  RCS valve on-times + achieved impulse per cycle
  <file>.gnc_hk.csv        GNC HK (1 Hz)
  <file>.events.txt        EVS events (from the fdr_evs file)

Other MIDs are counted and skipped. Recorder files land in the cFS exe dir:
build-native_std/exe/cpu1/cf/fdr_gnc<seq>.dat and fdr_evs<seq>.dat (see
cFS/sample_defs/cpu1/tables/ds_file_tbl.c). Layouts mirror gnc_app_msg.h,
nav_msg.h, sim_io_msg.h and the cFE EVS long event packet.

Usage:
    python3 tools/fdr_decode.py path/to/fdr_gnc00000001.dat [more files...]
"""
import csv
import struct
import sys
from collections import Counter
from pathlib import Path

CFE_FS_HDR_LEN = 64
TLM_HDR_LEN = 16

GNC_HK_MID = 0x0893
GNC_STATE_MID = 0x0895
THRUSTER_CMD_MID = 0x0898
NAV_HK_MID = 0x08D0
NAV_SOLUTION_MID = 0x08D1
EVS_LONG_MID = 0x0808

PHASES = {0: "IDLE", 1: "CORRECT", 2: "APPROACH", 3: "DOCKED", 4: "HOLD", 5: "MANUAL", 6: "DEPART"}

GNC_STATE = struct.Struct("<dIBBHHH23fHH3f")
GNC_STATE_FIELDS = [
    "sim_time", "seq", "phase", "flags", "under_delivery_streak", "settle_counter", "spare",
    "range_m", "closing_ms", "lateral_m", "lat_x_m", "lat_y_m",
    "pitch_err_deg", "yaw_err_deg", "roll_err_deg",
    "angvel_x", "angvel_y", "angvel_z",
    "relpos_x", "relpos_y", "relpos_z",
    "relvel_x", "relvel_y", "relvel_z",
    "px_ns", "py_ns", "pz_ns", "lx_nms", "ly_nms", "lz_nms",
    "overspeed_streak", "corridor_streak", "drift_safe_range_m", "cam_dv_ms", "spare2",
]

GNC_HK = struct.Struct("<IIIIBBHI")
GNC_HK_FIELDS = ["cmd_count", "cmd_err_count", "cycle_count", "sim_state_count", "phase", "flags", "spare",
                 "tlm_stale_sec"]

NAV_SOLUTION = struct.Struct("<dIfIHH4f3f3f3f3f3f3f3f3f2ff4ff")
assert NAV_SOLUTION.size == 168
NAV_SOLUTION_FIELDS = (
    ["sim_time", "seq", "cycle_dt", "status", "consec_rps_rejects", "spare", "q_x", "q_y", "q_z", "q_w",
     "rate_x", "rate_y", "rate_z", "atterr_x_rad", "atterr_y_rad", "atterr_z_rad",
     "cm_pos_x", "cm_pos_y", "cm_pos_z", "cm_vel_x", "cm_vel_y", "cm_vel_z",
     "port_pos_x", "port_pos_y", "port_pos_z", "port_vel_x", "port_vel_y", "port_vel_z",
     "range_m", "closing_ms", "lateral_m", "dv_x", "dv_y", "dv_z", "pos_sigma_m", "vel_sigma_ms",
     "att_sigma_rad", "dock_q_x", "dock_q_y", "dock_q_z", "dock_q_w", "spare2"]
)

NAV_HK = struct.Struct("<9I4f3f3fI")
NAV_HK_FIELDS = ["cmd_count", "cmd_err_count", "cycle_count", "rps_accepted", "rps_rejected",
                 "rps_unavailable", "filter_inits", "sensor_rejects", "status", "last_innov_sigma",
                 "pos_sigma_m", "vel_sigma_ms", "att_sigma_rad", "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
                 "accel_bias_x", "accel_bias_y", "accel_bias_z", "att_rejected"]

N_THR = 16
THRUSTER_CMD = struct.Struct(f"<IIIi{N_THR}f3f3f")
THRUSTER_CMD_FIELDS = (["seq", "spare", "num_thrusters", "gnc_phase"]
                       + [f"on_T{i:02d}" for i in range(N_THR)]
                       + ["ach_px_ns", "ach_py_ns", "ach_pz_ns", "ach_lx_nms", "ach_ly_nms", "ach_lz_nms"])

# CFE_EVS_LongEventTlm_t payload: AppName[20] EventID EventType SpacecraftID ProcessorID Message[122]
EVS = struct.Struct("<20sHHII122s")
EVS_TYPES = {1: "DEBUG", 2: "INFO", 3: "ERROR", 4: "CRIT"}


def packets(buf, start):
    """Yield (mid, packet bytes) walking CCSDS lengths from start; stop at the first bad length."""
    off = start
    while off + 6 <= len(buf):
        mid, _, length = struct.unpack_from(">HHH", buf, off)
        size = length + 7
        if size < TLM_HDR_LEN or off + size > len(buf):
            return
        yield mid, buf[off:off + size]
        off += size


def find_first_packet(buf):
    """The DS header size depends on the platform's max path length, so find the
    offset after the cFE header from which the packet stream parses cleanly to EOF."""
    known = {GNC_HK_MID, GNC_STATE_MID, THRUSTER_CMD_MID, NAV_SOLUTION_MID, NAV_HK_MID, EVS_LONG_MID}
    for start in range(CFE_FS_HDR_LEN, min(len(buf), CFE_FS_HDR_LEN + 512)):
        mid = struct.unpack_from(">H", buf, start)[0] if start + 2 <= len(buf) else None
        if mid not in known and (mid is None or mid & 0xF800 != 0x0800):
            continue
        end = start
        for _, pkt in packets(buf, start):
            end += len(pkt)
        if end == len(buf):
            return start
    return None


def decode(path: Path):
    buf = path.read_bytes()
    if len(buf) < CFE_FS_HDR_LEN or buf[:4] != b"cFE1":
        print(f"{path}: not a cFE file (no cFE1 header)")
        return
    start = find_first_packet(buf)
    if start is None:
        print(f"{path}: could not find a clean packet stream after the headers")
        return

    counts = Counter()
    writers = {}

    def writer(suffix, fields):
        if suffix not in writers:
            f = open(path.with_suffix(path.suffix + suffix), "w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["pkt_time"] + fields)
            writers[suffix] = (f, w)
        return writers[suffix][1]

    events = []
    for mid, pkt in packets(buf, start):
        counts[mid] += 1
        secs, subsecs = struct.unpack_from(">IH", pkt, 6)
        t = f"{secs + subsecs / 65536.0:.4f}"
        body = pkt[TLM_HDR_LEN:]
        if mid == GNC_STATE_MID and len(body) >= GNC_STATE.size:
            row = list(GNC_STATE.unpack_from(body))
            row[2] = PHASES.get(row[2], row[2])
            writer(".gnc_state.csv", GNC_STATE_FIELDS).writerow([t] + row)
        elif mid == THRUSTER_CMD_MID and len(body) >= THRUSTER_CMD.size:
            writer(".thruster_cmd.csv", THRUSTER_CMD_FIELDS).writerow([t] + list(THRUSTER_CMD.unpack_from(body)))
        elif mid == NAV_SOLUTION_MID and len(body) >= NAV_SOLUTION.size:
            writer(".nav_solution.csv", NAV_SOLUTION_FIELDS).writerow([t] + list(NAV_SOLUTION.unpack_from(body)))
        elif mid == NAV_HK_MID and len(body) >= NAV_HK.size:
            writer(".nav_hk.csv", NAV_HK_FIELDS).writerow([t] + list(NAV_HK.unpack_from(body)))
        elif mid == GNC_HK_MID and len(body) >= GNC_HK.size:
            row = list(GNC_HK.unpack_from(body))
            row[4] = PHASES.get(row[4], row[4])
            writer(".gnc_hk.csv", GNC_HK_FIELDS).writerow([t] + row)
        elif mid == EVS_LONG_MID and len(body) >= EVS.size:
            app, eid, etype, _, _, msg = EVS.unpack_from(body)
            events.append(f"{t}  {EVS_TYPES.get(etype, etype):<5} {app.rstrip(b'\0').decode(errors='replace')} "
                          f"{eid}: {msg.rstrip(b'\0').decode(errors='replace')}")

    for f, _ in writers.values():
        f.close()
    if events:
        path.with_suffix(path.suffix + ".events.txt").write_text("\n".join(events) + "\n", encoding="utf-8")

    summary = ", ".join(f"0x{m:04X}={n}" for m, n in sorted(counts.items()))
    outs = [path.name + s for s in writers] + ([path.name + ".events.txt"] if events else [])
    print(f"{path}: {sum(counts.values())} packets ({summary}) -> {', '.join(outs) or 'nothing decoded'}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for arg in sys.argv[1:]:
        decode(Path(arg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
