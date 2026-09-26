using System;
using UnityEngine;

/// <summary>
/// SimLink v2 — wire format between this simulation (the vehicle "hardware") and
/// cFS's SIM_IO app. Mirrors cFS/apps/sim_io/fsw/inc/simlink_icd.h; see
/// Docs/SIMLINK_ICD.md.
///
///   Header (24 B) | payload | Trailer (4 B), little-endian, no padding.
///
///   Header:  u32 Sync 'SLK2' | u16 Version | u16 Type | u32 Seq | u32 Length | f64 SimTime_s
///   Trailer: u16 CRC-16/CCITT-FALSE over every preceding byte | u16 spare
///
/// Lock-step: one SIM_STATE per GNC cycle with an incrementing Seq; the WRENCH_CMD
/// answering it echoes the same Seq.
/// </summary>
public static class SimLinkProtocol
{
    public const uint   Sync    = 0x324B4C53; // 'S','L','K','2' on the wire
    public const ushort Version = 2;

    public const ushort TypeSimState  = 1; // Unity -> cFS
    public const ushort TypeWrenchCmd = 2; // cFS -> Unity

    public const int HeaderBytes   = 24;
    public const int TrailerBytes  = 4;
    public const int SimStateBytes = 80;
    public const int WrenchBytes   = 32;

    public const int SimStateFrameBytes  = HeaderBytes + SimStateBytes + TrailerBytes; // 108
    public const int WrenchCmdFrameBytes = HeaderBytes + WrenchBytes + TrailerBytes;   // 60

    public const uint FlagInCorridor = 0x1;
    public const uint FlagDocked     = 0x2;

    /// <summary>SIM_STATE payload — field order must match SIMLINK_SimState_t.</summary>
    public struct SimState
    {
        public float   CycleDt_s;
        public float   Range_m, ClosingSpeed_ms, LateralOffset_m, AttitudeError_deg;
        public Vector3 RelPos;   // chaser - target origin, world (m)
        public Vector3 RelVel;   // chaser - target velocity, world (m/s)
        public Vector3 AngVel;   // chaser rate, chaser-port frame (rad/s)
        public uint    Flags;
        public float   PitchError_deg, YawError_deg, RollError_deg;
        public float   LatOffset_X, LatOffset_Y;
    }

    /// <summary>Decoded WRENCH_CMD frame.</summary>
    public struct WrenchCmd
    {
        public uint    Seq;
        public double  SimTime_s;
        public Vector3 Force;    // body frame (N)
        public Vector3 Torque;   // body frame (N·m)
        public float   Duration_s;
        public int     GncPhase;
    }

    public static byte[] BuildSimStateFrame(uint seq, double simTime, in SimState s)
    {
        var buf = new byte[SimStateFrameBytes];
        int off = WriteHeader(buf, TypeSimState, seq, simTime);

        off = PutF(buf, off, s.CycleDt_s);
        off = PutF(buf, off, s.Range_m);
        off = PutF(buf, off, s.ClosingSpeed_ms);
        off = PutF(buf, off, s.LateralOffset_m);
        off = PutF(buf, off, s.AttitudeError_deg);
        off = PutV(buf, off, s.RelPos);
        off = PutV(buf, off, s.RelVel);
        off = PutV(buf, off, s.AngVel);
        off = PutU32(buf, off, s.Flags);
        off = PutF(buf, off, s.PitchError_deg);
        off = PutF(buf, off, s.YawError_deg);
        off = PutF(buf, off, s.RollError_deg);
        off = PutF(buf, off, s.LatOffset_X);
        off = PutF(buf, off, s.LatOffset_Y);

        WriteTrailer(buf, off);
        return buf;
    }

    /// <summary>
    /// Validate and decode a WRENCH_CMD frame. Returns null (with a reason) on any
    /// sync/version/type/length/CRC failure — a corrupt command is dropped, never applied.
    /// </summary>
    public static WrenchCmd? TryParseWrenchCmd(byte[] buf, int len, out string reason)
    {
        reason = null;
        if (len != WrenchCmdFrameBytes)                            { reason = $"length {len}";   return null; }
        if (BitConverter.ToUInt32(buf, 0) != Sync)                 { reason = "sync word";       return null; }
        if (BitConverter.ToUInt16(buf, 4) != Version)              { reason = "version";         return null; }
        if (BitConverter.ToUInt16(buf, 6) != TypeWrenchCmd)        { reason = "type";            return null; }
        if (BitConverter.ToUInt32(buf, 12) != (uint)len)           { reason = "header length";   return null; }
        ushort crc = BitConverter.ToUInt16(buf, len - TrailerBytes);
        if (Crc16(buf, len - TrailerBytes) != crc)                 { reason = "CRC";             return null; }

        int p = HeaderBytes;
        return new WrenchCmd
        {
            Seq        = BitConverter.ToUInt32(buf, 8),
            SimTime_s  = BitConverter.ToDouble(buf, 16),
            Force      = new Vector3(F(buf, p), F(buf, p + 4), F(buf, p + 8)),
            Torque     = new Vector3(F(buf, p + 12), F(buf, p + 16), F(buf, p + 20)),
            Duration_s = F(buf, p + 24),
            GncPhase   = BitConverter.ToInt32(buf, p + 28),
        };
    }

    /// <summary>CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — same as SIMLINK_Crc16.</summary>
    public static ushort Crc16(byte[] data, int len)
    {
        ushort crc = 0xFFFF;
        for (int i = 0; i < len; i++)
        {
            crc ^= (ushort)(data[i] << 8);
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) != 0 ? (ushort)((crc << 1) ^ 0x1021) : (ushort)(crc << 1);
        }
        return crc;
    }

    // ── helpers ────────────────────────────────────────────────────────────

    static int WriteHeader(byte[] buf, ushort type, uint seq, double simTime)
    {
        int off = 0;
        off = PutU32(buf, off, Sync);
        off = PutU16(buf, off, Version);
        off = PutU16(buf, off, type);
        off = PutU32(buf, off, seq);
        off = PutU32(buf, off, (uint)buf.Length);
        Buffer.BlockCopy(BitConverter.GetBytes(simTime), 0, buf, off, 8);
        return off + 8;
    }

    static void WriteTrailer(byte[] buf, int off)
    {
        PutU16(buf, off, Crc16(buf, off));
        PutU16(buf, off + 2, 0);
    }

    static float F(byte[] b, int o) => BitConverter.ToSingle(b, o);

    static int PutF(byte[] b, int o, float v)   { Buffer.BlockCopy(BitConverter.GetBytes(v), 0, b, o, 4); return o + 4; }
    static int PutU32(byte[] b, int o, uint v)  { Buffer.BlockCopy(BitConverter.GetBytes(v), 0, b, o, 4); return o + 4; }
    static int PutU16(byte[] b, int o, ushort v){ Buffer.BlockCopy(BitConverter.GetBytes(v), 0, b, o, 2); return o + 2; }
    static int PutV(byte[] b, int o, Vector3 v) { o = PutF(b, o, v.x); o = PutF(b, o, v.y); return PutF(b, o, v.z); }
}
