using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Sends relative navigation telemetry to cFS over UDP at a fixed rate.
///
/// Packet layout (60 bytes, all little-endian):
///   [0]  float  MET_s               mission elapsed time
///   [4]  float  Range_m
///   [8]  float  ClosingSpeed_ms
///   [12] float  LateralOffset_m
///   [16] float  AttitudeError_deg
///   [20] float  Pos_X
///   [24] float  Pos_Y
///   [28] float  Pos_Z
///   [32] float  Vel_X
///   [36] float  Vel_Y
///   [40] float  Vel_Z
///   [44] float  AngVel_X
///   [48] float  AngVel_Y
///   [52] float  AngVel_Z
///   [56] int32  Flags  (bit 0 = InCorridor, bit 1 = Docked)
///
/// On the cFS side, declare a matching packed struct and read with CFE_SB or raw UDP.
/// </summary>
public class UdpTelemetrySender : MonoBehaviour
{
    public RelativeNav      nav;
    public VehicleState     chaser;
    public ApproachCorridor corridor;
    public DockingDetector  detector;

    [Header("Network")]
    [Tooltip("IP address of the machine running cFS.")]
    public string targetIP   = "127.0.0.1";
    public int    targetPort = 5005;

    [Header("Rate")]
    [Tooltip("Telemetry packets per second sent to cFS.")]
    public float sendRate = 10f;   // Hz — matches cFS scheduler default

    private UdpClient  udpClient;
    private IPEndPoint endpoint;
    private float      missionTime;
    private float      nextSend;

    // Shared between main thread (writes) and send thread (reads)
    private byte[]                 pendingPacket;
    private readonly object        packetLock  = new object();
    private ManualResetEventSlim   packetReady = new ManualResetEventSlim(false);
    private Thread                 sendThread;
    private volatile bool          running;

    void Start()
    {
        try
        {
            udpClient = new UdpClient();
            endpoint  = new IPEndPoint(IPAddress.Parse(targetIP), targetPort);
            Debug.Log($"[UdpTelemetrySender] Sending to {targetIP}:{targetPort} at {sendRate} Hz");
        }
        catch (Exception e)
        {
            Debug.LogError($"[UdpTelemetrySender] Failed to create socket: {e.Message}");
            enabled = false;
            return;
        }

        running    = true;
        sendThread = new Thread(SendLoop) { IsBackground = true, Name = "UdpSendThread" };
        sendThread.Start();
    }

    void FixedUpdate()
    {
        missionTime += Time.fixedDeltaTime;

        if (Time.time < nextSend) return;
        nextSend = Time.time + 1f / sendRate;

        if (nav == null || chaser == null) return;

        byte[] packet = BuildPacket();
        lock (packetLock)
            pendingPacket = packet;
        packetReady.Set();
    }

    // Background thread — wakes only when a new packet is staged, then fires it.
    void SendLoop()
    {
        while (running)
        {
            packetReady.Wait();
            packetReady.Reset();

            byte[] pkt = null;
            lock (packetLock)
            {
                pkt           = pendingPacket;
                pendingPacket = null;
            }

            if (pkt != null)
            {
                try { udpClient.Send(pkt, pkt.Length, endpoint); }
                catch (Exception e)
                {
                    Debug.LogWarning($"[UdpTelemetrySender] Send error: {e.Message}");
                }
            }
        }
    }

    byte[] BuildPacket()
    {
        Vector3 pos    = chaser.position;
        Vector3 vel    = chaser.velocity;
        Vector3 angVel = chaser.angularVelocity;

        bool inCorridor = corridor != null && corridor.inCorridor;
        bool docked     = detector != null && detector.isDocked;
        int  flags      = (inCorridor ? 1 : 0) | (docked ? 2 : 0);

        byte[] buf = new byte[60];
        int    off = 0;

        void WriteFloat(float v) { Buffer.BlockCopy(BitConverter.GetBytes(v), 0, buf, off, 4); off += 4; }
        void WriteInt(int v)     { Buffer.BlockCopy(BitConverter.GetBytes(v), 0, buf, off, 4); off += 4; }

        WriteFloat(missionTime);
        WriteFloat(nav.range);
        WriteFloat(nav.closingSpeed);
        WriteFloat(nav.lateralOffset);
        WriteFloat(nav.attitudeError);
        WriteFloat(pos.x);
        WriteFloat(pos.y);
        WriteFloat(pos.z);
        WriteFloat(vel.x);
        WriteFloat(vel.y);
        WriteFloat(vel.z);
        WriteFloat(angVel.x);
        WriteFloat(angVel.y);
        WriteFloat(angVel.z);
        WriteInt(flags);

        return buf;
    }

    void OnDestroy()
    {
        running = false;
        packetReady.Set();   // unblock the send thread so it can exit
        sendThread?.Join(200);
        udpClient?.Close();
        packetReady.Dispose();
    }
}
