using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives timed thruster commands from cFS over UDP and forwards them to RCSModel.
///
/// Command packet layout (8 bytes):
///   bytes [0-3]  int32  ThrusterMask   — 12-bit bitmask, same as RCSModel.SetThrusterCommand()
///   bytes [4-7]  float  BurnDuration_s — seconds to hold each active thruster ON
///
/// A mask of 0 with duration 0.0 is a coast/heartbeat command — it resets the
/// cFS timeout without firing any thrusters.
///
/// Bit assignments (match RCSModel.cs and gnc_app.h):
///   Bit 0  +X (right)      Bit 6  +pitch
///   Bit 1  -X (left)       Bit 7  -pitch
///   Bit 2  +Y (up)         Bit 8  +yaw
///   Bit 3  -Y (down)       Bit 9  -yaw
///   Bit 4  +Z (forward)    Bit 10 +roll
///   Bit 5  -Z (back)       Bit 11 -roll
/// </summary>
public class UdpCommandReceiver : MonoBehaviour
{
    public RCSModel rcsModel;

    [Header("Network")]
    [Tooltip("Port this script listens on (cFS sends commands here).")]
    public int listenPort = 5006;

    [Header("Timeout")]
    [Tooltip("Seconds without a command before reverting to keyboard control.")]
    public float commandTimeoutSec = 1.5f;

    [Header("Debug")]
    public bool debugLog = false;

    private UdpClient     listener;
    private Thread        recvThread;
    private volatile bool running;

    // Producer (recv thread) writes these then sets hasPendingCmd.
    // The volatile write to hasPendingCmd acts as a release fence, so the
    // main thread always sees consistent mask+duration when it reads them.
    private volatile int   pendingMask;
    private volatile float pendingDuration;
    private volatile bool  hasPendingCmd;

    private float lastCmdTime;
    private bool  cfsActive;

    void Start()
    {
        try
        {
            listener = new UdpClient(listenPort);
            Debug.Log($"[UdpCommandReceiver] Listening on port {listenPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[UdpCommandReceiver] Could not bind port {listenPort}: {e.Message}");
            enabled = false;
            return;
        }

        running    = true;
        recvThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "UdpRecvThread" };
        recvThread.Start();
    }

    void ReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);

        while (running)
        {
            try
            {
                byte[] data = listener.Receive(ref remote);
                if (data.Length >= 8)
                {
                    // Write values before setting the flag (release ordering via volatile).
                    pendingMask     = BitConverter.ToInt32(data, 0);
                    pendingDuration = BitConverter.ToSingle(data, 4);
                    hasPendingCmd   = true;
                }
            }
            catch (SocketException)
            {
                break; // listener.Close() during shutdown
            }
            catch (Exception e)
            {
                if (running)
                    Debug.LogWarning($"[UdpCommandReceiver] Recv error: {e.Message}");
            }
        }
    }

    void Update()
    {
        if (!hasPendingCmd) return;

        int   mask     = pendingMask;
        float duration = pendingDuration;
        hasPendingCmd  = false;

        if (rcsModel == null) return;

        rcsModel.SetThrusterCommand(mask, duration);
        lastCmdTime = Time.time;
        cfsActive   = true;

        if (debugLog)
            Debug.Log($"[UdpCommandReceiver] mask=0b{Convert.ToString(mask, 2).PadLeft(12, '0')} dur={duration:F3}s");
    }

    void FixedUpdate()
    {
        if (cfsActive && Time.time - lastCmdTime > commandTimeoutSec)
        {
            cfsActive = false;
            if (rcsModel != null)
                rcsModel.ClearExternalControl();
            Debug.Log("[UdpCommandReceiver] cFS command timeout — keyboard control restored.");
        }
    }

    void OnDestroy()
    {
        running = false;
        listener?.Close();
        recvThread?.Join(200);
    }
}
