using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives wrench commands from cFS over UDP and forwards them to RCSModel (Phase 6-4+).
///
/// Command packet layout (32 bytes, little-endian):
///   bytes [ 0- 3]  float  Fx  N    body-frame force
///   bytes [ 4- 7]  float  Fy  N
///   bytes [ 8-11]  float  Fz  N
///   bytes [12-15]  float  Tx  N·m  body-frame torque
///   bytes [16-19]  float  Ty  N·m
///   bytes [20-23]  float  Tz  N·m
///   bytes [24-27]  float  Duration_s
///   bytes [28-31]  int32  GncPhase  (0=IDLE, 1=CORRECT, 2=APPROACH, 3=DOCKED, 4=HOLD)
///
/// A zero wrench (Fx=Fy=Fz=Tx=Ty=Tz=0) with any duration is a coast/heartbeat —
/// resets the cFS timeout without firing any thrusters.
/// RCSModel.SetWrenchCommand() runs the pseudo-inverse allocator to map the wrench
/// to physical thruster firings.
/// </summary>
public class UdpCommandReceiver : MonoBehaviour
{
    public RCSModel rcsModel;

    [Header("Network")]
    [Tooltip("Port this script listens on (cFS sends commands here).")]
    public int listenPort = 5006;

    [Header("Timeout")]
    [Tooltip("Seconds without a command before reverting to keyboard control.")]
    public float commandTimeoutSec = 3.0f;

    [Header("Debug")]
    public bool debugLog = false;

    private UdpClient     listener;
    private Thread        recvThread;
    private volatile bool running;

    // Producer (recv thread) writes these then sets hasPendingCmd.
    // The volatile write to hasPendingCmd acts as a release fence, so the
    // main thread always sees a consistent wrench snapshot when it reads them.
    private volatile float pendingFx, pendingFy, pendingFz;
    private volatile float pendingTx, pendingTy, pendingTz;
    private volatile float pendingDuration;
    private volatile int   pendingPhase = -1;
    private volatile bool  hasPendingCmd;

    private float lastCmdTime;
    private bool  cfsActive;

    /// <summary>True while cFS has active command authority (timeout not expired).</summary>
    public bool CfsActive => cfsActive;

    /// <summary>
    /// Most recent GNC phase received from cFS (GNC_Phase_t: 0=IDLE, 1=CORRECT,
    /// 2=APPROACH, 3=DOCKED, 4=HOLD). -1 if no phase data received yet.
    /// </summary>
    public int GncPhase { get; private set; } = -1;

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
                if (data.Length >= 32)
                {
                    // Write all fields before setting the flag (release ordering via volatile).
                    pendingFx       = BitConverter.ToSingle(data,  0);
                    pendingFy       = BitConverter.ToSingle(data,  4);
                    pendingFz       = BitConverter.ToSingle(data,  8);
                    pendingTx       = BitConverter.ToSingle(data, 12);
                    pendingTy       = BitConverter.ToSingle(data, 16);
                    pendingTz       = BitConverter.ToSingle(data, 20);
                    pendingDuration = BitConverter.ToSingle(data, 24);
                    pendingPhase    = BitConverter.ToInt32 (data, 28);
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
        // Process any incoming packet first, then check timeout — ordering matters so a
        // packet arriving this frame always resets the timer before the check runs.
        if (hasPendingCmd)
        {
            var force      = new Vector3(pendingFx, pendingFy, pendingFz);
            var torque     = new Vector3(pendingTx, pendingTy, pendingTz);
            float duration = pendingDuration;
            GncPhase       = pendingPhase;
            hasPendingCmd  = false;

            if (rcsModel != null)
                rcsModel.SetWrenchCommand(force, torque, duration);
            lastCmdTime = Time.time;
            cfsActive   = true;

            if (debugLog)
                Debug.Log($"[UdpCommandReceiver] F=({force.x:F1},{force.y:F1},{force.z:F1})N " +
                          $"T=({torque.x:F1},{torque.y:F1},{torque.z:F1})Nm dur={duration:F3}s");
        }

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
