using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives SimLink THRUSTER_CMD frames (valve on-times from the cFS RCS app, via SIM_IO)
/// and hands them to RCSModel, which just opens and closes the valves.
/// See SimLinkProtocol / Docs/SIMLINK_ICD.md for the frame format.
///
/// Two ways a command gets applied:
///   • Lock-step (normal): UdpTelemetrySender sends SIM_STATE with Seq N, then calls
///     WaitForCommand(N) from FixedUpdate, which blocks until the command answering N
///     arrives and applies it in that same physics step — deterministic timing.
///   • Free-running (cFS not keeping up / not running): Update() applies whatever newest
///     command has arrived, like the pre-lock-step link did.
///
/// Every frame is validated (sync, version, type, length, CRC) on the receive thread;
/// corrupt frames are counted and dropped, never applied.
/// </summary>
public class UdpCommandReceiver : MonoBehaviour
{
    public RCSModel rcsModel;

    [Header("Network")]
    [Tooltip("Port this script listens on (cFS SIM_IO sends commands here).")]
    public int listenPort = 5006;

    [Header("Timeout")]
    [Tooltip("Seconds without a command before cFS authority is dropped and thrusters are cleared.")]
    public float commandTimeoutSec = 3.0f;

    [Header("Debug")]
    public bool debugLog = false;

    private UdpClient     listener;
    private Thread        recvThread;
    private volatile bool running;

    // Newest valid command from the recv thread. Guarded by _lock; Monitor.PulseAll
    // on arrival wakes a lock-step WaitForCommand.
    private readonly object              _lock = new object();
    private SimLinkProtocol.ThrusterCmd?   _latest;
    private uint                         _lastAppliedSeq;
    private bool                         _haveApplied;

    private int _rxGood, _rxBad;   // Interlocked from the recv thread

    private float lastCmdTime;
    private bool  cfsActive;

    /// <summary>True while cFS has active command authority (timeout not expired).</summary>
    public bool CfsActive => cfsActive;

    /// <summary>
    /// Set by UdpTelemetrySender while lock-step is engaged — commands are then applied
    /// only through WaitForCommand, never asynchronously from Update().
    /// </summary>
    public bool LockStepEngaged { get; set; }

    /// <summary>Most recent GNC phase (GNC_Phase_t: 0=IDLE … 4=HOLD, 5=MANUAL); -1 before any command.</summary>
    public int GncPhase { get; private set; } = -1;

    /// <summary>Last valve on-times applied (s per thruster). Null before any command.</summary>
    public float[] LastOnTimes { get; private set; }

    /// <summary>
    /// HUD/debug view of the last command: body force/torque the valve pulses produce
    /// (impulse ÷ longest pulse) and the longest pulse. Zeroed on command timeout.
    /// </summary>
    public Vector3 LastForce    { get; private set; }
    public Vector3 LastTorque   { get; private set; }
    public float   LastDuration { get; private set; }

    public int FramesGood => _rxGood;
    public int FramesBad  => _rxBad;

    /// <summary>Seq of the newest valid command received (0 if none yet).</summary>
    public uint LatestSeq { get { lock (_lock) return _latest?.Seq ?? 0; } }

    void Start()
    {
        try
        {
            listener = new UdpClient(listenPort);
            Debug.Log($"[UdpCommandReceiver] Listening for SimLink v{SimLinkProtocol.Version} commands on port {listenPort}");
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
        IPEndPoint remote      = new IPEndPoint(IPAddress.Any, 0);
        bool       badReported = false;

        while (running)
        {
            try
            {
                byte[] data = listener.Receive(ref remote);
                var cmd = SimLinkProtocol.TryParseThrusterCmd(data, data.Length, out string reason);
                if (cmd == null)
                {
                    Interlocked.Increment(ref _rxBad);
                    if (!badReported)
                        Debug.LogWarning($"[UdpCommandReceiver] Dropped bad SimLink frame ({reason}). " +
                                         "Is cFS running the matching SIM_IO build?");
                    badReported = true;
                    continue;
                }
                badReported = false;
                Interlocked.Increment(ref _rxGood);

                lock (_lock)
                {
                    _latest = cmd;
                    Monitor.PulseAll(_lock);
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

    /// <summary>
    /// Lock-step: block until the command answering <paramref name="seq"/> arrives (or the
    /// timeout expires), then apply it immediately. Returns false on timeout.
    /// Called from FixedUpdate so the burn starts in this very physics step.
    /// </summary>
    public bool WaitForCommand(uint seq, int timeoutMs)
    {
        SimLinkProtocol.ThrusterCmd cmd;
        lock (_lock)
        {
            var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
            while (_latest == null || _latest.Value.Seq < seq)
            {
                int remaining = (int)(deadline - DateTime.UtcNow).TotalMilliseconds;
                if (remaining <= 0 || !Monitor.Wait(_lock, remaining))
                    return false;
            }
            if (_latest.Value.Seq != seq)
                return false; // answer to a newer cycle than ours — shouldn't happen, treat as a miss
            cmd = _latest.Value;
        }
        Apply(cmd);
        return true;
    }

    void Update()
    {
        // Free-running path only — in lock-step, WaitForCommand owns application.
        if (!LockStepEngaged)
        {
            SimLinkProtocol.ThrusterCmd? pending = null;
            lock (_lock)
            {
                if (_latest != null && (!_haveApplied || _latest.Value.Seq != _lastAppliedSeq))
                    pending = _latest;
            }
            if (pending != null)
                Apply(pending.Value);
        }

        if (cfsActive && Time.time - lastCmdTime > commandTimeoutSec)
        {
            cfsActive = false;
            if (rcsModel != null)
                rcsModel.ClearExternalControl();
            LastForce = LastTorque = Vector3.zero;
            LastDuration = 0f;
            Debug.Log("[UdpCommandReceiver] cFS command timeout — cFS authority dropped, thrusters cleared.");
        }
    }

    void Apply(in SimLinkProtocol.ThrusterCmd cmd)
    {
        _lastAppliedSeq = cmd.Seq;
        _haveApplied    = true;

        GncPhase = cmd.GncPhase;
        lastCmdTime = Time.time;
        cfsActive   = true;

        float maxOn = 0f;
        foreach (float t in cmd.OnTime_s) maxOn = Mathf.Max(maxOn, t);
        LastOnTimes  = cmd.OnTime_s;
        LastDuration = maxOn;

        if (rcsModel != null)
        {
            rcsModel.SetThrusterOnTimes(cmd.OnTime_s);
            // HUD only: the body force/torque these valve pulses produce, averaged over the
            // longest pulse, from the vehicle's actual (scene) thruster geometry.
            rcsModel.ImpulseFromOnTimes(cmd.OnTime_s, out Vector3 lin, out Vector3 ang);
            LastForce  = maxOn > 0f ? lin / maxOn : Vector3.zero;
            LastTorque = maxOn > 0f ? ang / maxOn : Vector3.zero;
        }

        if (debugLog)
            Debug.Log($"[UdpCommandReceiver] Seq {cmd.Seq} phase {cmd.GncPhase} on-times " +
                      string.Join(" ", System.Array.ConvertAll(cmd.OnTime_s, t => t.ToString("F3"))));
    }

    void OnDestroy()
    {
        running = false;
        listener?.Close();
        recvThread?.Join(200);
    }
}
