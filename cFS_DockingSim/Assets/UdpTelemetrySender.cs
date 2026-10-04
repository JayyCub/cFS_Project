using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

/// <summary>
/// SimLink lock-step master. Every GNC cycle (gncCycleSec of simulation time) it sends a
/// SIM_STATE frame — the raw sensor readings from ChaserSensors plus the crew hand
/// controllers — to cFS SIM_IO, then blocks this physics step until the THRUSTER_CMD
/// answering that frame comes back and is applied — so flight software runs on
/// simulation time and every run with the same inputs is repeatable.
///
/// If cFS doesn't answer within lockStepTimeoutMs the link drops to free-running (the sim
/// keeps going, cFS commands are applied whenever they arrive) and re-engages automatically
/// once cFS answers the previous cycle again. Frame format: SimLinkProtocol / Docs/SIMLINK_ICD.md.
///
/// Execution order: runs after ChaserSensors (so the Δv it samples includes the last physics
/// step) and before RCSModel (so a command applied here fires in this same step).
/// </summary>
[DefaultExecutionOrder(-100)]
public class UdpTelemetrySender : MonoBehaviour
{
    [Tooltip("Truth navigation — used only to find the docking ports for the sensors.")]
    public RelativeNav      nav;
    public VehicleState     chaser;
    public DockingDetector  detector;
    [Tooltip("Navigation sensors (IMU, star tracker, LIDAR). Found or added automatically if left empty.")]
    public ChaserSensors    sensors;
    [Tooltip("Command side of the link. Found automatically if left empty.")]
    public UdpCommandReceiver commandReceiver;
    [Tooltip("Crew hand controllers (keyboard). Found or added automatically if left empty.")]
    public HandController handController;

    [Header("Network")]
    [Tooltip("IP address of the machine running cFS.")]
    public string targetIP   = "127.0.0.1";
    public int    targetPort = 5005;

    [Header("Lock-step")]
    [Tooltip("Simulation seconds per GNC cycle. Rounded to a whole number of physics steps; " +
             "cFS reads the actual value from each frame's CycleDt_s.")]
    public float gncCycleSec = 0.2f;
    [Tooltip("Hold each GNC cycle until cFS answers it. Off = free-running (legacy behaviour).")]
    public bool  lockStep = true;
    [Tooltip("Real-time milliseconds to wait for cFS before dropping to free-running.")]
    public int   lockStepTimeoutMs = 500;

    /// <summary>Simulation time (s) since play started — the time base cFS sees.</summary>
    public double SimTime       { get; private set; }
    public uint   Seq           { get; private set; }
    public bool   LockStepEngaged => _engaged;

    private UdpClient  udpClient;
    private IPEndPoint endpoint;
    private int        stepInCycle;
    private bool       _engaged;

    void Start()
    {
        if (commandReceiver == null)
            commandReceiver = FindAnyObjectByType<UdpCommandReceiver>();
        if (handController == null)
            handController = FindAnyObjectByType<HandController>() ?? gameObject.AddComponent<HandController>();
        WireSensors();

        try
        {
            udpClient = new UdpClient();
            endpoint  = new IPEndPoint(IPAddress.Parse(targetIP), targetPort);
        }
        catch (Exception e)
        {
            Debug.LogError($"[UdpTelemetrySender] Failed to create socket: {e.Message}");
            enabled = false;
            return;
        }

        Debug.Log($"[UdpTelemetrySender] SimLink v{SimLinkProtocol.Version} → {targetIP}:{targetPort}, " +
                  $"GNC cycle {StepsPerCycle * Time.fixedDeltaTime:F3}s ({StepsPerCycle} physics steps), " +
                  $"lock-step {(lockStep ? "ON" : "OFF")}");
    }

    int StepsPerCycle => Mathf.Max(1, Mathf.RoundToInt(gncCycleSec / Time.fixedDeltaTime));

    void WireSensors()
    {
        if (sensors == null)
            sensors = FindAnyObjectByType<ChaserSensors>() ?? gameObject.AddComponent<ChaserSensors>();
        if (sensors.chaser      == null) sensors.chaser      = chaser;
        if (sensors.detector    == null) sensors.detector    = detector;
        if (sensors.relativeNav == null) sensors.relativeNav = nav;
        if (sensors.gravity     == null) sensors.gravity     = FindAnyObjectByType<ClohessyWiltshire>();
        if (nav != null)
        {
            if (sensors.chaserPort == null) sensors.chaserPort = nav.chaserPort;
            if (sensors.targetPort == null) sensors.targetPort = nav.targetPort;
        }
    }

    void FixedUpdate()
    {
        if (stepInCycle == 0)
            RunCycleBoundary();

        stepInCycle = (stepInCycle + 1) % StepsPerCycle;
        SimTime += Time.fixedDeltaTime;
    }

    void RunCycleBoundary()
    {
        if (chaser == null || sensors == null) return;

        if (handController != null) handController.Sample();

        Seq++;
        byte[] frame = SimLinkProtocol.BuildSimStateFrame(Seq, SimTime, BuildState());
        try { udpClient.Send(frame, frame.Length, endpoint); }
        catch (Exception e) { Debug.LogWarning($"[UdpTelemetrySender] Send error: {e.Message}"); }

        if (!lockStep || commandReceiver == null)
        {
            SetEngaged(false, null);
            return;
        }

        // Engage once cFS has answered the previous cycle — proof it's alive and keeping up.
        if (!_engaged && Seq > 1 && commandReceiver.LatestSeq == Seq - 1)
            SetEngaged(true, $"cFS answering — lock-step ENGAGED at Seq {Seq}");

        if (_engaged && !commandReceiver.WaitForCommand(Seq, lockStepTimeoutMs))
            SetEngaged(false, $"no cFS answer to Seq {Seq} within {lockStepTimeoutMs} ms — " +
                              "lock-step DISENGAGED, sim free-running until cFS catches up");
    }

    void SetEngaged(bool engaged, string why)
    {
        if (engaged != _engaged && why != null)
        {
            if (engaged) Debug.Log($"[UdpTelemetrySender] {why}");
            else         Debug.LogWarning($"[UdpTelemetrySender] {why}");
        }
        _engaged = engaged;
        if (commandReceiver != null)
            commandReceiver.LockStepEngaged = engaged;
    }

    SimLinkProtocol.SimState BuildState()
    {
        float cycleDt = StepsPerCycle * Time.fixedDeltaTime;
        var   state   = new SimLinkProtocol.SimState
        {
            CycleDt_s = cycleDt,
            Thc       = handController != null ? handController.Translation : Vector3.zero,
            Rhc       = handController != null ? handController.Rotation    : Vector3.zero,
        };
        sensors.Sample(ref state, cycleDt);
        return state;
    }

    void OnDestroy()
    {
        udpClient?.Close();
    }
}
