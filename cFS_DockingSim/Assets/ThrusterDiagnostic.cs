using System.Collections;
using System.IO;
using UnityEngine;

/// <summary>
/// Automated thruster calibration diagnostic.
///
/// Runs every translational and attitude thruster group at two burn durations
/// (shortBurn + longBurn) and logs the resulting delta-V/delta-omega to the
/// Unity Console with a [DIAG] prefix.  Off-axis components (coupling) are
/// logged alongside the primary axis so you can see exactly how much backward
/// drift each command causes.
///
/// Setup:
///   1. Add this component to any active GameObject in the scene.
///   2. Drag the RCSModel into the "rcs" slot.  The Rigidbody is auto-found
///      from rcs.vehicle; override it in the Inspector if needed.
///   3. Run with cFS in IDLE/ABORT so UDP commands do not interfere.
///      (Send ABORT from the ground tool before pressing F8.)
///   4. Press F8.  Watch the Console, or open the timestamped .log file written
///      to the project root (cFS_DockingSim/ThrusterDiagnostic_*.log — same
///      convention as TelemetryLogger.cs) and copy-paste into a spreadsheet.
///
/// Reading the output:
///   Primary axis acceleration → update the matching table constant in
///   gnc_param_tbl.c (ApproachAccel_mss, BrakeAccel_Hard_mss, etc.).
///   Coupling terms (off-axis) → explain why LAT_CORR commands cause drift.
/// </summary>
public class ThrusterDiagnostic : MonoBehaviour
{
    [Header("References")]
    public RCSModel  rcs;
    public Rigidbody rb;  // auto-found from rcs.vehicle if null

    [Header("Burn durations (seconds)")]
    public float shortBurn  = 0.10f;
    public float longBurn   = 0.95f;

    [Header("Settle time between tests (seconds)")]
    public float settleTime = 0.80f;

    [Header("Observation window after each burn (seconds)")]
    [Tooltip("How long to coast and observe before the position resets. " +
             "Does not affect the velocity measurement, which is taken right at burn-end.")]
    public float observeTime = 1.5f;

    [Header("Torque moment arm — must match GNC_RCS_MOMENT_ARM (m)")]
    public float momentArm = 1.5f;

    [Header("Key to start the sequence")]
    public KeyCode runKey = KeyCode.F8;

    // ── status overlay ─────────────────────────────────────────────────────
    private bool   _running;
    private string _status = "";
    private int    _idx;
    private int    _total;

    // ── home pose, captured at run-start ───────────────────────────────────
    private Vector3    _homePos;
    private Quaternion _homeRot;

    // ── log file ────────────────────────────────────────────────────────────
    private StreamWriter _log;
    private string       _logPath;

    // ─────────────────────────────────────────────────────────────────────

    void Awake()
    {
        if (rcs == null) rcs = GetComponent<RCSModel>();
        if (rb  == null && rcs != null && rcs.vehicle != null)
            rb = rcs.vehicle.GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (Input.GetKeyDown(runKey) && !_running)
            StartCoroutine(RunDiagnostic());
    }

    void OnGUI()
    {
        if (!_running) return;
        var style = new GUIStyle(GUI.skin.box) { fontSize = 14, alignment = TextAnchor.MiddleLeft };
        style.normal.textColor = Color.yellow;
        GUI.Box(new Rect(8, 8, 480, 28), $"  DIAG [{_idx}/{_total}]  {_status}", style);
    }

    // ── main coroutine ─────────────────────────────────────────────────────

    IEnumerator RunDiagnostic()
    {
        if (rcs == null || rb == null)
        {
            Debug.LogError("[DIAG] rcs or rb is null — cannot run.");
            yield break;
        }

        _running  = true;
        _homePos  = rb.position;
        _homeRot  = rb.rotation;

        string logPath = Path.Combine(
            Application.dataPath,
            $"../ThrusterDiagnostic_{System.DateTime.Now:yyyyMMdd_HHmmss}.log");
        _log     = new StreamWriter(logPath, append: false) { AutoFlush = true };
        _logPath = logPath;
        Debug.Log($"[DIAG] Writing to: {Path.GetFullPath(logPath)}");

        float F   = rcs.thrusterForce;   // N per thruster (matches cFS kF)
        float m   = rb.mass;              // kg
        float arm = momentArm;            // m  (matches GNC_RCS_MOMENT_ARM)

        // Brake forces match what cFS actually sends: BrakeAccel × mass.
        // These match gnc_param_tbl.c empirical values — adjust if you retune.
        // Updated 2026-07-11 alongside gnc_param_tbl.c for VehicleMass = 12000.
        float hardBrakeN = 0.189f * m;   // BrakeAccel_Hard_mss × mass
        float softBrakeN = 0.133f * m;   // BrakeAccel_Light_mss × mass

        // label, body-frame force (N), body-frame torque (N·m)
        // Body frame: +Z = toward ISS, +X = right, +Y = up
        var cases = new (string label, Vector3 force, Vector3 torque)[]
        {
            // ── Translation ──────────────────────────────────────────────────
            ("+Fz Approach  (T04-T07)", new Vector3( 0,  0,  F),           Vector3.zero),
            ("-Fz HardBrake (T08-T15)", new Vector3( 0,  0, -hardBrakeN),  Vector3.zero),
            ("-Fz SoftBrake (T08-T11)", new Vector3( 0,  0, -softBrakeN),  Vector3.zero),
            ("+Fx Right               ", new Vector3( F,  0,  0),           Vector3.zero),
            ("-Fx Left                ", new Vector3(-F,  0,  0),           Vector3.zero),
            ("+Fy Up                  ", new Vector3( 0,  F,  0),           Vector3.zero),
            ("-Fy Down                ", new Vector3( 0, -F,  0),           Vector3.zero),

            // ── Attitude ─────────────────────────────────────────────────────
            ("+Tx Pitch up            ", Vector3.zero, new Vector3( F * arm, 0,       0)),
            ("-Tx Pitch down          ", Vector3.zero, new Vector3(-F * arm, 0,       0)),
            ("+Ty Yaw right           ", Vector3.zero, new Vector3(0,        F * arm, 0)),
            ("-Ty Yaw left            ", Vector3.zero, new Vector3(0,       -F * arm, 0)),
            ("+Tz Roll CW             ", Vector3.zero, new Vector3(0, 0,     F * arm)),
            ("-Tz Roll CCW            ", Vector3.zero, new Vector3(0, 0,    -F * arm)),

            // ── Coupling combos — mirror what cFS sends in LAT_CORR ───────────
            // Lateral + approach together (what cFS commands when correcting + drifting)
            ("+Fx+Fz LAT_CORR combo  ", new Vector3( F,  0,  F),           Vector3.zero),
            ("-Fx+Fz LAT_CORR combo  ", new Vector3(-F,  0,  F),           Vector3.zero),

            // ── Combined-demand stress tests ────────────────────────────────────
            // Reproduces the 2026-07-11 14:03 incident: LAT_CORR at a large lateral
            // offset commanded Fx=-400,Fy=-400,Fz=+1200,Tx=+600,Ty=-600 all at once
            // (full lateral burn + spin-rate-override attitude correction + the
            // LAT_CORR +Fz coupling feedforward), and the axial actuator-health
            // monitor (gnc_app.c GNC_APP_CheckActuatorHealth) caught ClosingSpeed_ms
            // going the WRONG direction relative to the commanded +Fz for 100+
            // cycles. Single/dual-axis cases above never exercise 5 simultaneous
            // channels competing for the same 16 thrusters — this does.
            ("5-axis LAT_CORR stress ", new Vector3(-F, -F, 3f * F), new Vector3( F * arm, -F * arm, 0)),
            ("5-axis stress (mirror) ", new Vector3( F,  F,-3f * F), new Vector3(-F * arm,  F * arm, 0)),

            // Every channel maxed simultaneously — beyond anything GNC would
            // realistically send, but finds the allocator's real ceiling.
            ("All-axis max demand    ", new Vector3( F,  F,  F),        new Vector3( F * arm,  F * arm,  F * arm)),

            // Isolates whether simultaneous multi-axis ROTATION alone degrades,
            // independent of the translation/attitude coupling in the cases above.
            ("3-axis attitude combo  ", Vector3.zero,                    new Vector3( F * arm,  F * arm,  F * arm)),
        };

        _total = cases.Length * 2;
        _idx   = 0;

        Log("════ THRUSTER DIAGNOSTIC START ════");
        Log($"thrusterForce={F} N  mass={m} kg  arm={arm} m  " +
            $"theory_accel={F/m:F4} m/s²  theory_alpha={F*arm/(m*4):F4} rad/s²");
        Log($"shortBurn={shortBurn}s  longBurn={longBurn}s  settleTime={settleTime}s");
        Log($"Hard brake command={hardBrakeN:F0} N ({hardBrakeN/m:F4} m/s²)  " +
            $"Soft brake command={softBrakeN:F0} N ({softBrakeN/m:F4} m/s²)");
        Log("Columns: label | dur | ΔVx ΔVy ΔVz (m/s) | ax ay az (m/s²)");
        Log("Off-axis values reveal axial coupling from lateral/attitude commands.");

        foreach (var c in cases)
        {
            foreach (float dur in new[] { shortBurn, longBurn })
            {
                _idx++;
                _status = c.label.TrimEnd();
                yield return ResetAndSettle();
                yield return FireAndMeasure(c.label, c.force, c.torque, dur);
            }
        }

        rcs.ClearExternalControl();
        _running = false;
        _status  = "";
        Log("════ DIAGNOSTIC COMPLETE ════");
        Log($"Log saved to: {Path.GetFullPath(_logPath)}");
        _log.Close();
        _log = null;
    }

    // ── helpers ────────────────────────────────────────────────────────────

    void Log(string msg)
    {
        string line = $"[DIAG] {msg}";
        Debug.Log(line);
        _log?.WriteLine($"{System.DateTime.Now:HH:mm:ss.fff}  {msg}");
    }

    void OnDestroy()
    {
        _log?.Close();
        _log = null;
    }

    IEnumerator ResetAndSettle()
    {
        rcs.ClearExternalControl();
        rb.position        = _homePos;
        rb.rotation        = _homeRot;
        rb.linearVelocity  = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        // Wait one fixed timestep so the physics engine processes the velocity
        // reset before we sample v0 at the start of the next test.
        yield return new WaitForFixedUpdate();
        yield return new WaitForSeconds(settleTime);
    }

    IEnumerator FireAndMeasure(string label, Vector3 force, Vector3 torque, float dur)
    {
        bool hasForce  = force.sqrMagnitude  > 0.01f;
        bool hasTorque = torque.sqrMagnitude > 0.01f;

        Vector3 v0 = rb.linearVelocity;
        Vector3 w0 = rb.angularVelocity;

        rcs.SetWrenchCommand(force, torque, dur);

        // Poll every physics tick for the duration of the burn, OR-accumulating
        // which thrusters were active at any point (rcs.CurrentThrusterMask,
        // bit i = thruster i at >=5% throttle). A single end-of-burn snapshot
        // would miss thrusters whose individual PWM pulse already ended — this
        // gives the full set of thrusters the allocator actually used, which is
        // what you need to tell "allocator dropped thrusters under combined
        // demand" apart from "allocator delivered a weak but honest solution".
        int   firedMask = 0;
        float elapsed   = 0f;
        float window    = dur + 0.05f;  // let burn fully complete in physics
        while (elapsed < window)
        {
            firedMask |= rcs.CurrentThrusterMask;
            yield return new WaitForFixedUpdate();
            elapsed += Time.fixedDeltaTime;
        }

        // Rigidbody.linearVelocity/angularVelocity are world-frame, but commands
        // are body-frame — rotating by the inverse of the attitude captured at
        // ResetAndSettle (_homeRot, which every test case resets rb.rotation to)
        // reports deltas in the same body frame the command was issued in. Without
        // this, results are only clean if the capsule happens to start level;
        // a tilted start attitude bleeds a "pure" body-axis command's effect across
        // all three world axes and reads as spurious coupling.
        Quaternion toBody = Quaternion.Inverse(_homeRot);
        Vector3    dv     = toBody * (rb.linearVelocity  - v0);
        Vector3    dw     = toBody * (rb.angularVelocity - w0);
        Vector3    accel  = dv / dur;
        Vector3    alpha  = dw / dur;

        string line = $"{label,-32} | dur={dur:F2}s";
        if (hasForce)
            line += $" | dV  ({dv.x:+0.0000;-0.0000} {dv.y:+0.0000;-0.0000} {dv.z:+0.0000;-0.0000}) m/s" +
                    $" | a   ({accel.x:+0.0000;-0.0000} {accel.y:+0.0000;-0.0000} {accel.z:+0.0000;-0.0000}) m/s²";
        if (hasTorque)
            line += $" | dW  ({dw.x:+0.0000;-0.0000} {dw.y:+0.0000;-0.0000} {dw.z:+0.0000;-0.0000}) rad/s" +
                    $" | α   ({alpha.x:+0.0000;-0.0000} {alpha.y:+0.0000;-0.0000} {alpha.z:+0.0000;-0.0000}) rad/s²";
        line += $" | fired=[{MaskToList(firedMask)}]";
        Log(line);

        // Coast window: hold position so you can observe the post-burn motion
        // before the next reset. Velocity was already captured above.
        yield return new WaitForSeconds(observeTime);
    }

    static string MaskToList(int mask)
    {
        if (mask == 0) return "none";
        var names = new System.Collections.Generic.List<string>();
        for (int i = 0; i < 16; i++)
            if ((mask & (1 << i)) != 0)
                names.Add($"T{i:D2}");
        return string.Join(",", names);
    }
}
