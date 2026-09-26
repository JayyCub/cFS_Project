using System.Collections;
using System.IO;
using UnityEngine;

/// <summary>
/// Thruster hardware bench test — the simulation equivalent of firing valves from ground
/// support equipment, bypassing flight software.
///
/// Fires fixed thruster groups, then (optionally) every active thruster alone, at two burn
/// durations, and logs the measured body-frame ΔV / Δω next to what the scene's thruster
/// geometry predicts (RCSModel.ImpulseFromOnTimes ÷ mass / inertia). A thruster that is
/// dead, weak, mis-canted or wired to the wrong index shows up as measured ≠ predicted.
///
/// Since realism phase 2 there are no GNC acceleration constants to calibrate any more
/// (GNC requests impulse, the cFS RCS app allocates it from its thruster table), so this is
/// now purely a hardware check. If the check passes but flight looks wrong, compare the cFS
/// table (apps/rcs/fsw/tables/rcs_thr_tbl.c) against RCSModel's "Log cFS thruster table"
/// context-menu output instead.
///
/// Setup:
///   1. Add this component to any active GameObject; drag the RCSModel into "rcs".
///      The Rigidbody is auto-found from rcs.vehicle.
///   2. Run with cFS stopped (or ABORTed) so flight software isn't commanding valves too.
///   3. Press F8. Results go to the Console and to cFS_DockingSim/ThrusterDiagnostic_*.log.
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

    [Header("Individual thruster sweep")]
    [Tooltip("After the group cases, fire each active thruster alone at both burn durations.")]
    public bool includeIndividualThrusters = true;

    [Header("Key to start the sequence")]
    public KeyCode runKey = KeyCode.F8;

    // Named thruster groups (bit i = thruster Ti) — see Docs/RCS_THRUSTER_REFERENCE.md.
    static readonly (string label, int mask)[] Groups =
    {
        ("Approach    (T04-T07)", 0x00F0),
        ("Brake-Yaw   (T08-T11)", 0x0F00),
        ("Brake-Pitch (T12-T15)", 0xF000),
        ("All brake   (T08-T15)", 0xFF00),
    };

    private bool   _running;
    private string _status = "";
    private int    _idx;
    private int    _total;

    private Vector3    _homePos;
    private Quaternion _homeRot;

    private StreamWriter _log;
    private string       _logPath;

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

        _logPath = Path.Combine(Application.dataPath,
                                $"../ThrusterDiagnostic_{System.DateTime.Now:yyyyMMdd_HHmmss}.log");
        _log = new StreamWriter(_logPath, append: false) { AutoFlush = true };
        Debug.Log($"[DIAG] Writing to: {Path.GetFullPath(_logPath)}");

        var active = new System.Collections.Generic.List<int>();
        for (int i = 0; i < rcs.ThrusterCount; i++)
            if (rcs.IsThrusterActive(i)) active.Add(i);

        _total = Groups.Length * 2 + (includeIndividualThrusters ? active.Count * 2 : 0);
        _idx   = 0;

        Log("════ THRUSTER BENCH TEST START ════");
        Log($"thrusterForce={rcs.thrusterForce} N  mass={rb.mass} kg  inertia={rb.inertiaTensor:F0} kg·m²");
        Log($"shortBurn={shortBurn}s  longBurn={longBurn}s  settleTime={settleTime}s");
        Log("Columns: label | dur | measured dV (m/s) | predicted dV | measured dW (rad/s) | predicted dW | fired");

        foreach (var g in Groups)
            foreach (float dur in new[] { shortBurn, longBurn })
            {
                _idx++;
                _status = g.label;
                yield return ResetAndSettle();
                yield return FireMaskAndMeasure(g.label, g.mask, dur);
            }

        if (includeIndividualThrusters)
        {
            Log("──── INDIVIDUAL THRUSTER SWEEP ────");
            foreach (int i in active)
                foreach (float dur in new[] { shortBurn, longBurn })
                {
                    _idx++;
                    _status = $"T{i:D2} solo";
                    yield return ResetAndSettle();
                    yield return FireMaskAndMeasure($"T{i:D2} solo", 1 << i, dur);
                }
        }

        rcs.ClearExternalControl();
        _running = false;
        _status  = "";
        Log("════ BENCH TEST COMPLETE ════");
        Log($"Log saved to: {Path.GetFullPath(_logPath)}");
        _log.Close();
        _log = null;
    }

    void Log(string msg)
    {
        Debug.Log($"[DIAG] {msg}");
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

    IEnumerator FireMaskAndMeasure(string label, int mask, float dur)
    {
        Vector3 v0 = rb.linearVelocity;
        Vector3 w0 = rb.angularVelocity;

        var onTimes = new float[rcs.ThrusterCount];
        for (int i = 0; i < onTimes.Length && i < 32; i++)
            onTimes[i] = (mask & (1 << i)) != 0 ? dur : 0f;
        rcs.ImpulseFromOnTimes(onTimes, out Vector3 lin, out Vector3 ang);
        Vector3 I         = rb.inertiaTensor; // principal, body axes (VehicleState sets rotation = identity)
        Vector3 predDv    = lin / rb.mass;
        Vector3 predDw    = new Vector3(ang.x / I.x, ang.y / I.y, ang.z / I.z);

        rcs.SetThrusterCommand(mask, dur);

        // OR-accumulate which thrusters actually fired over the whole burn.
        int   firedMask = 0;
        float elapsed   = 0f;
        float window    = dur + 0.05f;
        while (elapsed < window)
        {
            firedMask |= rcs.CurrentThrusterMask;
            yield return new WaitForFixedUpdate();
            elapsed += Time.fixedDeltaTime;
        }

        // Rigidbody velocities are world-frame; rotate into the body frame of the
        // home attitude (every test starts there) so a tilted start doesn't read as coupling.
        Quaternion toBody = Quaternion.Inverse(_homeRot);
        Vector3    dv     = toBody * (rb.linearVelocity  - v0);
        Vector3    dw     = toBody * (rb.angularVelocity - w0);

        string line = $"{label,-24} | dur={dur:F2}s" +
                      $" | dV {V(dv)} pred {V(predDv)}" +
                      $" | dW {V(dw)} pred {V(predDw)}" +
                      $" | fired=[{MaskToList(firedMask)}]";
        if (firedMask != mask)
            line += "  *** UNEXPECTED FIRED MASK ***";
        if (Mismatch(dv, predDv) || Mismatch(dw, predDw))
            line += "  *** MEASURED != PREDICTED ***";
        Log(line);

        yield return new WaitForSeconds(observeTime);
    }

    // >20% or clearly-above-noise disagreement between a measurement and its prediction
    static bool Mismatch(Vector3 measured, Vector3 predicted)
    {
        float err = (measured - predicted).magnitude;
        return err > 0.2f * predicted.magnitude + 1e-4f;
    }

    static string V(Vector3 v) =>
        $"({v.x:+0.0000;-0.0000} {v.y:+0.0000;-0.0000} {v.z:+0.0000;-0.0000})";

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
