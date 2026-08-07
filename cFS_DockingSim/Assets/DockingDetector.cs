using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Monitors RelativeNav each physics step and declares a successful dock
/// when all four conditions are simultaneously within threshold.
/// Fires onDock once — future cFS telemetry handshake hooks in here.
/// </summary>
public class DockingDetector : MonoBehaviour
{
    public RelativeNav nav;

    [Header("Docking Thresholds")]
    public float maxRange         = 0.15f;  // meters
    public float maxClosingSpeed  = 0.30f;  // m/s  (must also be > 0 — actually approaching)
    public float maxLateralOffset = 0.10f;  // meters
    public float maxAttitudeError = 10f;    // degrees
    public float maxRollError     = 10f;    // degrees — twist around the docking axis (petal/slot indexing)

    [Header("Debug")]
    [Tooltip("Once within this range, logs which threshold(s) are failing (throttled) so a failed " +
             "approach is diagnosable instead of silent.")]
    public float diagnosticRange    = 1.0f;
    public float diagnosticInterval = 0.5f;
    private float _nextDiagLog;

    public bool isDocked { get; private set; }

    public UnityEvent onDock;

    void FixedUpdate()
    {
        if (isDocked || nav == null) return;

        bool rangeOk    = nav.range          <= maxRange;
        bool speedOk    = nav.closingSpeed   >  0f && nav.closingSpeed <= maxClosingSpeed;
        bool lateralOk  = nav.lateralOffset  <= maxLateralOffset;
        bool attitudeOk = nav.attitudeError  <= maxAttitudeError;
        // attitudeError only measures the cone angle between the two ports' forward axes — it's
        // blind to twist around the docking axis. Without also checking rollError, two ports facing
        // each other dead-on but rotated ~90° apart pass as "aligned," and the petals end up jammed
        // into the ISS ring's solid sections instead of its slots.
        bool rollOk     = Mathf.Abs(nav.rollError) <= maxRollError;

        if (nav.range <= diagnosticRange && Time.time >= _nextDiagLog)
        {
            _nextDiagLog = Time.time + diagnosticInterval;
            Debug.Log($"[DOCK] Approaching — " +
                      $"Range:{nav.range:F3}m[{(rangeOk ? "OK" : "FAIL")}] " +
                      $"Speed:{nav.closingSpeed:F3}m/s[{(speedOk ? "OK" : "FAIL")}] " +
                      $"Lateral:{nav.lateralOffset:F3}m[{(lateralOk ? "OK" : "FAIL")}] " +
                      $"Attitude:{nav.attitudeError:F1}°[{(attitudeOk ? "OK" : "FAIL")}] " +
                      $"Roll:{nav.rollError:F1}°[{(rollOk ? "OK" : "FAIL")}]");
        }

        if (rangeOk && speedOk && lateralOk && attitudeOk && rollOk)
        {
            isDocked = true;
            Debug.Log($"[DOCK] SUCCESS — Range: {nav.range:F3} m | " +
                      $"Speed: {nav.closingSpeed:F3} m/s | " +
                      $"Lateral: {nav.lateralOffset:F3} m | " +
                      $"Attitude: {nav.attitudeError:F1}° | " +
                      $"Roll: {nav.rollError:F1}°");
            onDock?.Invoke();
        }
    }

    public void Reset()
    {
        isDocked = false;
    }
}
