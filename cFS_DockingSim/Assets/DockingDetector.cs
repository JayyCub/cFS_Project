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

    public bool isDocked { get; private set; }

    public UnityEvent onDock;

    void FixedUpdate()
    {
        if (isDocked || nav == null) return;

        bool rangeOk    = nav.range          <= maxRange;
        bool speedOk    = nav.closingSpeed   >  0f && nav.closingSpeed <= maxClosingSpeed;
        bool lateralOk  = nav.lateralOffset  <= maxLateralOffset;
        bool attitudeOk = nav.attitudeError  <= maxAttitudeError;

        if (rangeOk && speedOk && lateralOk && attitudeOk)
        {
            isDocked = true;
            Debug.Log($"[DOCK] SUCCESS — Range: {nav.range:F3} m | " +
                      $"Speed: {nav.closingSpeed:F3} m/s | " +
                      $"Lateral: {nav.lateralOffset:F3} m | " +
                      $"Attitude: {nav.attitudeError:F1}°");
            onDock?.Invoke();
        }
    }

    public void Reset()
    {
        isDocked = false;
    }
}
