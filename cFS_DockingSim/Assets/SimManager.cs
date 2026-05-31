using UnityEngine;

/// <summary>
/// Logs relative-state telemetry at a fixed interval.
/// Closing speed is positive when approaching, negative when receding.
/// </summary>
public class SimManager : MonoBehaviour
{
    public VehicleState chaser;
    public VehicleState target;

    [Tooltip("Seconds between telemetry log lines.")]
    public float logInterval = 1f;

    private float nextLog;

    void FixedUpdate()
    {
        if (chaser == null || target == null) return;
        if (Time.time < nextLog) return;
        nextLog = Time.time + logInterval;

        Vector3 relPos = chaser.position - target.position;
        Vector3 relVel = chaser.velocity - target.velocity;
        float range = relPos.magnitude;
        float closing = range > 0f ? -Vector3.Dot(relVel, relPos.normalized) : 0f;

        Debug.Log($"[SIM] Range: {range:F2} m | Closing: {closing:+0.000;-0.000} m/s | RelPos: {relPos:F2}");
    }
}
