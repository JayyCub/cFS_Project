using UnityEngine;

/// <summary>
/// Defines a cone-shaped legal approach zone along the docking axis.
/// The chaser must stay within the cone angle for a valid approach.
/// Draws a gizmo cone in the Scene view for visual reference.
/// </summary>
public class ApproachCorridor : MonoBehaviour
{
    [Header("References")]
    public RelativeNav nav;
    public Transform targetPort;
    public Transform chaserPort;

    [Header("Corridor Shape")]
    [Tooltip("Half-angle of the approach cone in degrees.")]
    public float coneHalfAngle = 15f;
    [Tooltip("Maximum range at which corridor check is active (meters).")]
    public float maxRange = 20f;

    /// <summary>True when the chaser port is inside the approach cone.</summary>
    public bool inCorridor { get; private set; }

    /// <summary>Current angle from the docking axis (degrees).</summary>
    public float corridorAngle { get; private set; }

    void FixedUpdate()
    {
        if (targetPort == null || chaserPort == null || nav == null)
        {
            inCorridor = false;
            return;
        }

        // Beyond max range the corridor check doesn't apply yet
        if (nav.range > maxRange)
        {
            inCorridor    = false;
            corridorAngle = 0f;
            return;
        }

        // Angle between the docking axis and the vector from target port to chaser port.
        // targetPort.forward points toward the chaser when ports are set up correctly.
        Vector3 toChaser = (chaserPort.position - targetPort.position).normalized;
        corridorAngle = Vector3.Angle(targetPort.forward, toChaser);
        inCorridor    = corridorAngle <= coneHalfAngle;
    }

    // Draws the approach corridor cone in the Scene view
    void OnDrawGizmos()
    {
        if (targetPort == null) return;

        int segments  = 32;
        float length  = maxRange;
        float radius  = Mathf.Tan(coneHalfAngle * Mathf.Deg2Rad) * length;

        Gizmos.color = inCorridor ? new Color(0f, 1f, 0f, 0.25f) : new Color(1f, 0.5f, 0f, 0.25f);

        Vector3 apex = targetPort.position;
        Vector3 tip  = apex + targetPort.forward * length;

        // Draw cone edge lines from apex
        for (int i = 0; i < segments; i++)
        {
            float angle = i * 360f / segments * Mathf.Deg2Rad;
            Vector3 right = targetPort.right;
            Vector3 up    = targetPort.up;
            Vector3 edge  = tip + (right * Mathf.Cos(angle) + up * Mathf.Sin(angle)) * radius;
            Gizmos.DrawLine(apex, edge);
        }

        // Draw rim circle at the base of the cone
        for (int i = 0; i < segments; i++)
        {
            float a0 = i       * 360f / segments * Mathf.Deg2Rad;
            float a1 = (i + 1) * 360f / segments * Mathf.Deg2Rad;
            Vector3 right = targetPort.right;
            Vector3 up    = targetPort.up;
            Vector3 p0 = tip + (right * Mathf.Cos(a0) + up * Mathf.Sin(a0)) * radius;
            Vector3 p1 = tip + (right * Mathf.Cos(a1) + up * Mathf.Sin(a1)) * radius;
            Gizmos.DrawLine(p0, p1);
        }

        // Draw center axis line
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(apex, tip);
    }
}
