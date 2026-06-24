using UnityEngine;

/// <summary>
/// Computes approach navigation state between two docking ports each physics step.
/// All other Phase 2+ components (DockingDetector, DockingHUD, cFS telemetry) read from here.
/// </summary>
public class RelativeNav : MonoBehaviour
{
    [Header("Vehicles")]
    public VehicleState chaser;
    public VehicleState target;

    [Header("Docking Ports")]
    public Transform chaserPort;
    public Transform targetPort;

    /// <summary>Distance between port faces in meters.</summary>
    public float range { get; private set; }

    /// <summary>Positive = approaching, negative = receding (m/s).</summary>
    public float closingSpeed { get; private set; }

    /// <summary>Perpendicular distance from the docking axis (meters).</summary>
    public float lateralOffset { get; private set; }

    /// <summary>Angle from perfect port alignment (degrees). 0 = ready to dock.</summary>
    public float attitudeError { get; private set; }

    /// <summary>Per-axis attitude errors (degrees, [-180,180]). 0 = aligned on that axis.</summary>
    public float pitchError { get; private set; }
    public float yawError   { get; private set; }
    public float rollError  { get; private set; }

    void FixedUpdate()
    {
        if (chaserPort == null || targetPort == null) return;

        // Range between port faces
        Vector3 portDelta = chaserPort.position - targetPort.position;
        range = portDelta.magnitude;

        // Closing speed derived from Rigidbody velocities — more stable than differencing range
        if (chaser != null && target != null && range > 0f)
        {
            Vector3 relVel = chaser.velocity - target.velocity;
            closingSpeed = -Vector3.Dot(relVel, portDelta.normalized);
        }

        // Lateral offset: how far the chaser port is from the docking axis
        // Axis is defined by targetPort.forward (the approach corridor direction)
        float axial = Vector3.Dot(portDelta, targetPort.forward);
        lateralOffset = (portDelta - axial * targetPort.forward).magnitude;

        // Scalar attitude error: 0° when ports are perfectly anti-parallel (facing each other)
        attitudeError = Vector3.Angle(chaserPort.forward, -targetPort.forward);

        // Per-axis attitude errors: decompose the error quaternion from chaserPort to the
        // target docking orientation (chaserPort.forward = -targetPort.forward, ups aligned).
        Quaternion targetRot = Quaternion.LookRotation(-targetPort.forward, targetPort.up);
        Quaternion errQuat   = Quaternion.Inverse(chaserPort.rotation) * targetRot;
        Vector3    euler     = errQuat.eulerAngles;
        pitchError = euler.x > 180f ? euler.x - 360f : euler.x;
        yawError   = euler.y > 180f ? euler.y - 360f : euler.y;
        rollError  = euler.z > 180f ? euler.z - 360f : euler.z;
    }
}
