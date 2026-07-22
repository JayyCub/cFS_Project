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

    /// <summary>
    /// Signed lateral offset components, in WORLD X/Y axes (meters). Unlike
    /// GNC's old Pos_X/Pos_Y (chaser transform origin vs. world origin), these
    /// measure the docking PORT's position relative to the actual corridor
    /// axis — the port is offset from the chaser's origin by a fixed moment
    /// arm, so attitude changes sweep the port sideways in world space even
    /// while the origin doesn't move. Using the origin as a stand-in for the
    /// port meant a rotating-but-not-translating chaser could drift the port
    /// off-axis with the lateral controller seeing zero error.
    ///
    /// CORRECTED 2026-07-18: originally decomposed against targetPort's own
    /// right/up axes, not world X/Y. That was a real second bug, not just a
    /// style choice — GNC's Vel_X/Y (and Pos_X/Y, used for the CW feedforward)
    /// are genuine world-frame quantities, and the lateral controller compares
    /// LatOffset_X/Y against Vel_X/Y directly (v_err = v_tgt(from LatOffset) -
    /// Vel_X). Mixing a target-port-local position against a world-frame
    /// velocity silently compares two different axes whenever the target
    /// port's orientation isn't world-axis-aligned — confirmed in telemetry:
    /// VelX held steady at +0.05 (the controller's own target speed, so it
    /// read as "satisfied" and mostly stopped firing) while LatOffX kept
    /// growing steadily more negative the entire time, completely unmoved by
    /// that "positive" velocity. lateralVec is already computed perpendicular
    /// to the corridor axis (targetPort.forward); taking its raw world X/Y
    /// components keeps the port as the reference point while staying in the
    /// same frame as everything else in the control law.
    /// </summary>
    public float lateralOffsetX { get; private set; }
    public float lateralOffsetY { get; private set; }

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
        float   axial      = Vector3.Dot(portDelta, targetPort.forward);
        Vector3 lateralVec = portDelta - axial * targetPort.forward;
        lateralOffset  = lateralVec.magnitude;
        lateralOffsetX = lateralVec.x;
        lateralOffsetY = lateralVec.y;

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
