using UnityEngine;

/// <summary>
/// Replaces real PhysX mesh-to-mesh contact between the chaser petals and the station docking
/// port with a scripted funnel constraint, for the approach phase between "close enough to touch"
/// and full capture (SoftCaptureController's onDock handoff). Raw mesh contact here was the source
/// of every "glitch away" instability worked through before this — convex-hull petal geometry
/// catching an edge of the station's concave mesh at an angle, injecting depenetration/torque
/// impulses PhysX has no way to bound sanely for a mechanism that's *supposed* to interlock.
///
/// First version of this script left lateral offset and attitude error completely unconstrained
/// once collision was removed — no glitching, but also no physical feel at all: a misaligned ship
/// would just float straight through instead of the petals camming it into alignment the way real
/// docking hardware does. This version reintroduces that, scripted rather than via mesh contact: as
/// the ports engage deeper along the docking axis, the ALLOWED lateral offset and attitude error
/// both shrink toward zero (a literal narrowing funnel/cone in position+rotation space), and the
/// chaser port is clamped back onto that funnel's surface whenever it's outside it. A small error
/// gets squeezed out smoothly as the ship slides in; a large one just never gets deep enough
/// (maxInterlockDepth) to be forced fully straight, matching a real funnel's limited catch cone.
///
/// Roll (twist around the docking axis — the "index pins seating into slots" correction, distinct
/// from the cone-angle funnel above) is corrected the same way: allowed roll error narrows toward 0
/// with engagement depth. Rather than re-deriving RelativeNav's alignment-quaternion math (and its
/// rig-specific rollErrorOffset calibration) independently, ApplyRollFunnel drives directly off the
/// already-trusted nav.rollError scalar — same value DockingDetector.rollOk already checks — via a
/// small-angle result: rotating chaserPort by an additional +θ about its own local forward axis
/// changes rollError by -θ (exact when swing/cone-angle error is already ~0, which is the regime
/// this activates in most tightly since ApplyAttitudeFunnel has already been narrowing that down).
/// Self-correcting either way — it's recomputed from the live rollError every FixedUpdate rather
/// than applied open-loop, so even where the small-swing approximation is imperfect it still pulls
/// toward less roll error, not away from it. Worth confirming live: a couple degrees of roll should
/// visibly shrink once engagement starts, not grow.
///
/// RCS, RateDamping, and VehicleState's depenetration/angular-velocity caps are untouched by any of
/// this — this script doesn't suppress or replace them, it just removes the one interaction (mesh
/// contact on the docking colliders) that needed all of that tuning to begin with.
///
/// Collision between the assigned docking colliders is ignored unconditionally (Awake), not just
/// inside the contact envelope — outside contactRange they can't be touching anyway, so there's no
/// downside to leaving it off, and it avoids re-toggling Physics.IgnoreCollision every time range
/// crosses the threshold. Only the docking-specific colliders are ignored (chaser petal tabs,
/// station PMA2/IDA2/IDA2_Details_Petals) — NOT the whole vehicle like SoftCaptureController does
/// post-capture, so a bad approach can still hit the station hull for real.
///
/// Hands off to SoftCaptureController once DockingDetector.isDocked fires; this script goes
/// inactive from that point (checked every FixedUpdate) rather than fighting the joint.
/// </summary>
public class DockingContactConstraint : MonoBehaviour
{
    [Header("Docking")]
    public RelativeNav      nav;
    public DockingDetector  dockingDetector;

    [Header("Vehicles")]
    public Rigidbody chaserRigidbody;
    public Transform chaserPort;
    public Transform targetPort;

    [Header("Docking Colliders")]
    [Tooltip("The chaser's docking-relevant colliders only (e.g. the individual Petals.NNN convex " +
             "hulls under Petal_Rim) — NOT the whole vehicle's hull colliders.")]
    public Collider[] chaserDockingColliders;
    [Tooltip("The station's docking-relevant colliders only (PMA2, IDA2, IDA2_Details_Petals) — " +
             "NOT the whole station.")]
    public Collider[] targetDockingColliders;

    [Header("Contact Envelope")]
    [Tooltip("Port-to-port range (m, matches RelativeNav.range) within which this script is active " +
             "at all. Outside this range the ports can't plausibly be touching, so no correction is " +
             "applied. Should comfortably cover the real petal geometry's reach.")]
    public float contactRange = 0.5f;

    [Tooltip("How far past the target port's face (m, along targetPort.forward) the chaser port is " +
             "allowed to travel before being pushed back — also the depth over which the lateral/" +
             "attitude funnel below narrows from full width down to zero. Approximates how far the " +
             "real petal geometry interlocks; there's no real mesh contact to measure against " +
             "anymore, so this is a pure tuning knob. Tune it live: too small and the ports never " +
             "look like they engage, too large and the visual meshes will noticeably overlap.")]
    public float maxInterlockDepth = 0.12f;

    [Header("Alignment Funnel")]
    [Tooltip("Lateral offset (m) allowed at first touch (axial = 0) — the funnel's mouth. Narrows " +
             "linearly to 0 by the time the port reaches maxInterlockDepth. Wider than " +
             "DockingDetector.maxLateralOffset on purpose: this is the physical catch cone, not the " +
             "final success tolerance.")]
    public float funnelLateralCapture = 0.25f;

    [Tooltip("Attitude error (deg, cone angle only — not roll) allowed at first touch. Narrows " +
             "linearly to 0 by maxInterlockDepth, so a ship a couple degrees off gets progressively " +
             "straightened out as it slides in rather than snapping straight or being allowed through unchanged.")]
    public float funnelAttitudeCaptureDeg = 20f;

    [Tooltip("Roll error (deg, twist around the docking axis — RelativeNav.rollError) allowed at " +
             "first touch. Narrows linearly to 0 by maxInterlockDepth, same shape as the attitude " +
             "funnel above but for the index/roll axis instead of cone angle.")]
    public float funnelRollCaptureDeg = 20f;

    void Awake()
    {
        SetIgnoreCollisions();
    }

    void FixedUpdate()
    {
        if (nav == null || chaserRigidbody == null || chaserPort == null || targetPort == null) return;
        if (dockingDetector != null && dockingDetector.isDocked) return; // SoftCaptureController owns it from here

        if (nav.range > contactRange) return;

        ApplyAxialConstraint();
        ApplyLateralFunnel();
        ApplyAttitudeFunnel();
        ApplyRollFunnel();
    }

    /// <summary>Current engagement depth as a 0..1 fraction of maxInterlockDepth. 0 = not yet
    /// touching (axial >= 0), 1 = fully seated. Drives how tight the funnel below currently is.</summary>
    float EngagementFraction()
    {
        Vector3 portDelta = chaserPort.position - targetPort.position;
        float   axial     = Vector3.Dot(portDelta, targetPort.forward);
        return Mathf.Clamp01(-axial / maxInterlockDepth);
    }

    /// <summary>
    /// One-sided plane constraint along the docking axis: if the chaser port has advanced past the
    /// allowed interlock depth, push it back to exactly that depth and zero the inbound component of
    /// velocity so it doesn't immediately re-penetrate next step. Never pushes the other direction —
    /// approaching from outside maxInterlockDepth is exactly what's supposed to happen.
    /// </summary>
    void ApplyAxialConstraint()
    {
        Vector3 portDelta = chaserPort.position - targetPort.position;
        float   axial     = Vector3.Dot(portDelta, targetPort.forward);
        float   minAxial  = -maxInterlockDepth;

        if (axial >= minAxial) return;

        float pushBack = minAxial - axial;
        chaserRigidbody.position += targetPort.forward * pushBack;

        Vector3 vel      = chaserRigidbody.linearVelocity;
        float   axialVel = Vector3.Dot(vel, targetPort.forward);
        if (axialVel < 0f)
            chaserRigidbody.linearVelocity = vel - targetPort.forward * axialVel;
    }

    /// <summary>
    /// Clamps lateral offset to whatever the funnel currently allows at this engagement depth,
    /// pulling the chaser port radially back onto the funnel's wall — the "petals sliding the ship
    /// into alignment" feel. Zeroes the outward-growing component of lateral velocity so the ship
    /// doesn't keep grinding against the wall.
    /// </summary>
    void ApplyLateralFunnel()
    {
        Vector3 portDelta  = chaserPort.position - targetPort.position;
        float   axial      = Vector3.Dot(portDelta, targetPort.forward);
        Vector3 lateralVec = portDelta - axial * targetPort.forward;
        float   lateralMag = lateralVec.magnitude;

        float allowedLateral = Mathf.Lerp(funnelLateralCapture, 0f, EngagementFraction());
        if (lateralMag <= allowedLateral || lateralMag < 1e-6f) return;

        Vector3 lateralDir = lateralVec / lateralMag;
        chaserRigidbody.position += lateralDir * (allowedLateral - lateralMag);

        Vector3 vel        = chaserRigidbody.linearVelocity;
        float   outwardVel = Vector3.Dot(vel, lateralDir);
        if (outwardVel > 0f)
            chaserRigidbody.linearVelocity = vel - lateralDir * outwardVel;
    }

    /// <summary>
    /// Clamps cone-angle attitude error to whatever the funnel currently allows, rotating the chaser
    /// about the CURRENT chaserPort position (not the vehicle's center of mass) so straightening the
    /// ship out doesn't itself reintroduce a lateral offset.
    /// </summary>
    void ApplyAttitudeFunnel()
    {
        Vector3 currentForward = chaserPort.forward;
        Vector3 desiredForward = -targetPort.forward;
        float   angle          = Vector3.Angle(currentForward, desiredForward);

        float allowedAttitude = Mathf.Lerp(funnelAttitudeCaptureDeg, 0f, EngagementFraction());
        if (angle <= allowedAttitude || angle < 0.01f) return;

        float      fraction   = 1f - allowedAttitude / angle;
        Quaternion deltaRot   = Quaternion.FromToRotation(currentForward, desiredForward);
        Quaternion partialRot = Quaternion.Slerp(Quaternion.identity, deltaRot, fraction);

        Vector3 pivot = chaserPort.position;
        chaserRigidbody.rotation = partialRot * chaserRigidbody.rotation;
        chaserRigidbody.position = pivot + partialRot * (chaserRigidbody.position - pivot);
    }

    /// <summary>
    /// Clamps roll error (twist about the docking axis) to whatever the funnel currently allows.
    /// Derives the correction directly from nav.rollError rather than reconstructing RelativeNav's
    /// calibrated target orientation independently — see class remarks for the small-angle argument
    /// that "additional +θ local-forward rotation on chaserPort ⇒ rollError changes by -θ." Applied
    /// as a world-space delta pivoted at chaserPort.position (same pattern as ApplyAttitudeFunnel)
    /// so it doesn't reintroduce lateral offset.
    /// </summary>
    void ApplyRollFunnel()
    {
        float rollError = nav.rollError;
        float absRoll   = Mathf.Abs(rollError);

        float allowedRoll = Mathf.Lerp(funnelRollCaptureDeg, 0f, EngagementFraction());
        if (absRoll <= allowedRoll) return;

        // Amount to remove from rollError, signed the same way as rollError itself.
        float theta = Mathf.Sign(rollError) * (absRoll - allowedRoll);
        Quaternion localCorrection = Quaternion.AngleAxis(theta, Vector3.forward); // chaserPort's own local +Z

        Vector3    pivot      = chaserPort.position;
        Quaternion worldDelta = chaserPort.rotation * localCorrection * Quaternion.Inverse(chaserPort.rotation);
        chaserRigidbody.rotation = worldDelta * chaserRigidbody.rotation;
        chaserRigidbody.position = pivot + worldDelta * (chaserRigidbody.position - pivot);
    }

    void SetIgnoreCollisions()
    {
        if (chaserDockingColliders == null || targetDockingColliders == null) return;
        foreach (var a in chaserDockingColliders)
        {
            if (a == null) continue;
            foreach (var b in targetDockingColliders)
            {
                if (b == null) continue;
                Physics.IgnoreCollision(a, b, true);
            }
        }
    }
}
