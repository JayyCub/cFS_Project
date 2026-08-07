using UnityEngine;

/// <summary>
/// Replaces sustained mesh-collider contact between the chaser and target with a
/// compliant ConfigurableJoint once DockingDetector reports a successful capture.
/// Continuous PhysX contact between the 12,000 kg dynamic capsule and the kinematic
/// ISS was the source of the "glitch away" instability on prolonged contact — this
/// mutually ignores collision between the two vehicles' colliders and hands off to
/// a joint instead.
///
/// The joint's rest pose is captured from the ACTUAL relative position at the instant
/// of capture (chaserPort's current world position), not the idealized perfectly-aligned
/// point — DockingDetector allows up to ~0.15m/10° of residual misalignment at capture,
/// and anchoring to the idealized point meant the joint sprang that whole gap shut the
/// moment it engaged (felt like a magnet snap, and clipped the petals/rims through each
/// other since collisions are off). Anchoring to the as-captured pose means zero travel
/// at t=0: linear motion gets a small free-float envelope with a firm (non-drive) stop
/// at the edge — "a little give" with no continuous pull in any direction. Angular motion
/// is fully Locked rather than also given a small limit: ConfigurableJoint's angular
/// limits are measured against an axis/secondaryAxis-dependent reference frame that can't
/// be verified without running the Editor, and getting it wrong would reintroduce a snap.
/// Locked is unambiguous — freezes wherever it is right now — so it's the safe choice
/// until a small angular give can be tuned and confirmed live.
///
/// Press U to undock (debug convenience only, for re-testing an approach without
/// restarting Play mode — not a hard-dock/undock feature).
/// </summary>
public class SoftCaptureController : MonoBehaviour
{
    [Header("Docking")]
    public DockingDetector dockingDetector;
    public RCSModel        rcsModel;
    public RateDamping     rateDamping;

    [Header("Vehicles")]
    public Rigidbody chaserRigidbody;
    public Rigidbody targetRigidbody;
    public Transform chaserPort;
    public Transform targetPort;

    [Header("Linear Give")]
    [Tooltip("Radius (meters) the joint can float within before the stop resists. NOTE: if this " +
             "component already exists in the scene, the Inspector value overrides this default and " +
             "won't update from code changes — check/reset it by hand.")]
    public float linearEnvelope = 0.02f;
    [Tooltip("Explicit non-zero spring — relying on spring=0 as an implicit 'hard stop' wasn't " +
             "reliably arresting the chaser's momentum (12,000 kg), so this is a real restoring force " +
             "now, not an assumed rigid constraint. Tune live if it still lets the capsule punch through.")]
    public float linearStopSpring = 500000f;
    public float linearStopDamper = 150000f;

    private ConfigurableJoint _joint;
    private Collider[]        _chaserColliders;
    private Collider[]        _targetColliders;
    private bool               _captured;

    void Awake()
    {
        if (dockingDetector == null)
        {
            Debug.LogError("[SoftCaptureController] dockingDetector not assigned!", this);
            return;
        }
        dockingDetector.onDock.AddListener(EngageSoftCapture);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.U) && _captured)
            Undock();
    }

    public void EngageSoftCapture()
    {
        if (_captured) return;
        if (chaserRigidbody == null || targetRigidbody == null || chaserPort == null || targetPort == null)
        {
            Debug.LogError("[SoftCaptureController] Missing vehicle/port reference — cannot engage soft capture.", this);
            return;
        }

        _captured = true;

        if (rcsModel != null) rcsModel.suppressForces = true;
        if (rateDamping != null) rateDamping.SetActive(false);

        _chaserColliders = chaserRigidbody.GetComponentsInChildren<Collider>();
        _targetColliders = targetRigidbody.GetComponentsInChildren<Collider>();
        SetIgnoreCollisions(true);

        // Rest pose = wherever the chaser port ACTUALLY is right now, not the idealized
        // aligned point — anchor and connectedAnchor both resolve to this same world point,
        // so linear error is exactly zero at the instant the joint engages. No travel, no snap.
        Vector3 capturedWorldPoint = chaserPort.position;

        _joint = chaserRigidbody.gameObject.AddComponent<ConfigurableJoint>();
        _joint.connectedBody = targetRigidbody;
        _joint.autoConfigureConnectedAnchor = false;
        _joint.anchor          = chaserRigidbody.transform.InverseTransformPoint(capturedWorldPoint);
        _joint.connectedAnchor = targetRigidbody.transform.InverseTransformPoint(capturedWorldPoint);

        _joint.xMotion = _joint.yMotion = _joint.zMotion = ConfigurableJointMotion.Limited;
        _joint.angularXMotion = _joint.angularYMotion = _joint.angularZMotion = ConfigurableJointMotion.Locked;

        _joint.linearLimit = new SoftJointLimit { limit = linearEnvelope };
        _joint.linearLimitSpring = new SoftJointLimitSpring { spring = linearStopSpring, damper = linearStopDamper };

        _joint.breakForce  = Mathf.Infinity;
        _joint.breakTorque = Mathf.Infinity;

        // Reuses DockingDetector's own nav readings rather than recomputing pose error here —
        // an earlier version recomputed it independently via a raw Quaternion.Angle against the
        // uncalibrated target reference, which disagreed with RelativeNav's (calibrated) rollError
        // and caused a confusing false alarm. One source of truth for "how aligned was capture."
        float posErrorAtCapture = Vector3.Distance(chaserPort.position, targetPort.position);
        float attitudeAtCapture = dockingDetector.nav != null ? dockingDetector.nav.attitudeError : -1f;
        float rollAtCapture     = dockingDetector.nav != null ? dockingDetector.nav.rollError     : -1f;
        Debug.Log($"[SoftCaptureController] Soft capture engaged — RCS/RateDamping suppressed, " +
                  $"chaser/target collisions ignored. Residual pose error at contact: " +
                  $"{posErrorAtCapture:F3} m / attitude {attitudeAtCapture:F1}° / roll {rollAtCapture:F1}° " +
                  $"(rotation now locked there; position gets ±{linearEnvelope:F3} m of float before the stop).");
    }

    public void Undock()
    {
        if (!_captured) return;
        _captured = false;

        if (_joint != null) Destroy(_joint);
        _joint = null;

        SetIgnoreCollisions(false);
        _chaserColliders = null;
        _targetColliders = null;

        if (rcsModel != null) rcsModel.suppressForces = false;
        if (rateDamping != null) rateDamping.SetActive(true);
        if (dockingDetector != null) dockingDetector.Reset();

        Debug.Log("[SoftCaptureController] Undocked — collisions/forces restored, ready to re-approach.");
    }

    void SetIgnoreCollisions(bool ignore)
    {
        if (_chaserColliders == null || _targetColliders == null) return;
        foreach (var a in _chaserColliders)
        {
            if (a == null) continue;
            foreach (var b in _targetColliders)
            {
                if (b == null) continue;
                Physics.IgnoreCollision(a, b, ignore);
            }
        }
    }
}
