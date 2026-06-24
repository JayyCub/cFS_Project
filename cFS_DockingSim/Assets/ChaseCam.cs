using UnityEngine;

/// <summary>
/// Third-person follow camera that trails the Dragon capsule.
/// Position is smoothed with SmoothDamp; rotation smoothly looks at the target.
/// Offset is in the target's local space, so the camera stays behind Dragon
/// regardless of which way it's pointing.
///
/// Snaps instantly to the correct position the first frame it becomes active
/// so there is no jarring lerp-from-the-wrong-place on camera switch.
/// </summary>
[RequireComponent(typeof(Camera))]
public class ChaseCam : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The Dragon capsule root transform to follow.")]
    public Transform target;

    [Header("Follow")]
    [Tooltip("Offset from target in target's local space. Negative Z = behind, positive Y = above.")]
    public Vector3 localOffset = new Vector3(0f, 3f, -14f);
    [Tooltip("Position smoothing time (seconds). Lower = snappier.")]
    public float positionSmoothTime  = 0.4f;
    [Tooltip("Rotation smoothing speed (higher = snappier).")]
    public float rotationSmoothSpeed = 6f;

    private Camera  _cam;
    private Vector3 _velPos;
    private bool    _wasActive;

    void Start()
    {
        _cam = GetComponent<Camera>();
    }

    void LateUpdate()
    {
        if (_cam == null || !_cam.enabled || target == null) return;

        Vector3 desiredPos = target.TransformPoint(localOffset);

        if (!_wasActive)
        {
            // First frame active — snap so there is no lerp from a stale position.
            transform.position = desiredPos;
            _velPos            = Vector3.zero;
            _wasActive         = true;
        }
        else
        {
            transform.position = Vector3.SmoothDamp(
                transform.position, desiredPos, ref _velPos, positionSmoothTime);
        }

        // Always look at the target, smoothly
        Vector3    toTarget    = target.position - transform.position;
        Quaternion desiredRot  = toTarget.sqrMagnitude > 0.001f
            ? Quaternion.LookRotation(toTarget, Vector3.up)
            : transform.rotation;

        transform.rotation = Quaternion.Slerp(
            transform.rotation, desiredRot, rotationSmoothSpeed * Time.deltaTime);
    }

    // Called implicitly when Camera is disabled (via CameraManager) to reset
    // the snap flag so the next activation re-snaps cleanly.
    void OnDisable() { }   // Unity hook — intentionally empty, used below

    public void OnCameraDeactivated() => _wasActive = false;
}
