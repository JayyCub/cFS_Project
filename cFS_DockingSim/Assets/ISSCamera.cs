using UnityEngine;

/// <summary>
/// Robotic pan-tilt-zoom camera for ISS viewpoints.
/// Place this component on a Camera GameObject parented to the ISS — it inherits the
/// ISS transform automatically, so no position code is needed.
///
/// Controls (only active when this Camera component is enabled):
///   Arrow keys          — pan (pitch / yaw)
///   Enter               — reset pan to placed position
///   Scroll wheel        — zoom (FOV)
///   [ / ]               — zoom in / out (keyboard alternative)
///   Backspace           — reset zoom to default FOV
/// </summary>
[RequireComponent(typeof(Camera))]
public class ISSCamera : MonoBehaviour
{
    [Header("Pan")]
    public float lookSpeed = 60f;
    public float maxPitch  = 80f;
    public float maxYaw    = 120f;
    public KeyCode resetPanKey = KeyCode.Return;

    [Header("Zoom")]
    [Tooltip("FOV change per second when holding [ or ].")]
    public float keyZoomSpeed    = 30f;
    [Tooltip("FOV change per scroll wheel tick.")]
    public float scrollZoomSpeed = 5f;
    public float minFov = 5f;
    public float maxFov = 90f;
    public KeyCode resetZoomKey = KeyCode.Backspace;

    private Camera      _cam;
    private float       _pitchOffset;
    private float       _yawOffset;
    private float       _baseFov;
    private Quaternion  _baseLocalRotation;

    void Start()
    {
        _cam               = GetComponent<Camera>();
        _baseFov           = _cam.fieldOfView;
        _baseLocalRotation = transform.localRotation;
    }

    void Update()
    {
        if (!_cam.enabled) return;

        // Pan
        float pitchInput = 0f, yawInput = 0f;
        if (Input.GetKey(KeyCode.UpArrow))    pitchInput = -1f;
        if (Input.GetKey(KeyCode.DownArrow))  pitchInput =  1f;
        if (Input.GetKey(KeyCode.LeftArrow))  yawInput   = -1f;
        if (Input.GetKey(KeyCode.RightArrow)) yawInput   =  1f;

        _pitchOffset += pitchInput * lookSpeed * Time.deltaTime;
        _yawOffset   += yawInput   * lookSpeed * Time.deltaTime;
        _pitchOffset  = Mathf.Clamp(_pitchOffset, -maxPitch, maxPitch);
        _yawOffset    = Mathf.Clamp(_yawOffset,   -maxYaw,   maxYaw);

        if (Input.GetKeyDown(resetPanKey))
        {
            _pitchOffset = 0f;
            _yawOffset   = 0f;
        }

        // Zoom — scroll wheel + [ / ] keys
        float scroll = Input.mouseScrollDelta.y;
        _cam.fieldOfView -= scroll * scrollZoomSpeed;

        if (Input.GetKey(KeyCode.LeftBracket))  _cam.fieldOfView += keyZoomSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.RightBracket)) _cam.fieldOfView -= keyZoomSpeed * Time.deltaTime;

        _cam.fieldOfView = Mathf.Clamp(_cam.fieldOfView, minFov, maxFov);

        if (Input.GetKeyDown(resetZoomKey))
            _cam.fieldOfView = _baseFov;
    }

    void LateUpdate()
    {
        if (!_cam.enabled) return;
        transform.localRotation = _baseLocalRotation * Quaternion.Euler(_pitchOffset, _yawOffset, 0f);
    }
}
