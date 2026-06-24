using UnityEngine;

/// <summary>
/// Docking camera that stays at its placed position/rotation and moves with the chaser port.
/// Arrow keys pan the view; Enter resets to center.
/// </summary>
public class DockingCamera : MonoBehaviour
{
    public Transform chaserPort;

    [Header("Look Controls")]
    public float lookSpeed = 60f;   // degrees per second
    public float maxPitch  = 80f;
    public float maxYaw    = 120f;
    public KeyCode resetLookKey = KeyCode.Return;

    private float pitchOffset;
    private float yawOffset;

    // Capture the camera's initial position/rotation relative to the chaser port so it
    // moves with the chaser without being overridden by an offset calculation.
    private Camera     _cam;
    private Vector3    localPosition;
    private Quaternion localRotation;

    void Start()
    {
        _cam = GetComponent<Camera>();
        if (chaserPort == null) return;
        localPosition = chaserPort.InverseTransformPoint(transform.position);
        localRotation = Quaternion.Inverse(chaserPort.rotation) * transform.rotation;
    }

    void Update()
    {
        if (_cam != null && !_cam.enabled) return;

        float pitchInput = 0f;
        float yawInput   = 0f;

        if (Input.GetKey(KeyCode.UpArrow))    pitchInput = -1f;
        if (Input.GetKey(KeyCode.DownArrow))  pitchInput =  1f;
        if (Input.GetKey(KeyCode.LeftArrow))  yawInput   = -1f;
        if (Input.GetKey(KeyCode.RightArrow)) yawInput   =  1f;

        pitchOffset += pitchInput * lookSpeed * Time.deltaTime;
        yawOffset   += yawInput   * lookSpeed * Time.deltaTime;

        pitchOffset = Mathf.Clamp(pitchOffset, -maxPitch, maxPitch);
        yawOffset   = Mathf.Clamp(yawOffset,   -maxYaw,   maxYaw);

        if (Input.GetKeyDown(resetLookKey))
        {
            pitchOffset = 0f;
            yawOffset   = 0f;
        }
    }

    void LateUpdate()
    {
        if (chaserPort == null) return;
        if (_cam != null && !_cam.enabled) return;

        transform.position = chaserPort.TransformPoint(localPosition);
        transform.rotation = chaserPort.rotation * localRotation * Quaternion.Euler(pitchOffset, yawOffset, 0f);
    }
}
