using UnityEngine;

/// <summary>
/// Resets the chaser to its initial position, rotation, and zero velocity.
/// Essential for rapid iteration during cFS GNC testing — no need to restart Play mode.
/// </summary>
public class ScenarioReset : MonoBehaviour
{
    public VehicleState    chaser;
    public RateDamping     rateDamping;
    public DockingDetector detector;

    [Tooltip("Key that triggers a reset.")]
    public KeyCode resetKey = KeyCode.Backspace;

    private Vector3    initialPosition;
    private Quaternion initialRotation;

    void Start()
    {
        if (chaser == null) return;
        initialPosition = chaser.transform.position;
        initialRotation = chaser.transform.rotation;
    }

    void Update()
    {
        if (Input.GetKeyDown(resetKey))
            DoReset();
    }

    public void DoReset()
    {
        if (chaser == null) return;

        chaser.transform.SetPositionAndRotation(initialPosition, initialRotation);

        Rigidbody rb = chaser.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.linearVelocity  = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        detector?.Reset();

        Debug.Log("[RESET] Chaser returned to initial conditions.");
    }
}
