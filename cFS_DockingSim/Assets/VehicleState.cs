using UnityEngine;

/// <summary>
/// Thin wrapper around Unity's Rigidbody for spacecraft state.
/// Provides the interface RCSModel and cFS will use to command the vehicle.
/// </summary>
public class VehicleState : MonoBehaviour
{
    [Header("Mass Properties")]
    [Tooltip("Set > 0 to override the Rigidbody mass (kg).")]
    public float massOverride = 0f;
    [Tooltip("Principal-axis inertia tensor (kg·m²). Non-zero overrides Unity's implicit calculation.")]
    public Vector3 inertiaTensorOverride = Vector3.zero;

    private Rigidbody rb;

    // Read-only accessors for telemetry
    public Vector3 position       => transform.position;
    public Vector3 velocity       => rb != null ? rb.linearVelocity : Vector3.zero;
    public Quaternion attitude    => transform.rotation;
    public Vector3 angularVelocity => rb != null ? rb.angularVelocity : Vector3.zero;
    public float mass             => rb != null ? rb.mass : 0f;
    public Vector3 inertiaTensor  => rb != null ? rb.inertiaTensor : Vector3.one;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError($"[VehicleState] {gameObject.name} needs a Rigidbody", this);
            return;
        }

        if (massOverride > 0f)
            rb.mass = massOverride;

        if (inertiaTensorOverride != Vector3.zero)
        {
            rb.inertiaTensor = inertiaTensorOverride;
            rb.inertiaTensorRotation = Quaternion.identity;
        }
    }

    public void AddForce(Vector3 worldForce)
    {
        if (rb != null) rb.AddForce(worldForce, ForceMode.Force);
    }

    public void AddTorque(Vector3 worldTorque)
    {
        if (rb != null) rb.AddTorque(worldTorque, ForceMode.Force);
    }
}
