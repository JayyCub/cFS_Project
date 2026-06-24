using UnityEngine;

/// <summary>
/// Thin wrapper around Unity's Rigidbody for spacecraft state.
///
/// Mass: set directly on the Rigidbody component in the Inspector.
///
/// Inertia: computed automatically from mass + shape parameters using a solid-cylinder
/// approximation (good enough for a capsule + trunk).  Unity's mesh-derived inertia
/// is almost always wrong for spacecraft because it assumes uniform density; this gives
/// a physically-plausible value that scales correctly whenever you change the mass.
///
/// CoM: overridable because mesh-centroid is wrong for spacecraft with concentrated
/// mass (heat shield, engines, batteries) — must be set explicitly.
/// </summary>
public class VehicleState : MonoBehaviour
{
    [Header("Shape (for inertia estimation)")]
    [Tooltip("Outer radius of the vehicle (m). Dragon 2 capsule ≈ 2.0 m.")]
    public float shapeRadius = 2.0f;
    [Tooltip("Total length along the roll axis (m). Dragon 2 capsule + trunk ≈ 6.0 m.")]
    public float shapeLength = 6.0f;

    [Header("Center of Mass")]
    [Tooltip("CoM in local space. Non-zero overrides Unity's mesh-derived value. " +
             "The mesh value is logged at startup so you can read and refine it.")]
    public Vector3 centerOfMassOverride = Vector3.zero;

    private Rigidbody rb;

    // Read-only accessors
    public Vector3    position        => transform.position;
    public Vector3    velocity        => rb != null ? rb.linearVelocity  : Vector3.zero;
    public Quaternion attitude        => transform.rotation;
    public Vector3    angularVelocity => rb != null ? rb.angularVelocity : Vector3.zero;
    public float      mass            => rb != null ? rb.mass            : 0f;
    public Vector3    inertiaTensor   => rb != null ? rb.inertiaTensor   : Vector3.one;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError($"[VehicleState] {gameObject.name} needs a Rigidbody", this);
            return;
        }

        // Apply CoM override first (RCSModel.DelayedInit reads this via rb.centerOfMass).
        Debug.Log($"[VehicleState] {gameObject.name} mesh CoM (local) = {rb.centerOfMass:F3}");
        if (centerOfMassOverride != Vector3.zero)
        {
            rb.centerOfMass = centerOfMassOverride;
            Debug.Log($"[VehicleState] CoM overridden to {centerOfMassOverride:F3}");
        }

        // Compute inertia from current rb.mass + shape.
        // Solid-cylinder approximation — reasonable for a capsule-shaped spacecraft.
        //   Roll  (spin around long axis): I = ½ m r²
        //   Pitch / Yaw (tumble):          I = m (3r² + l²) / 12
        ApplyInertia();
    }

    void ApplyInertia()
    {
        float m = rb.mass;
        float r = shapeRadius;
        float l = shapeLength;

        float iRoll      = 0.5f * m * r * r;
        float iPitchYaw  = m * (3f * r * r + l * l) / 12f;

        rb.inertiaTensor         = new Vector3(iPitchYaw, iPitchYaw, iRoll);
        rb.inertiaTensorRotation = Quaternion.identity;

        Debug.Log($"[VehicleState] mass={m:F0} kg  " +
                  $"I_pitch/yaw={iPitchYaw:F0}  I_roll={iRoll:F0} kg·m²  " +
                  $"(r={r} m  l={l} m)");
    }

    // Called from the Inspector gear menu or after changing mass at runtime.
    [ContextMenu("Recalculate Inertia")]
    public void RecalculateInertia()
    {
        rb = GetComponent<Rigidbody>();
        if (rb != null) ApplyInertia();
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
