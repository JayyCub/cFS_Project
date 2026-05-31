using UnityEngine;

/// <summary>
/// Drives linear and angular velocity toward zero using available thruster authority.
/// Directly writes rb.angularVelocity / rb.linearVelocity to avoid ForceMode ambiguity.
/// Toggle with H.
/// </summary>
public class RateDamping : MonoBehaviour
{
    public VehicleState vehicle;
    [Tooltip("When assigned, RDM backs off on any axis where thrusters are actively firing.")]
    public RCSModel rcsModel;

    [Header("Thruster Authority (match RCSModel values)")]
    public float thrusterForce = 10f;
    public float momentArm     = 1.5f;

    [Header("Authority Fraction")]
    [Range(0f, 1f)]
    public float linearGain  = 0.8f;
    [Range(0f, 1f)]
    public float angularGain = 0.8f;

    [Header("Inertia")]
    [Tooltip("Must match your VehicleState inertiaTensorOverride value (kg·m²). " +
             "Set explicitly here to avoid Unity runtime override issues.")]
    public float inertiaTensor = 80f;

    [Header("Deadbands")]
    public float linearDeadband  = 0.005f;  // m/s
    public float angularDeadband = 0.1f;    // deg/s

    [Header("Debug")]
    [Tooltip("Logs angular velocity and correction each physics step.")]
    public bool debugLog = false;

    public bool isActive { get; private set; } = false;

    private Rigidbody rb;

    void Start()
    {
        if (vehicle == null)
        {
            Debug.LogError("[RateDamping] vehicle not assigned!", this);
            return;
        }

        rb = vehicle.GetComponent<Rigidbody>();

        if (rb == null)
            Debug.LogError("[RateDamping] no Rigidbody found on vehicle!", this);
        else
            Debug.Log($"[RateDamping] rb.inertiaTensor at Start = {rb.inertiaTensor}, " +
                      $"using inertiaTensor field = {inertiaTensor}");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.H))
            isActive = !isActive;
    }

    void FixedUpdate()
    {
        if (!isActive || vehicle == null || rb == null) return;

        NullLinearVelocity();
        NullAngularVelocity();
    }

    // Bits 0-5 = translation thrusters, bits 6-11 = rotation thrusters
    private const int TranslationBits = 0b0000_0011_1111;
    private const int RotationBits    = 0b1111_1100_0000;

    void NullLinearVelocity()
    {
        // Back off if any translation thruster is commanding motion this frame
        if (rcsModel != null && (rcsModel.CurrentThrusterMask & TranslationBits) != 0) return;

        Vector3 vel = rb.linearVelocity;
        if (vel.magnitude <= linearDeadband) return;

        float maxDeltaV  = thrusterForce * linearGain / vehicle.mass * Time.fixedDeltaTime;
        float reduction  = Mathf.Min(vel.magnitude, maxDeltaV);
        rb.linearVelocity = vel - vel.normalized * reduction;
    }

    void NullAngularVelocity()
    {
        // Back off if any rotation thruster is commanding motion this frame
        if (rcsModel != null && (rcsModel.CurrentThrusterMask & RotationBits) != 0) return;

        Vector3 angVel   = rb.angularVelocity;
        float   angSpeed = angVel.magnitude * Mathf.Rad2Deg;

        if (debugLog)
            Debug.Log($"[RateDamping] angVel={angSpeed:F3} deg/s  isActive={isActive}  I={inertiaTensor}");

        if (angSpeed <= angularDeadband) return;

        float I             = inertiaTensor > 0f ? inertiaTensor : 1f;
        float maxDeltaOmega = thrusterForce * momentArm * angularGain * Time.fixedDeltaTime / I;

        if (debugLog)
            Debug.Log($"[RateDamping] maxDeltaOmega={maxDeltaOmega * Mathf.Rad2Deg:F3} deg/s");

        if (angVel.magnitude <= maxDeltaOmega)
            rb.angularVelocity = Vector3.zero;
        else
            rb.angularVelocity = angVel - angVel.normalized * maxDeltaOmega;
    }
}
