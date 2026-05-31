using UnityEngine;

/// <summary>
/// Discrete RCS thruster control for the chaser.
///
/// In keyboard mode, thrusters fire while keys are held.
/// In cFS mode, SetThrusterCommand() specifies which thrusters to fire and
/// for how long (in seconds). The thrusters cut off automatically when the
/// burn duration elapses — cFS does not need to send a "stop" command.
/// A mask of 0 or duration of 0 is a coast: no force is applied.
/// </summary>
public class RCSModel : MonoBehaviour
{
    public VehicleState vehicle;

    [Header("Thruster Authority")]
    public float thrusterForce = 10f;  // Newtons per thruster — must match GNC_THRUSTER_FORCE in gnc_app.h
    public float momentArm = 1.5f;     // meters from CoM to thruster

    private int   thrusterMask    = 0;
    private bool  externalControl = false; // true while cFS has authority
    private float burnEndTime     = -1f;   // Time.fixedTime when the current burn expires

    void Update()
    {
        // Keyboard input — suppressed while cFS is in control
        if (externalControl) return;

        int mask = 0;
        if (Input.GetKey(KeyCode.D))           mask |= (1 << 0);  // +X right
        if (Input.GetKey(KeyCode.A))           mask |= (1 << 1);  // -X left
        if (Input.GetKey(KeyCode.Space))       mask |= (1 << 2);  // +Y up
        if (Input.GetKey(KeyCode.LeftControl)) mask |= (1 << 3);  // -Y down
        if (Input.GetKey(KeyCode.W))           mask |= (1 << 4);  // +Z forward
        if (Input.GetKey(KeyCode.S))           mask |= (1 << 5);  // -Z back
        if (Input.GetKey(KeyCode.R))           mask |= (1 << 6);  // +pitch
        if (Input.GetKey(KeyCode.F))           mask |= (1 << 7);  // -pitch
        if (Input.GetKey(KeyCode.E))           mask |= (1 << 8);  // +yaw
        if (Input.GetKey(KeyCode.Q))           mask |= (1 << 9);  // -yaw
        if (Input.GetKey(KeyCode.Z))           mask |= (1 << 10); // +roll
        if (Input.GetKey(KeyCode.X))           mask |= (1 << 11); // -roll
        thrusterMask = mask;
    }

    void FixedUpdate()
    {
        // Auto-cut the burn when its commanded duration has elapsed.
        // This is what makes the timed-burn model work: cFS commands a duration
        // and Unity enforces the cutoff without needing a second "stop" packet.
        if (externalControl && thrusterMask != 0 && Time.fixedTime >= burnEndTime)
            thrusterMask = 0;

        if (vehicle == null || thrusterMask == 0) return;

        float f  = thrusterForce;
        float ma = momentArm;

        // Translation
        if ((thrusterMask & (1 << 0)) != 0) vehicle.AddForce( transform.right   * f);
        if ((thrusterMask & (1 << 1)) != 0) vehicle.AddForce(-transform.right   * f);
        if ((thrusterMask & (1 << 2)) != 0) vehicle.AddForce( transform.up      * f);
        if ((thrusterMask & (1 << 3)) != 0) vehicle.AddForce(-transform.up      * f);
        if ((thrusterMask & (1 << 4)) != 0) vehicle.AddForce( transform.forward * f);
        if ((thrusterMask & (1 << 5)) != 0) vehicle.AddForce(-transform.forward * f);

        // Rotation
        if ((thrusterMask & (1 << 6))  != 0) vehicle.AddTorque( transform.right   * f * ma);
        if ((thrusterMask & (1 << 7))  != 0) vehicle.AddTorque(-transform.right   * f * ma);
        if ((thrusterMask & (1 << 8))  != 0) vehicle.AddTorque( transform.up      * f * ma);
        if ((thrusterMask & (1 << 9))  != 0) vehicle.AddTorque(-transform.up      * f * ma);
        if ((thrusterMask & (1 << 10)) != 0) vehicle.AddTorque( transform.forward * f * ma);
        if ((thrusterMask & (1 << 11)) != 0) vehicle.AddTorque(-transform.forward * f * ma);
    }

    /// <summary>
    /// Called by UdpCommandReceiver when a cFS packet arrives.
    /// Fires the given thrusters for exactly <paramref name="duration"/> seconds,
    /// then cuts off. A mask of 0 or duration of 0 is a coast command.
    /// </summary>
    public void SetThrusterCommand(int mask, float duration)
    {
        externalControl = true;
        thrusterMask    = mask;
        burnEndTime     = Time.fixedTime + duration;
    }

    /// <summary>
    /// Called by UdpCommandReceiver when the cFS command timeout fires.
    /// Clears cFS authority so keyboard polling resumes.
    /// </summary>
    public void ClearExternalControl()
    {
        externalControl = false;
        thrusterMask    = 0;
        burnEndTime     = -1f;
    }

    // Read-only access so RateDamping can tell which axes are being commanded
    public int CurrentThrusterMask => thrusterMask;
}
