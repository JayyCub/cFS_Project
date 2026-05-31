using UnityEngine;

/// <summary>
/// Injects Clohessy-Wiltshire orbital dynamics into the chaser each FixedUpdate.
/// Models the relative motion a spacecraft in circular orbit experiences due to
/// the curvature of the reference orbit — without this, straight-line thrusting
/// would not produce realistic drift behavior.
///
/// LVLH assumption: world X = radial (away from Earth), Y = along-track (prograde),
/// Z = cross-track (orbit-normal). This holds as long as the target keeps a fixed
/// world-space position (kinematic).
///
/// SCENE VERIFICATION: confirm that the target sits at the world origin and that
/// the chaser starts along the +X axis (radial). If your scene uses a different
/// orientation (e.g. Unity +Y = up ≈ radial), swap the ax/ay/az axis assignments
/// below to match your actual LVLH mapping or the drift forces will be misdirected.
/// </summary>
public class ClohessyWiltshire : MonoBehaviour
{
    [Header("Reference Orbit")]
    [Tooltip("Mean motion n = sqrt(mu/a³) in rad/s. ISS (400 km LEO) ≈ 0.00113")]
    public float meanMotion = 0.00113f;

    [Header("Vehicles")]
    public VehicleState chaser;
    public VehicleState target;

    void FixedUpdate()
    {
        if (chaser == null || target == null) return;

        // Relative state in LVLH (world) frame
        Vector3 r = chaser.position - target.position;
        Vector3 v = chaser.velocity - target.velocity;

        float n  = meanMotion;
        float n2 = n * n;

        // CW equations: differential acceleration of chaser relative to target
        float ax =  3f * n2 * r.x + 2f * n * v.y;   // radial
        float ay = -2f * n * v.x;                      // along-track
        float az = -n2 * r.z;                          // cross-track

        chaser.AddForce(new Vector3(ax, ay, az) * chaser.mass);
    }
}
