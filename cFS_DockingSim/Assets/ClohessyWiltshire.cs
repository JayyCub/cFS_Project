using UnityEngine;

/// <summary>
/// Injects Clohessy-Wiltshire orbital dynamics into the chaser each FixedUpdate.
/// Models the relative motion a spacecraft in circular orbit experiences due to
/// the curvature of the reference orbit — without this, straight-line thrusting
/// would not produce realistic drift behavior.
///
/// LVLH mapping (target-centred, Unity world axes, scene unchanged):
///   radial      (away from Earth)        = +Y world
///   along-track (orbital velocity dir)   = −Z world
///   cross-track (orbit normal)           = +X world
/// The chaser starts at −Z of the ISS and closes toward +Z, i.e. it sits AHEAD of the
/// station on +V-bar and approaches the forward port against the direction of flight —
/// the same geometry as a real Dragon approach to IDA-2. (Before 2026-09-26 Z was treated
/// as cross-track, which made the approach run along the orbit normal.)
///
/// CW / Hill equations, x radial, y along-track, z cross-track:
///   ẍ =  3n²x + 2nẏ
///   ÿ = −2nẋ
///   z̈ = −n²z
/// The cross-track sign convention doesn't enter (that axis is decoupled), so the
/// mapping above is valid despite Unity's left-handed axes.
///
/// Must stay consistent with the CW feedforward in gnc_app.c (GNC_APP_ComputeControl).
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

        // Relative state, world axes
        Vector3 r = chaser.position - target.position;
        Vector3 v = chaser.velocity - target.velocity;

        // World → LVLH
        float rRad   =  r.y,  vRad   =  v.y;
        float rCross =  r.x;
        float vAlong = -v.z;

        float n  = meanMotion;
        float n2 = n * n;

        float aRad   =  3f * n2 * rRad + 2f * n * vAlong;
        float aAlong = -2f * n * vRad;
        float aCross = -n2 * rCross;

        // LVLH → world
        Vector3 aWorld = new Vector3(aCross, aRad, -aAlong);
        chaser.AddForce(aWorld * chaser.mass);
    }
}
