using System;
using UnityEngine;

/// <summary>
/// The chaser's navigation sensors — the "hardware" whose raw readings go to cFS in each
/// SimLink SIM_STATE frame. Flight software's NAV app turns them into a navigation solution;
/// nothing computed from truth reaches flight software any more (realism phase 4).
///
///   IMU            gyro body rate, and the non-gravitational Δv accumulated over the cycle
///                  (thrust and contact — an accelerometer in free fall reads zero, so the
///                  Clohessy-Wiltshire gravity ClohessyWiltshire.cs applies is taken back out)
///   star tracker   body attitude quaternion in LVLH (= world in this sim)
///   LIDAR          range / azimuth / elevation from the mount to the target port reflector,
///                  only inside its field of view and range limits
///   capture        soft-capture latch switch (DockingDetector)
///
/// Noise is Gaussian from a seeded generator, so lock-stepped runs stay repeatable. The noise
/// numbers are the sensor spec sheet; cFS nav_cfg_tbl.c holds FSW's model of the same sensors.
/// The "Fault injection" switches let NAV's degraded modes be tested from the Inspector.
///
/// Execution order: after RelativeNav / DockingDetector, before UdpTelemetrySender samples it,
/// and before ClohessyWiltshire, so ClohessyWiltshire.LastAccel is still the previous step's.
/// </summary>
[DefaultExecutionOrder(-150)]
public class ChaserSensors : MonoBehaviour
{
    [Header("Vehicle (wired by UdpTelemetrySender if left empty)")]
    public VehicleState      chaser;
    public Transform         chaserPort;
    public Transform         targetPort;
    public ClohessyWiltshire gravity;
    public DockingDetector   detector;
    [Tooltip("Truth navigation — only used by the NAV table generator, for the docking roll index.")]
    public RelativeNav       relativeNav;
    [Tooltip("LIDAR mount (boresight +Z). Empty = the chaser docking port.")]
    public Transform         rpsMount;

    [Header("IMU (at the centre of mass)")]
    [Tooltip("Gyro noise per sample, 1-sigma (rad/s). 1e-5 ≈ a navigation-grade FOG.")]
    public float   gyroNoise_rads   = 1e-5f;
    public Vector3 gyroBias_rads    = Vector3.zero;
    [Tooltip("Accelerometer Δv noise per GNC cycle, 1-sigma (m/s).")]
    public float   accelDvNoise_ms  = 1e-5f;
    public Vector3 accelBias_mss    = Vector3.zero;

    [Header("Star tracker")]
    [Tooltip("Attitude noise per axis, 1-sigma (rad). 5e-5 ≈ 10 arcsec.")]
    public float starTrackerNoise_rad = 5e-5f;

    [Header("Relative-pose LIDAR")]
    public float rpsRangeNoise_m     = 0.005f;
    [Tooltip("Additional range noise as a fraction of range.")]
    public float rpsRangeNoiseFrac   = 0.001f;
    public float rpsAngleNoise_rad   = 0.0005f;
    public float rpsMinRange_m       = 0.3f;
    public float rpsMaxRange_m       = 250f;
    [Tooltip("Half-angle of the field of view about the boresight (deg).")]
    public float rpsHalfFov_deg      = 35f;

    [Header("Fault injection")]
    public bool imuFailed;
    public bool starTrackerFailed;
    public bool rpsFailed;

    [Header("Determinism")]
    public int seed = 1;

    System.Random _rng;
    Vector3       _dvBody;      // non-gravitational Δv since the last sample, body frame
    Vector3       _prevVel;
    bool          _havePrevVel;

    Transform Mount => rpsMount != null ? rpsMount : chaserPort;

    void Awake() => _rng = new System.Random(seed);

    void FixedUpdate()
    {
        if (chaser == null) return;

        // Velocity now reflects the previous physics step; take out the CW gravity that step
        // applied (an accelerometer can't feel it) and keep what thrust and contact did.
        Vector3 v = chaser.velocity;
        if (_havePrevVel)
        {
            Vector3 dvGrav = gravity != null ? gravity.LastAccel * Time.fixedDeltaTime : Vector3.zero;
            _dvBody += chaser.transform.InverseTransformDirection(v - _prevVel - dvGrav);
        }
        _prevVel     = v;
        _havePrevVel = true;
    }

    /// <summary>
    /// Forget the Δv accumulated so far. Called on a scenario reset, so the teleport and
    /// velocity zeroing aren't measured as a huge acceleration.
    /// </summary>
    public void ResetSensors()
    {
        _dvBody      = Vector3.zero;
        _havePrevVel = false;
    }

    /// <summary>Fill the sensor fields of a SIM_STATE at the cycle boundary.</summary>
    public void Sample(ref SimLinkProtocol.SimState s, float cycleDt)
    {
        uint valid = 0;

        if (!imuFailed)
        {
            Vector3 rate = chaser.transform.InverseTransformDirection(chaser.angularVelocity);
            s.GyroRate_B = rate + gyroBias_rads + Noise3(gyroNoise_rads);
            s.DeltaV_B   = _dvBody + accelBias_mss * cycleDt + Noise3(accelDvNoise_ms);
            valid |= SimLinkProtocol.SensorImu;
        }
        _dvBody = Vector3.zero;

        if (!starTrackerFailed)
        {
            s.StQuat = chaser.transform.rotation * SmallRotation(Noise3(starTrackerNoise_rad));
            valid |= SimLinkProtocol.SensorSt;
        }

        Transform mount = Mount;
        if (!rpsFailed && mount != null && targetPort != null)
        {
            Vector3 p     = mount.InverseTransformPoint(targetPort.position); // reflector, sensor frame
            float   range = p.magnitude;
            if (range >= rpsMinRange_m && range <= rpsMaxRange_m &&
                Vector3.Angle(Vector3.forward, p) <= rpsHalfFov_deg)
            {
                s.RpsRange_m = range + Gauss() * (rpsRangeNoise_m + rpsRangeNoiseFrac * range);
                s.RpsAz_rad  = Mathf.Atan2(p.x, p.z) + Gauss() * rpsAngleNoise_rad;
                s.RpsEl_rad  = Mathf.Asin(p.y / range) + Gauss() * rpsAngleNoise_rad;
                valid |= SimLinkProtocol.SensorRps;
            }
        }

        s.MechFlags   = (detector != null && detector.isDocked) ? SimLinkProtocol.MechCapture : 0u;
        s.SensorValid = valid;
    }

    float Gauss()
    {
        // Box-Muller
        double u1 = 1.0 - _rng.NextDouble();
        double u2 = _rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
    }

    Vector3 Noise3(float sigma) => sigma > 0f ? new Vector3(Gauss(), Gauss(), Gauss()) * sigma : Vector3.zero;

    static Quaternion SmallRotation(Vector3 rotVec)
    {
        float angle = rotVec.magnitude;
        return angle > 0f ? Quaternion.AngleAxis(angle * Mathf.Rad2Deg, rotVec / angle) : Quaternion.identity;
    }

    // ── cFS NAV table generator ────────────────────────────────────────────

    /// <summary>
    /// Prints this scene's geometry as the C initializer rows for cFS
    /// apps/nav/fsw/tables/nav_cfg_tbl.c. Run in Play mode (the CoM override is applied in
    /// VehicleState.Start) after moving a docking port, the LIDAR mount or the CoM, so flight
    /// software's table matches the hardware again.
    /// </summary>
    [ContextMenu("Log cFS NAV table (nav_cfg_tbl.c)")]
    void LogCfsNavTable()
    {
        var rb = chaser != null ? chaser.GetComponent<Rigidbody>() : null;
        VehicleState target = relativeNav != null ? relativeNav.target : null;
        if (rb == null || chaserPort == null || targetPort == null || target == null || Mount == null)
        {
            Debug.LogWarning("[ChaserSensors] NAV table: chaser Rigidbody, ports, LIDAR mount and RelativeNav.target must all be set");
            return;
        }

        Transform  body = chaser.transform;
        Vector3    com  = rb.worldCenterOfMass;
        Quaternion qB   = body.rotation;
        Transform  iss  = target.transform;

        // Docking attitude of the chaser port, the same definition RelativeNav uses: facing
        // the target port, ups aligned, then the rollErrorOffset roll index.
        Quaternion dockedPort = Quaternion.LookRotation(-targetPort.forward, targetPort.up) *
                                Quaternion.Euler(0f, 0f, -relativeNav.rollErrorOffset);

        var sb = new System.Text.StringBuilder();
        sb.AppendLine("[ChaserSensors] cFS nav_cfg_tbl.c geometry rows:");
        sb.AppendLine($"    .ChaserPortPos_B  = {V(body.InverseTransformDirection(chaserPort.position - com))},");
        sb.AppendLine($"    .ChaserPortQuat_B = {Q(Quaternion.Inverse(qB) * chaserPort.rotation)},");
        sb.AppendLine($"    .RpsPos_B         = {V(body.InverseTransformDirection(Mount.position - com))},");
        sb.AppendLine($"    .RpsQuat_B        = {Q(Quaternion.Inverse(qB) * Mount.rotation)},");
        sb.AppendLine($"    .TargetAttQuat_L  = {Q(iss.rotation)},");
        sb.AppendLine($"    .TargetPortPos_T  = {V(iss.InverseTransformDirection(targetPort.position - iss.position))},");
        sb.AppendLine($"    .TargetPortQuat_T = {Q(Quaternion.Inverse(iss.rotation) * targetPort.rotation)},");
        sb.AppendLine($"    .DockedQuat_TP    = {Q(Quaternion.Inverse(targetPort.rotation) * dockedPort)},");
        Debug.Log(sb.ToString());
    }

    static string F(float f) => f.ToString("0.0#######", System.Globalization.CultureInfo.InvariantCulture) + "f";
    static string V(Vector3 v) => $"{{{F(v.x)}, {F(v.y)}, {F(v.z)}}}";
    static string Q(Quaternion q) => $"{{{F(q.x)}, {F(q.y)}, {F(q.z)}, {F(q.w)}}}";
}
