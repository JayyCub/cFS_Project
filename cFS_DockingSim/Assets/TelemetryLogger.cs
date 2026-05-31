using System;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Writes relative nav state to a CSV file each log interval.
/// A new file is created each Play session with a timestamp in the name.
/// The fields logged here define the UDP telemetry packet format for Phase 4 cFS integration.
/// </summary>
public class TelemetryLogger : MonoBehaviour
{
    public RelativeNav        nav;
    public VehicleState       chaser;
    public ApproachCorridor   corridor;
    public DockingDetector    detector;

    [Tooltip("Seconds between log writes.")]
    public float logInterval = 0.1f;   // 10 Hz — matches typical cFS scheduler rate

    private StreamWriter writer;
    private float        nextLog;
    private float        missionTime;

    void Start()
    {
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string path      = Path.Combine(Application.dataPath, $"../TelemetryLog_{timestamp}.csv");

        writer = new StreamWriter(path, false, Encoding.UTF8);
        WriteHeader();

        Debug.Log($"[TELEMETRY] Logging to: {Path.GetFullPath(path)}");
    }

    void WriteHeader()
    {
        writer.WriteLine(
            "MET_s," +
            "Range_m,ClosingSpeed_ms,LateralOffset_m,AttitudeError_deg," +
            "Pos_X,Pos_Y,Pos_Z," +
            "Vel_X,Vel_Y,Vel_Z," +
            "AngVel_X,AngVel_Y,AngVel_Z," +
            "InCorridor,Docked"
        );
        writer.Flush();
    }

    void FixedUpdate()
    {
        missionTime += Time.fixedDeltaTime;

        if (Time.time < nextLog) return;
        nextLog = Time.time + logInterval;

        if (nav == null || chaser == null) return;

        Vector3 pos    = chaser.position;
        Vector3 vel    = chaser.velocity;
        Vector3 angVel = chaser.angularVelocity;

        bool inCorridor = corridor != null && corridor.inCorridor;
        bool docked     = detector != null && detector.isDocked;

        writer.WriteLine(
            $"{missionTime:F3}," +
            $"{nav.range:F4},{nav.closingSpeed:F4},{nav.lateralOffset:F4},{nav.attitudeError:F4}," +
            $"{pos.x:F4},{pos.y:F4},{pos.z:F4}," +
            $"{vel.x:F4},{vel.y:F4},{vel.z:F4}," +
            $"{angVel.x:F6},{angVel.y:F6},{angVel.z:F6}," +
            $"{(inCorridor ? 1 : 0)},{(docked ? 1 : 0)}"
        );
        writer.Flush();
    }

    void OnDestroy()
    {
        writer?.Close();
    }
}
