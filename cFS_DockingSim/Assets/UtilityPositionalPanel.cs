using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Left-edge Utility-mode panel: absorbs the old DockingHUD top-left box (Range/Closing/
/// Lateral/Attitude/Corridor/GNC-phase/docked-status, same green/red threshold coloring
/// against DockingDetector's max* fields) plus three RadialGauge dials for absolute vehicle
/// attitude. RDM status moved to UtilityDebugPanel instead of duplicating it here.
///
/// IMPORTANT: the three gauges use chaser.attitude/angularVelocity (absolute vehicle
/// orientation), NOT nav.pitchError/yawError/rollError (docking-port *alignment* error — a
/// different quantity). Same euler-normalize / body-frame-rate math as the old
/// DockingHUD.RefreshHudCache.
/// </summary>
[RequireComponent(typeof(RectTransform))]
public class UtilityPositionalPanel : MonoBehaviour
{
    const float PanelWidth   = 280f;
    const float RowHeight    = 20f;
    const float HeaderHeight = 22f;
    const float GaugeSize    = 68f;
    const float Pad          = 10f;
    const float SectionGap   = 14f;

    static readonly Color PanelBg     = new Color(0f, 0f, 0f, 0.62f);
    static readonly Color HeaderColor = new Color(0.62f, 0.82f, 1f, 0.88f);
    static readonly Color LabelColor  = new Color(0.62f, 0.62f, 0.62f, 1f);
    static readonly Color VelColor    = new Color(1f, 0.88f, 0.55f, 1f);

    static readonly string[] GncPhaseNames = { "IDLE", "CORRECT", "APPROACH", "DOCKED", "HOLD" };
    static readonly Color[]  GncPhaseColors =
    {
        new Color(0.50f, 0.50f, 0.50f, 1f),
        new Color(1.00f, 0.85f, 0.20f, 1f),
        new Color(0.20f, 0.90f, 1.00f, 1f),
        Color.green,
        new Color(1.00f, 0.55f, 0.10f, 1f),
    };

    private RelativeNav        _nav;
    private DockingDetector    _detector;
    private ApproachCorridor   _corridor;
    private VehicleState       _chaser;
    private UdpCommandReceiver _cfsReceiver;
    private float              _refreshInterval;
    private float              _nextRefresh;

    private Text _rangeVal, _closingVal, _lateralVal, _attitudeVal, _corridorVal, _gncVal, _statusVal;
    private RadialGauge _rollGauge, _pitchGauge, _yawGauge;
    private Text _vxVal, _vyVal, _vzVal;

    public void Initialize(RelativeNav nav, DockingDetector detector, RateDamping rateDamping,
        ApproachCorridor corridor, VehicleState chaser, UdpCommandReceiver cfsReceiver, float refreshInterval)
    {
        _nav = nav; _detector = detector; _corridor = corridor;
        _chaser = chaser; _cfsReceiver = cfsReceiver; _refreshInterval = refreshInterval;

        int dockingRows = 4 + (corridor != null ? 1 : 0) + (cfsReceiver != null ? 1 : 0) + (detector != null ? 1 : 0);
        float gaugeLabelSpace = RadialGauge.LabelGap + RadialGauge.LabelHeight;
        float height = Pad * 2f + HeaderHeight + dockingRows * RowHeight + SectionGap
                     + HeaderHeight + gaugeLabelSpace + GaugeSize + SectionGap + 3f * RowHeight;

        var size = new Vector2(PanelWidth, height);
        var rt   = (RectTransform)transform;
        // Top-left corner of screen, 12px in from each edge — same origin the old DockingHUD box used.
        UIFactory.SetAnchor(rt, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(12f, -12f), size);

        var slide = gameObject.AddComponent<SlideOutPanel>();
        slide.Initialize(SlideEdge.Left, size, PanelBg);

        BuildContent(slide.Content, dockingRows);
    }

    void BuildContent(RectTransform content, int dockingRows)
    {
        float y = Pad;
        AddHeader(content, ref y, "DOCKING");

        _rangeVal    = AddRow(content, ref y, "RANGE",    "-- m");
        _closingVal  = AddRow(content, ref y, "CLOSING",  "-- m/s");
        _lateralVal  = AddRow(content, ref y, "LATERAL",  "-- m");
        _attitudeVal = AddRow(content, ref y, "ATTITUDE", "-- deg");
        if (_corridor    != null) _corridorVal = AddRow(content, ref y, "CORRIDOR", "--");
        if (_cfsReceiver != null) _gncVal      = AddRow(content, ref y, "GNC",      "---");
        if (_detector    != null) _statusVal   = AddRow(content, ref y, "STATUS",   "APPROACHING");

        y += SectionGap;
        AddHeader(content, ref y, "VEHICLE STATE");
        y += RadialGauge.LabelGap + RadialGauge.LabelHeight; // room for the axis label above each dial

        float gaugeCenterY = y + GaugeSize / 2f;
        _rollGauge  = CreateGauge(content, "ROLL",  new Vector2(Pad + GaugeSize / 2f, -gaugeCenterY));
        _pitchGauge = CreateGauge(content, "PITCH", new Vector2(PanelWidth / 2f, -gaugeCenterY));
        _yawGauge   = CreateGauge(content, "YAW",   new Vector2(PanelWidth - Pad - GaugeSize / 2f, -gaugeCenterY));
        y += GaugeSize + SectionGap;

        _vxVal = AddRow(content, ref y, "Vx", "+0.000 m/s", VelColor);
        _vyVal = AddRow(content, ref y, "Vy", "+0.000 m/s", VelColor);
        _vzVal = AddRow(content, ref y, "Vz", "+0.000 m/s", VelColor);
    }

    Text AddHeader(RectTransform content, ref float y, string text)
    {
        var hdr = UIFactory.CreateText(content, $"{text}_Header", text, 13, HeaderColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(hdr.rectTransform, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0f, 1f),
            new Vector2(Pad, -y), new Vector2(PanelWidth - Pad * 2f, HeaderHeight));
        y += HeaderHeight;
        return hdr;
    }

    Text AddRow(RectTransform content, ref float y, string label, string initialValue, Color? valueColor = null)
    {
        var lbl = UIFactory.CreateText(content, $"{label}_Label", label, 12, LabelColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(lbl.rectTransform, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0f, 1f),
            new Vector2(Pad, -y), new Vector2(90f, RowHeight));

        var val = UIFactory.CreateText(content, $"{label}_Value", initialValue, 12, valueColor ?? Color.white, TextAnchor.MiddleRight);
        UIFactory.SetAnchor(val.rectTransform, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f),
            new Vector2(-Pad, -y), new Vector2(150f, RowHeight));

        y += RowHeight;
        return val;
    }

    RadialGauge CreateGauge(RectTransform content, string label, Vector2 centerOffset)
    {
        var go = new GameObject($"Gauge_{label}", typeof(RectTransform));
        go.transform.SetParent(content, false);
        UIFactory.SetAnchor((RectTransform)go.transform, new Vector2(0f, 1f), new Vector2(0f, 1f),
            new Vector2(0.5f, 0.5f), centerOffset, Vector2.zero);

        var gauge = go.AddComponent<RadialGauge>();
        gauge.Initialize(label, GaugeSize);
        return gauge;
    }

    void Update()
    {
        if (_nav == null || Time.time < _nextRefresh) return;
        _nextRefresh = Time.time + _refreshInterval;

        float tRange    = _detector != null ? _detector.maxRange         : 0.15f;
        float tClosing  = _detector != null ? _detector.maxClosingSpeed  : 0.30f;
        float tLateral  = _detector != null ? _detector.maxLateralOffset : 0.10f;
        float tAttitude = _detector != null ? _detector.maxAttitudeError : 10f;

        SetRow(_rangeVal,    $"{_nav.range:F2} m",                       _nav.range <= tRange);
        SetRow(_closingVal,  $"{_nav.closingSpeed:+0.000;-0.000} m/s",   _nav.closingSpeed > 0f && _nav.closingSpeed <= tClosing);
        SetRow(_lateralVal,  $"{_nav.lateralOffset:F3} m",               _nav.lateralOffset <= tLateral);
        SetRow(_attitudeVal, $"{_nav.attitudeError:F1} deg",             _nav.attitudeError <= tAttitude);

        if (_corridor != null && _corridorVal != null)
            SetRow(_corridorVal, $"{_corridor.corridorAngle:F1}  {(_corridor.inCorridor ? "IN" : "OUT")}", _corridor.inCorridor);

        if (_cfsReceiver != null && _gncVal != null)
        {
            int  phase     = _cfsReceiver.GncPhase;
            bool connected = _cfsReceiver.CfsActive;
            _gncVal.text  = (!connected) ? "---"
                          : (phase >= 0 && phase < GncPhaseNames.Length) ? GncPhaseNames[phase] : "???";
            _gncVal.color = (connected && phase >= 0 && phase < GncPhaseColors.Length)
                ? GncPhaseColors[phase] : new Color(0.50f, 0.50f, 0.50f, 1f);
        }

        if (_detector != null && _statusVal != null)
        {
            _statusVal.text  = _detector.isDocked ? "DOCKED" : "APPROACHING";
            _statusVal.color = _detector.isDocked ? Color.green : new Color(0.62f, 0.62f, 0.62f, 1f);
        }

        if (_chaser == null) return;

        Vector3 euler = _chaser.attitude.eulerAngles;
        float roll  = Normalize(euler.z);
        float pitch = Normalize(euler.x);
        float yaw   = Normalize(euler.y);

        Vector3 bodyAngVel = Quaternion.Inverse(_chaser.attitude) * _chaser.angularVelocity;
        float rollRate  = bodyAngVel.z * Mathf.Rad2Deg;
        float pitchRate = bodyAngVel.x * Mathf.Rad2Deg;
        float yawRate   = bodyAngVel.y * Mathf.Rad2Deg;

        _rollGauge?.SetValue(euler.z, rollRate);
        _pitchGauge?.SetValue(euler.x, pitchRate);
        _yawGauge?.SetValue(euler.y, yawRate);

        Vector3 bodyVel = Quaternion.Inverse(_chaser.attitude) * _chaser.velocity;
        SetRow(_vxVal, $"{bodyVel.x:+0.000;-0.000} m/s", true, VelColor);
        SetRow(_vyVal, $"{bodyVel.y:+0.000;-0.000} m/s", true, VelColor);
        SetRow(_vzVal, $"{bodyVel.z:+0.000;-0.000} m/s", true, VelColor);
    }

    static void SetRow(Text field, string text, bool good, Color? fixedColor = null)
    {
        if (field == null) return;
        field.text  = text;
        field.color = fixedColor ?? (good ? Color.green : Color.red);
    }

    static float Normalize(float angle) => angle > 180f ? angle - 360f : angle;
}
