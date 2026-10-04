using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Left-edge Utility-mode panel: absorbs the old DockingHUD top-left box (Range/Closing/
/// Lateral/Attitude/Corridor/GNC-phase/docked-status, same green/red threshold coloring
/// against DockingDetector's max* fields) plus absolute Roll/Pitch/Yaw and their rate of
/// change. RDM status moved to UtilityDebugPanel instead of duplicating it here.
/// </summary>
public class UtilityPositionalPanel : MonoBehaviour
{
    const float PanelWidth = 280f;

    static readonly Color VelColor = new Color(1f, 0.88f, 0.55f, 1f);

    static readonly string[] GncPhaseNames = { "IDLE", "CORRECT", "APPROACH", "DOCKED", "HOLD", "MANUAL", "DEPART" };
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

    private Label _rangeVal, _closingVal, _lateralVal, _attitudeVal, _corridorVal, _gncVal, _statusVal;
    private Label _rollVal, _pitchVal, _yawVal;
    private Label _rollRocVal, _pitchRocVal, _yawRocVal;
    private Label _vxVal, _vyVal, _vzVal;

    public void Initialize(VisualElement parent, RelativeNav nav, DockingDetector detector, RateDamping rateDamping,
        ApproachCorridor corridor, VehicleState chaser, UdpCommandReceiver cfsReceiver, float refreshInterval)
    {
        _nav = nav; _detector = detector; _corridor = corridor;
        _chaser = chaser; _cfsReceiver = cfsReceiver; _refreshInterval = refreshInterval;

        // Always-visible panel, no slide-out/collapse -- unlike the debug/thruster panels,
        // this one stays permanently on screen, so there's no SlideOutPanel/tab here.
        //
        // Centered via flexbox (an invisible full-screen "anchor" with justify-content/
        // align-items) rather than computed pixel offsets -- a reactive GeometryChangedEvent
        // measurement of Content's height kept needing rework as rows were added/removed in
        // the UXML, and the same approach broke outright for the thruster panel (see
        // UtilityThrusterPanel.Initialize). Flexbox centering doesn't need to know Body's size
        // at all, so it can't drift out of sync with however many rows the UXML ends up with.
        var anchor = new VisualElement { name = "PositionalAnchor" };
        anchor.style.position = Position.Absolute;
        anchor.style.left = 0; anchor.style.right = 0; anchor.style.top = 0; anchor.style.bottom = 0;
        anchor.style.justifyContent = Justify.Center;  // vertical centering (main axis, column direction)
        anchor.style.alignItems     = Align.FlexStart; // pin to the left edge (cross axis)
        anchor.style.paddingLeft    = 0f;              // gap from the screen's left edge (0 = flush)
        parent.Add(anchor);

        var body = new VisualElement { name = "Body" };
        body.AddToClassList("panel");
        // Override .panel's `position: absolute` -- Body needs to be a normal-flow child of
        // `anchor` for justify-content/align-items to actually center it; an absolutely
        // positioned child is removed from flex layout entirely and ignores both.
        body.style.position = Position.Relative;
        body.style.width = PanelWidth;
        anchor.Add(body);

        var template = Resources.Load<VisualTreeAsset>("UI/UtilityPositionalPanel");
        var content  = template.Instantiate();
        body.Add(content);

        _rangeVal    = content.Q<Label>("RangeValue");
        _closingVal  = content.Q<Label>("ClosingValue");
        _lateralVal  = content.Q<Label>("LateralValue");
        _attitudeVal = content.Q<Label>("AttitudeValue");

        var corridorRow = content.Q<VisualElement>("Corridor");
        var gncRow      = content.Q<VisualElement>("GNC");
        var statusRow   = content.Q<VisualElement>("Status");

        // Guard every lookup with a null check, not just the data-source condition: this UXML
        // is being hand-edited in UI Builder, and a renamed/missing element here previously
        // threw a NullReferenceException that aborted Initialize() before it ever reached the
        // gauge-construction lines below -- silently leaving the gauges empty instead of
        // failing loudly at the row that actually broke.
        if (corridor != null) _corridorVal = content.Q<Label>("CorridorValue");
        else if (corridorRow != null) corridorRow.style.display = DisplayStyle.None;

        if (cfsReceiver != null) _gncVal = content.Q<Label>("GNCValue");
        else if (gncRow != null) gncRow.style.display = DisplayStyle.None;

        if (detector != null) _statusVal = content.Q<Label>("StatusValue");
        else if (statusRow != null) statusRow.style.display = DisplayStyle.None;

        _rollVal  = content.Q<Label>("RollValue");
        _pitchVal = content.Q<Label>("PitchValue");
        _yawVal   = content.Q<Label>("YawValue");

        _rollRocVal  = content.Q<Label>("RollROCValue");
        _pitchRocVal = content.Q<Label>("PitchROCValue");
        _yawRocVal   = content.Q<Label>("YawROCValue");

        _vxVal = content.Q<Label>("VxValue");
        _vyVal = content.Q<Label>("VyValue");
        _vzVal = content.Q<Label>("VzValue");
    }

    void Update()
    {
        if (_nav == null || Time.time < _nextRefresh) return;
        _nextRefresh = Time.time + _refreshInterval;

        float tGap      = _detector != null ? _detector.maxAxialGap      : 0.05f;
        float tClosing  = _detector != null ? _detector.maxClosingSpeed  : 0.30f;
        float tLateral  = _detector != null ? _detector.maxLateralOffset : 0.10f;
        float tAttitude = _detector != null ? _detector.maxAttitudeError : 10f;

        SetRow(_rangeVal,    $"{_nav.range:F2} m",                       _nav.axialGap <= tGap);
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
            _gncVal.style.color = (connected && phase >= 0 && phase < GncPhaseColors.Length)
                ? GncPhaseColors[phase] : new Color(0.50f, 0.50f, 0.50f, 1f);
        }

        if (_detector != null && _statusVal != null)
        {
            _statusVal.text = _detector.isDocked ? "DOCKED" : "APPROACHING";
            _statusVal.style.color = _detector.isDocked ? Color.green : new Color(0.62f, 0.62f, 0.62f, 1f);
        }

        if (_chaser == null) return;

        // Absolute vehicle orientation (0-360, raw) -- NOT nav.pitchError/yawError/rollError,
        // which is docking-port *alignment* error, a different quantity. Vehicle attitude
        // legitimately sweeps the full circle as it tumbles, so these are shown unsigned/raw
        // rather than normalized to +-180.
        Vector3 euler = _chaser.attitude.eulerAngles;
        SetRow(_rollVal,  $"{Mathf.Repeat(euler.z, 360f):F1}°", true, Color.white);
        SetRow(_pitchVal, $"{Mathf.Repeat(euler.x, 360f):F1}°", true, Color.white);
        SetRow(_yawVal,   $"{Mathf.Repeat(euler.y, 360f):F1}°", true, Color.white);

        // Body-frame angular rate (deg/s), signed -- same convention the old RadialGauge's
        // rate readout used.
        Vector3 bodyAngVel = Quaternion.Inverse(_chaser.attitude) * _chaser.angularVelocity;
        float rollRate  = bodyAngVel.z * Mathf.Rad2Deg;
        float pitchRate = bodyAngVel.x * Mathf.Rad2Deg;
        float yawRate   = bodyAngVel.y * Mathf.Rad2Deg;
        SetRow(_rollRocVal,  $"{rollRate:+0.0;-0.0} d/s",  true, Color.white);
        SetRow(_pitchRocVal, $"{pitchRate:+0.0;-0.0} d/s", true, Color.white);
        SetRow(_yawRocVal,   $"{yawRate:+0.0;-0.0} d/s",   true, Color.white);

        Vector3 bodyVel = Quaternion.Inverse(_chaser.attitude) * _chaser.velocity;
        SetRow(_vxVal, $"{bodyVel.x:+0.000;-0.000} m/s", true, VelColor);
        SetRow(_vyVal, $"{bodyVel.y:+0.000;-0.000} m/s", true, VelColor);
        SetRow(_vzVal, $"{bodyVel.z:+0.000;-0.000} m/s", true, VelColor);
    }

    static void SetRow(Label field, string text, bool good, Color? fixedColor = null)
    {
        if (field == null) return;
        field.text = text;
        field.style.color = fixedColor ?? (good ? Color.green : Color.red);
    }
}
