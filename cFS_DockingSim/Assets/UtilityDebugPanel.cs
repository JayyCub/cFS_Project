using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Bottom-right Utility-mode "debug" panel: every raw control input/condition currently being
/// given to the craft, Unity-side only (no cFS/UDP protocol changes -- commanded wrench comes
/// from UdpCommandReceiver's cached Last* properties, everything else is already public).
///
/// RCSModel.suppressForces is shown first -- this panel exists specifically because that flag
/// (toggled by the 'T' debug hotkey) can silently kill all thruster force application with zero
/// other on-screen indication, which previously cost a long telemetry-log investigation to
/// diagnose. Surfacing it directly closes that gap.
/// </summary>
public class UtilityDebugPanel : MonoBehaviour
{
    // Matches UtilityPositionalPanel/UtilityThrusterPanel's bordered-box styling (background/
    // border/rounded corners/15px padding). Wider than those two -- 7 values laid out as a
    // 2-column x 4-row grid (last cell empty) needs more horizontal room per column than a
    // single stacked column would.
    const float PanelWidth  = 320f;
    const float PanelHeight = 260f; // header (~27) + 4 grid rows (45px each incl. margin) + box padding (30) + panel padding (20)

    [Tooltip("Show/hide this panel. No on-screen tab any more -- positioning it reliably as a " +
             "child of a sliding element turned out to be persistently fragile, so it's a plain " +
             "key press instead, matching this project's other debug hotkeys (T = suppress, " +
             "H = rate damping).")]
    public KeyCode toggleKey = KeyCode.F3;

    // Matches .row-value-warn intent -- kept in C# since it's driven by a runtime condition,
    // not a static style.
    static readonly Color WarnColor      = new Color(1.00f, 0.35f, 0.30f, 1f);
    static readonly Color NeutralColor   = new Color(0.62f, 0.62f, 0.62f, 1f);

    static readonly string[] GncPhaseNames = { "IDLE", "CORRECT", "APPROACH", "DOCKED", "HOLD", "MANUAL", "DEPART" };

    private RCSModel           _rcs;
    private UdpCommandReceiver _cfsReceiver;
    private RateDamping        _rateDamping;
    private float              _refreshInterval;
    private float              _nextRefresh;
    private SlideOutPanel      _slide;

    private Label _suppressVal, _gncVal, _rdmVal, _cmdFVal, _cmdTVal, _durationVal, _firingVal;

    public void Initialize(VisualElement parent, RCSModel rcsModel, UdpCommandReceiver cfsReceiver,
        RateDamping rateDamping, float refreshInterval)
    {
        _rcs             = rcsModel;
        _cfsReceiver     = cfsReceiver;
        _rateDamping     = rateDamping;
        _refreshInterval = refreshInterval;

        // Starts hidden -- with no visible tab to invite clicking any more, showing it by
        // default with only a keyboard shortcut to hide it would leave no discoverable way to
        // get rid of it. Press F3 (or whatever toggleKey is set to) to bring it up.
        var slide = new SlideOutPanel(parent, SlideEdge.Right, PanelWidth, PanelHeight, startExpanded: false);
        _slide = slide;

        // `right`/`bottom` didn't reliably combine with an explicit size for absolutely
        // positioned elements in this project's Unity version -- the panel rendered pinned to
        // the left edge instead. Compute a plain pixel left/top from the parent's actual
        // resolved size instead. (Body's height is now an explicit constant rather than
        // `auto` -- Yoga's auto-height measurement for an absolutely positioned element didn't
        // reliably use the content's natural size when the container was short, which both
        // squished the rows to fit and, worse, fed a bogus height into this same bottom-anchor
        // math, pushing the panel off-screen entirely.)
        void Reposition(Rect parentRect)
        {
            slide.Body.style.left = parentRect.width  - PanelWidth  - 0f;
            slide.Body.style.top  = parentRect.height - PanelHeight - 12f;
        }
        parent.RegisterCallback<GeometryChangedEvent>(evt => Reposition(evt.newRect));

        var template = Resources.Load<VisualTreeAsset>("UI/UtilityDebugPanel");
        var content  = template.Instantiate();
        slide.Content.Add(content);

        _suppressVal = content.Q<Label>("SuppressValue");
        _gncVal      = content.Q<Label>("GncValue");
        _rdmVal      = content.Q<Label>("RdmValue");
        _cmdFVal     = content.Q<Label>("CmdFValue");
        _cmdTVal     = content.Q<Label>("CmdTValue");
        _durationVal = content.Q<Label>("DurationValue");
        _firingVal   = content.Q<Label>("FiringValue");
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) _slide.Toggle();

        if (Time.time < _nextRefresh) return;
        _nextRefresh = Time.time + _refreshInterval;

        if (_rcs != null && _suppressVal != null)
        {
            _suppressVal.text = _rcs.suppressForces ? "SUPPRESSED" : "normal";
            _suppressVal.style.color = _rcs.suppressForces ? WarnColor : Color.green;
        }

        if (_cfsReceiver != null)
        {
            int  phase     = _cfsReceiver.GncPhase;
            bool connected = _cfsReceiver.CfsActive;

            if (_gncVal != null)
            {
                _gncVal.text = !connected ? "---" : (phase >= 0 && phase < GncPhaseNames.Length ? GncPhaseNames[phase] : "???");
                _gncVal.style.color = connected ? Color.cyan : NeutralColor;
            }

            if (_cmdFVal != null)
            {
                Vector3 f = _cfsReceiver.LastForce;
                _cmdFVal.text = $"{f.x:F0}/{f.y:F0}/{f.z:F0} N";
            }
            if (_cmdTVal != null)
            {
                Vector3 t = _cfsReceiver.LastTorque;
                _cmdTVal.text = $"{t.x:F0}/{t.y:F0}/{t.z:F0} Nm";
            }
            if (_durationVal != null)
                _durationVal.text = $"{_cfsReceiver.LastDuration:F2} s";
        }

        if (_rateDamping != null && _rdmVal != null)
        {
            _rdmVal.text = _rateDamping.isActive ? "ON" : "OFF";
            _rdmVal.style.color = _rateDamping.isActive ? Color.cyan : NeutralColor;
        }

        if (_rcs != null && _firingVal != null)
        {
            int firing = 0;
            for (int i = 4; i < _rcs.ThrusterCount && i < 16; i++)
                if (_rcs.GetThrottle(i) > 0.05f) firing++;
            _firingVal.text = $"{firing}/12";
        }
    }
}
