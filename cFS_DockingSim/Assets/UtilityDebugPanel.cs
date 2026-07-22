using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Bottom-right Utility-mode "debug" panel: every raw control input/condition currently being
/// given to the craft, Unity-side only (no cFS/UDP protocol changes — commanded wrench comes
/// from UdpCommandReceiver's cached Last* properties, everything else is already public).
///
/// RCSModel.suppressForces is shown first — this panel exists specifically because that flag
/// (toggled by the 'T' debug hotkey) can silently kill all thruster force application with zero
/// other on-screen indication, which previously cost a long telemetry-log investigation to
/// diagnose. Surfacing it directly closes that gap.
/// </summary>
[RequireComponent(typeof(RectTransform))]
public class UtilityDebugPanel : MonoBehaviour
{
    const float PanelWidth   = 260f;
    const float RowHeight    = 20f;
    const float HeaderHeight = 22f;
    const float Pad          = 10f;
    const int   RowCount     = 7; // suppress, gnc, rdm, cmdF, cmdT, duration, firing

    static readonly Color PanelBg     = new Color(0f, 0f, 0f, 0.62f);
    static readonly Color HeaderColor = new Color(0.62f, 0.82f, 1f, 0.88f);
    static readonly Color LabelColor  = new Color(0.62f, 0.62f, 0.62f, 1f);
    static readonly Color WarnColor   = new Color(1.00f, 0.35f, 0.30f, 1f);

    static readonly string[] GncPhaseNames = { "IDLE", "CORRECT", "APPROACH", "DOCKED", "HOLD" };

    private RCSModel           _rcs;
    private UdpCommandReceiver _cfsReceiver;
    private RateDamping        _rateDamping;
    private float              _refreshInterval;
    private float              _nextRefresh;

    private Text _suppressVal, _gncVal, _rdmVal, _cmdFVal, _cmdTVal, _durationVal, _firingVal;

    public void Initialize(RCSModel rcsModel, UdpCommandReceiver cfsReceiver, RateDamping rateDamping, float refreshInterval)
    {
        _rcs             = rcsModel;
        _cfsReceiver     = cfsReceiver;
        _rateDamping     = rateDamping;
        _refreshInterval = refreshInterval;

        float height = Pad * 2f + HeaderHeight + RowCount * RowHeight;
        var   size   = new Vector2(PanelWidth, height);
        var   rt     = (RectTransform)transform;

        // Bottom-right, docked directly under the right thruster panel.
        UIFactory.SetAnchor(rt, new Vector2(1f, 0f), new Vector2(1f, 0f), new Vector2(1f, 0f), new Vector2(-12f, 12f), size);

        var slide = gameObject.AddComponent<SlideOutPanel>();
        slide.Initialize(SlideEdge.Right, size, PanelBg);

        BuildContent(slide.Content);
    }

    void BuildContent(RectTransform content)
    {
        float y = Pad;
        AddHeader(content, ref y, "DEBUG");

        _suppressVal = AddRow(content, ref y, "SUPPRESS [T]", "--");
        _gncVal      = AddRow(content, ref y, "GNC",          "---");
        _rdmVal      = AddRow(content, ref y, "RDM [H]",      "--");
        _cmdFVal     = AddRow(content, ref y, "CMD F",        "0/0/0 N");
        _cmdTVal     = AddRow(content, ref y, "CMD T",        "0/0/0 Nm");
        _durationVal = AddRow(content, ref y, "DURATION",     "0.00 s");
        _firingVal   = AddRow(content, ref y, "FIRING",       "0/12");
    }

    Text AddHeader(RectTransform content, ref float y, string text)
    {
        var hdr = UIFactory.CreateText(content, $"{text}_Header", text, 13, HeaderColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(hdr.rectTransform, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0f, 1f),
            new Vector2(Pad, -y), new Vector2(PanelWidth - Pad * 2f, HeaderHeight));
        y += HeaderHeight;
        return hdr;
    }

    Text AddRow(RectTransform content, ref float y, string label, string initialValue)
    {
        var lbl = UIFactory.CreateText(content, $"{label}_Label", label, 12, LabelColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(lbl.rectTransform, new Vector2(0f, 1f), new Vector2(0f, 1f), new Vector2(0f, 1f),
            new Vector2(Pad, -y), new Vector2(100f, RowHeight));

        var val = UIFactory.CreateText(content, $"{label}_Value", initialValue, 12, Color.white, TextAnchor.MiddleRight);
        UIFactory.SetAnchor(val.rectTransform, new Vector2(1f, 1f), new Vector2(1f, 1f), new Vector2(1f, 1f),
            new Vector2(-Pad, -y), new Vector2(130f, RowHeight));

        y += RowHeight;
        return val;
    }

    void Update()
    {
        if (Time.time < _nextRefresh) return;
        _nextRefresh = Time.time + _refreshInterval;

        if (_rcs != null && _suppressVal != null)
        {
            _suppressVal.text  = _rcs.suppressForces ? "SUPPRESSED" : "normal";
            _suppressVal.color = _rcs.suppressForces ? WarnColor : Color.green;
        }

        if (_cfsReceiver != null)
        {
            int  phase     = _cfsReceiver.GncPhase;
            bool connected = _cfsReceiver.CfsActive;

            if (_gncVal != null)
            {
                _gncVal.text  = !connected ? "---" : (phase >= 0 && phase < GncPhaseNames.Length ? GncPhaseNames[phase] : "???");
                _gncVal.color = connected ? Color.cyan : LabelColor;
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
            _rdmVal.text  = _rateDamping.isActive ? "ON" : "OFF";
            _rdmVal.color = _rateDamping.isActive ? Color.cyan : LabelColor;
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
