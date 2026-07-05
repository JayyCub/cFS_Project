using UnityEngine;

/// <summary>
/// On-screen docking approach display. Green = within limits, red = out of limits.
/// Uses Unity's immediate-mode GUI — no external UI packages required.
/// </summary>
public class DockingHUD : MonoBehaviour
{
    public RelativeNav        nav;
    public DockingDetector    detector;
    public RateDamping        rateDamping;
    public ApproachCorridor   corridor;
    public VehicleState       chaser;
    public UdpCommandReceiver cfsReceiver;

    private GUIStyle  _style;       // main data rows  (12 pt, bold, left-aligned)
    private GUIStyle  _styleRight;  // value column    (12 pt, bold, right-aligned)
    private GUIStyle  _small;       // controls legend (11 pt, bold)
    private Texture2D _bg;

    const float PAD   = 8f;
    const float LINE  = 17f;
    const float HLINE = 20f;
    const int   FS    = 12;
    const int   FS_SM = 11;

    void OnDestroy() { if (_bg != null) Destroy(_bg); }

    void InitStyles()
    {
        if (_bg == null)
        {
            _bg = new Texture2D(1, 1);
            _bg.SetPixel(0, 0, new Color(0f, 0f, 0f, 0.52f));
            _bg.Apply();
        }
        if (_style == null)
            _style = new GUIStyle(GUI.skin.label) { fontSize = FS, fontStyle = FontStyle.Bold };
        if (_styleRight == null)
            _styleRight = new GUIStyle(GUI.skin.label)
            {
                fontSize  = FS,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleRight
            };
        if (_small == null)
            _small = new GUIStyle(GUI.skin.label) { fontSize = FS_SM, fontStyle = FontStyle.Bold };
    }

    void OnGUI()
    {
        if (nav == null) return;
        InitStyles();
        DrawDockingPanel();
        DrawControlsLegend();
        DrawStatePanel();
    }

    // ── shared helpers ────────────────────────────────────────────────────────

    void BgBox(float x, float y, float w, float h)
        => GUI.DrawTexture(new Rect(x, y, w, h), _bg);

    void SectionHeader(float px, ref float y, float pw, string text)
    {
        _style.normal.textColor = new Color(0.62f, 0.82f, 1f, 0.88f);
        GUI.Label(new Rect(px + PAD, y, pw - PAD * 2, HLINE), text, _style);
        y += HLINE;
    }

    // Two-column row: left-aligned label, right-aligned colored value.
    void DataRow(float px, ref float y, float lblW, float valW,
                 string label, string value, Color valueColor)
    {
        _style.normal.textColor = new Color(0.62f, 0.62f, 0.62f, 1f);
        GUI.Label(new Rect(px + PAD, y, lblW, LINE), label, _style);
        _styleRight.normal.textColor = valueColor;
        GUI.Label(new Rect(px + PAD + lblW, y, valW - PAD, LINE), value, _styleRight);
        y += LINE;
    }

    // ── docking metrics panel (top-left) ──────────────────────────────────────

    static readonly string[] GncPhaseNames = { "IDLE", "CORRECT", "APPROACH", "DOCKED", "HOLD" };
    static readonly Color[]  GncPhaseColors =
    {
        new Color(0.50f, 0.50f, 0.50f, 1f),  // IDLE    — gray
        new Color(1.00f, 0.85f, 0.20f, 1f),  // CORRECT — yellow
        new Color(0.20f, 0.90f, 1.00f, 1f),  // APPROACH— cyan
        Color.green,                           // DOCKED  — green
        new Color(1.00f, 0.55f, 0.10f, 1f),  // HOLD    — orange
    };

    void DrawDockingPanel()
    {
        const float LBL = 72f, VAL = 108f;
        float pw = LBL + VAL + PAD * 2;

        float tRange    = detector != null ? detector.maxRange         : 0.15f;
        float tClosing  = detector != null ? detector.maxClosingSpeed  : 0.30f;
        float tLateral  = detector != null ? detector.maxLateralOffset : 0.10f;
        float tAttitude = detector != null ? detector.maxAttitudeError : 10f;
        const float tAttAxis = 5f;  // per-axis green threshold (tighter than scalar)

        bool showCorridor = corridor     != null;
        bool showRdm      = rateDamping  != null;
        bool showPhase    = cfsReceiver  != null;
        bool showDocked   = detector     != null && detector.isDocked;

        int rows = 7 + (showCorridor ? 1 : 0) + (showRdm ? 1 : 0) + (showPhase ? 1 : 0);
        float ph = PAD + HLINE + rows * LINE + (showDocked ? LINE + 3f : 0f) + PAD;

        float px = 12f, py = 12f;
        BgBox(px, py, pw, ph);

        float y = py + PAD;
        SectionHeader(px, ref y, pw, "DOCKING");

        DataRow(px, ref y, LBL, VAL, "RANGE",
            $"{nav.range:F2} m",
            nav.range <= tRange ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "CLOSING",
            $"{nav.closingSpeed:+0.000;-0.000} m/s",
            nav.closingSpeed > 0f && nav.closingSpeed <= tClosing ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "LATERAL",
            $"{nav.lateralOffset:F3} m",
            nav.lateralOffset <= tLateral ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "ATTITUDE",
            $"{nav.attitudeError:F1} deg",
            nav.attitudeError <= tAttitude ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "  PITCH",
            $"{nav.pitchError:+0.0;-0.0} deg",
            Mathf.Abs(nav.pitchError) <= tAttAxis ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "  YAW",
            $"{nav.yawError:+0.0;-0.0} deg",
            Mathf.Abs(nav.yawError) <= tAttAxis ? Color.green : Color.red);

        DataRow(px, ref y, LBL, VAL, "  ROLL",
            $"{nav.rollError:+0.0;-0.0} deg",
            Mathf.Abs(nav.rollError) <= tAttAxis ? Color.green : Color.red);

        if (showCorridor)
            DataRow(px, ref y, LBL, VAL, "CORRIDOR",
                $"{corridor.corridorAngle:F1}  {(corridor.inCorridor ? "IN" : "OUT")}",
                corridor.inCorridor ? Color.green : Color.red);

        if (showRdm)
        {
            bool rdmSuppressed = cfsReceiver != null && cfsReceiver.CfsActive;
            string rdmLabel    = rdmSuppressed ? "SUPPRESSED" : (rateDamping.isActive ? "ON  [H]" : "OFF [H]");
            Color  rdmColor    = rdmSuppressed
                ? new Color(0.50f, 0.50f, 0.50f, 1f)
                : (rateDamping.isActive ? Color.cyan : new Color(0.50f, 0.50f, 0.50f, 1f));
            DataRow(px, ref y, LBL, VAL, "RDM", rdmLabel, rdmColor);
        }

        if (showPhase)
        {
            int    phase      = cfsReceiver.GncPhase;
            bool   connected  = cfsReceiver.CfsActive;
            string phaseName  = (!connected)                                ? "---"
                              : (phase >= 0 && phase < GncPhaseNames.Length) ? GncPhaseNames[phase]
                              : "???";
            Color  phaseColor = (connected && phase >= 0 && phase < GncPhaseColors.Length)
                              ? GncPhaseColors[phase]
                              : new Color(0.50f, 0.50f, 0.50f, 1f);
            DataRow(px, ref y, LBL, VAL, "GNC", phaseName, phaseColor);
        }

        if (showDocked)
        {
            y += 3f;
            _style.normal.textColor = Color.green;
            _style.alignment = TextAnchor.MiddleCenter;
            GUI.Label(new Rect(px + PAD, y, pw - PAD * 2, LINE), "DOCKED", _style);
            _style.alignment = TextAnchor.UpperLeft;
        }
    }

    // ── controls legend (top-right) ───────────────────────────────────────────

    void DrawControlsLegend()
    {
        string[] keys = { "W / S", "A / D", "Space", "Ctrl", "R / F", "E / Q", "Z / X", "H", "`", "T", "Bksp", "Arrows", "Enter" };
        string[] acts = { "Fwd / Back", "Left / Right", "Up", "Down", "Pitch", "Yaw", "Roll", "Rate Damp", "RCS Panel", "No Forces", "Reset", "Look", "Center Cam" };

        const float LBL = 44f, ACT = 110f;
        const float LH  = 16f;
        float pw = LBL + ACT + PAD * 2;
        float ph = PAD + HLINE + keys.Length * LH + PAD;
        float px = Screen.width - pw - 12f;
        float py = 12f;

        BgBox(px, py, pw, ph);

        float y = py + PAD;
        SectionHeader(px, ref y, pw, "CONTROLS");

        for (int i = 0; i < keys.Length; i++)
        {
            _small.normal.textColor = new Color(0.62f, 0.62f, 0.62f, 1f);
            GUI.Label(new Rect(px + PAD, y, LBL, LH), keys[i], _small);
            _small.normal.textColor = new Color(0.88f, 0.88f, 0.88f, 0.85f);
            GUI.Label(new Rect(px + PAD + LBL, y, ACT, LH), acts[i], _small);
            y += LH;
        }
    }

    // ── vehicle state panel (bottom-left) ─────────────────────────────────────

    void DrawStatePanel()
    {
        if (chaser == null) return;

        Vector3 euler = chaser.attitude.eulerAngles;
        float roll    = Normalize(euler.z);
        float pitch   = Normalize(euler.x);
        float yaw     = Normalize(euler.y);

        Vector3 bodyAngVel = Quaternion.Inverse(chaser.attitude) * chaser.angularVelocity;
        float rollRate  = bodyAngVel.z * Mathf.Rad2Deg;
        float pitchRate = bodyAngVel.x * Mathf.Rad2Deg;
        float yawRate   = bodyAngVel.y * Mathf.Rad2Deg;

        Vector3 bodyVel = Quaternion.Inverse(chaser.attitude) * chaser.velocity;

        const float LBL = 46f, ANG = 74f, RATE = 84f;
        float pw = LBL + ANG + RATE + PAD * 2;
        float ph = PAD + HLINE + 3 * LINE + 4f + 3 * LINE + PAD;
        float px = 12f;
        float py = Screen.height - ph - 12f;

        BgBox(px, py, pw, ph);

        float y = py + PAD;
        SectionHeader(px, ref y, pw, "VEHICLE STATE");

        Color attColor = new Color(0.78f, 0.78f, 1.00f, 1f);
        AttRow(px, ref y, LBL, ANG, RATE, "ROLL",  roll,  rollRate,  attColor);
        AttRow(px, ref y, LBL, ANG, RATE, "PITCH", pitch, pitchRate, attColor);
        AttRow(px, ref y, LBL, ANG, RATE, "YAW",   yaw,   yawRate,   attColor);

        y += 4f;

        Color velColor = new Color(1f, 0.88f, 0.55f, 1f);
        VelRow(px, ref y, LBL, ANG + RATE, "Vx", bodyVel.x, velColor);
        VelRow(px, ref y, LBL, ANG + RATE, "Vy", bodyVel.y, velColor);
        VelRow(px, ref y, LBL, ANG + RATE, "Vz", bodyVel.z, velColor);
    }

    void AttRow(float px, ref float y, float lblW, float angW, float rateW,
                string label, float angle, float rate, Color color)
    {
        _style.normal.textColor = new Color(0.62f, 0.62f, 0.62f, 1f);
        GUI.Label(new Rect(px + PAD, y, lblW, LINE), label, _style);
        _styleRight.normal.textColor = color;
        GUI.Label(new Rect(px + PAD + lblW,        y, angW  - 2f, LINE), $"{angle:+0.0;-0.0} deg", _styleRight);
        GUI.Label(new Rect(px + PAD + lblW + angW, y, rateW - PAD, LINE), $"{rate:+0.0;-0.0} d/s",  _styleRight);
        y += LINE;
    }

    void VelRow(float px, ref float y, float lblW, float valW,
                string label, float vel, Color color)
    {
        _style.normal.textColor = new Color(0.62f, 0.62f, 0.62f, 1f);
        GUI.Label(new Rect(px + PAD, y, lblW, LINE), label, _style);
        _styleRight.normal.textColor = color;
        GUI.Label(new Rect(px + PAD + lblW, y, valW - PAD, LINE), $"{vel:+0.000;-0.000} m/s", _styleRight);
        y += LINE;
    }

    static float Normalize(float angle) => angle > 180f ? angle - 360f : angle;
}
