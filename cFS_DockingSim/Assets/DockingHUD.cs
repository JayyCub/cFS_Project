using UnityEngine;

/// <summary>
/// Controls legend overlay (top-right, off by default). The main docking/vehicle-state
/// display this used to draw has moved to the Canvas-based DockingUIManager/
/// UtilityPositionalPanel — see those for RANGE/CLOSING/LATERAL/ATTITUDE/CORRIDOR/GNC/attitude
/// gauges. This script now only owns the keybinding reference panel.
/// </summary>
public class DockingHUD : MonoBehaviour
{
    [Tooltip("Show the keybinding legend panel (top-right). Off by default — flip on if you need the reference.")]
    public bool showControlsLegend = false;

    private GUIStyle  _style;  // section headers / legend rows (12 pt, bold, left-aligned)
    private GUIStyle  _small;  // controls legend            (11 pt, bold)
    private Texture2D _bg;

    const float PAD   = 8f;
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
        if (_small == null)
            _small = new GUIStyle(GUI.skin.label) { fontSize = FS_SM, fontStyle = FontStyle.Bold };
    }

    void OnGUI()
    {
        if (!showControlsLegend) return;
        InitStyles();
        DrawControlsLegend();
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
}
