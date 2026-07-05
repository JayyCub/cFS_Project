using UnityEngine;

/// <summary>
/// Immediate-mode debug panel for manually selecting and firing any combination of RCS thrusters.
///
/// Setup: add this component to any active GameObject in the scene, then drag the RCSModel
/// GameObject into the "Rcs" slot in the Inspector.
///
/// Usage:
///   `             — show/hide the panel
///   Click T00-T15 — toggle thruster selection (green = selected; dark = deselected; dim = disabled in scene)
///   All/None/Inv  — bulk-select helpers
///   Hold FIRE     — fires all selected thrusters for as long as the button is held;
///                   releases RCS control when you let go so cFS can resume
/// </summary>
public class ThrusterTestUI : MonoBehaviour
{
    [Header("References")]
    public RCSModel rcs;

    [Header("Input")]
    [Tooltip("Key to show/hide this panel.")]
    public KeyCode toggleKey = KeyCode.BackQuote;

    // Corner + role label for each thruster, shown as subtitle in the debug panel.
    // T00-T03 are orbital retrograde thrusters — blacked out, not used for docking.
    // T04-T07: approach group (Ap).  T08-T11: brake-yaw (By).  T12-T15: brake-pitch (Bp).
    static readonly string[] _thrusterNames =
    {
        "ORB", "ORB", "ORB", "ORB",   // T00-T03: orbital retrograde (locked out)
        "NE-A","NW-A","SW-A","SE-A",   // T04-T07: approach
        "NE-Y","NW-Y","SW-Y","SE-Y",   // T08-T11: brake-yaw
        "NE-P","NW-P","SW-P","SE-P",   // T12-T15: brake-pitch
    };

    // Thrusters below this index are orbital retrograde — locked in the UI.
    const int OrbitalCount = 4;

    // ── State ─────────────────────────────────────────────────────────────────
    private bool   _visible     = false;
    private bool[] _selected    = new bool[16];
    private bool   _fireHeld    = false;
    private bool   _prevFire    = false;
    private Rect   _windowRect  = new Rect(10, 10, 298, 100); // height auto-expands

    // ── Cached GUI resources (allocated once in EnsureStyles) ─────────────────
    private bool      _stylesReady;
    private GUIStyle  _stOn, _stOff, _stDim, _stBlocked, _stFire, _stFooter;
    private Texture2D _txOn, _txOnH, _txOff, _txOffH, _txDim, _txBlocked, _txFireN, _txFireH;

    // ─────────────────────────────────────────────────────────────────────────

    void Awake()
    {
        if (rcs == null) rcs = GetComponent<RCSModel>();
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) _visible = !_visible;

        // Drive RCSModel: refresh the burn end-time every frame while FIRE is held.
        if (_fireHeld && rcs != null)
        {
            int mask = SelectedMask();
            if (mask != 0)
                rcs.SetThrusterCommand(mask, 0.15f); // 0.15 s > one frame; refreshed each Update
        }
        else if (_prevFire && !_fireHeld && rcs != null)
        {
            rcs.ClearExternalControl(); // hand control back to cFS / keyboard
        }
        _prevFire = _fireHeld;
    }

    void OnGUI()
    {
        if (!_visible) return;
        EnsureStyles();
        _windowRect = GUILayout.Window(0xF11F, _windowRect, DrawPanel, "Thruster Test  [`]");
    }

    void DrawPanel(int id)
    {
        Event e = Event.current;

        GUILayout.Space(2);

        // ── 4 × 4 thruster grid (row-major: T00-T03 top row, T12-T15 bottom) ─
        for (int row = 0; row < 4; row++)
        {
            GUILayout.BeginHorizontal();
            for (int col = 0; col < 4; col++)
            {
                int idx = row * 4 + col;

                bool isOrbital   = idx < OrbitalCount;
                bool sceneActive = !isOrbital && rcs != null && rcs.IsThrusterActive(idx);
                bool nowFiring   = !isOrbital && rcs != null && (rcs.CurrentThrusterMask & (1 << idx)) != 0;
                bool selected    = _selected[idx];

                GUIStyle st = isOrbital  ? _stBlocked :
                              !sceneActive ? _stDim    :
                               selected    ? _stOn     : _stOff;

                string name  = idx < _thrusterNames.Length ? _thrusterNames[idx] : "";
                string label = isOrbital  ? $"T{idx:D2}\n({name})" :
                               nowFiring  ? $"T{idx:D2} ●\n({name})" : $"T{idx:D2}\n({name})";

                if (GUILayout.Button(label, st, GUILayout.Width(66), GUILayout.Height(42)) && sceneActive)
                    _selected[idx] = !_selected[idx];
            }
            GUILayout.EndHorizontal();
        }

        GUILayout.Space(6);

        // ── Bulk-select row ───────────────────────────────────────────────────
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("All", GUILayout.Height(24)))
            for (int i = OrbitalCount; i < 16; i++)
                _selected[i] = rcs == null || rcs.IsThrusterActive(i);

        if (GUILayout.Button("None", GUILayout.Height(24)))
            System.Array.Clear(_selected, 0, 16);

        if (GUILayout.Button("Invert", GUILayout.Height(24)))
            for (int i = OrbitalCount; i < 16; i++)
                if (rcs == null || rcs.IsThrusterActive(i))
                    _selected[i] = !_selected[i];
        GUILayout.EndHorizontal();

        GUILayout.Space(4);

        // ── FIRE button — hold to keep thrusters on ───────────────────────────
        // We track mouse-down / mouse-up rather than using Button() so the
        // thrusters stay on for every frame between those two events.
        Rect fireRect = GUILayoutUtility.GetRect(
            new GUIContent("FIRE"), _stFire,
            GUILayout.Height(50), GUILayout.ExpandWidth(true));

        if (e.type == EventType.MouseDown && fireRect.Contains(e.mousePosition))
        {
            _fireHeld = true;
            e.Use();
        }
        if (e.type == EventType.MouseUp)
            _fireHeld = false;

        // Swap background colour (red = firing, blue = idle)
        _stFire.normal.background = _fireHeld ? _txFireH : _txFireN;
        GUI.Label(fireRect, _fireHeld ? "■  FIRING  ■" : "FIRE  (hold)", _stFire);

        // ── Status line ───────────────────────────────────────────────────────
        int selCount = Popcount(SelectedMask());
        GUILayout.Label(
            $"Selected: {selCount}  |  Mask: 0x{SelectedMask():X4}",
            _stFooter, GUILayout.Height(20));

        // Only the title bar (top 20 px) is draggable so it doesn't fight the FIRE button.
        GUI.DragWindow(new Rect(0, 0, 9999, 20));
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    int SelectedMask()
    {
        int m = 0;
        for (int i = OrbitalCount; i < 16; i++)
            if (_selected[i]) m |= (1 << i);
        return m;
    }

    static int Popcount(int v)
    {
        int c = 0;
        while (v != 0) { c += v & 1; v >>= 1; }
        return c;
    }

    // ── Style / texture initialisation ────────────────────────────────────────

    void EnsureStyles()
    {
        if (_stylesReady) return;

        _txOn    = Tex(new Color(0.18f, 0.75f, 0.22f));
        _txOnH   = Tex(new Color(0.28f, 0.92f, 0.32f));
        _txOff   = Tex(new Color(0.20f, 0.20f, 0.28f));
        _txOffH  = Tex(new Color(0.30f, 0.30f, 0.42f));
        _txDim     = Tex(new Color(0.10f, 0.10f, 0.10f, 0.55f));
        _txBlocked = Tex(new Color(0.06f, 0.06f, 0.06f, 1.00f));
        _txFireN   = Tex(new Color(0.15f, 0.45f, 0.85f));
        _txFireH   = Tex(new Color(0.92f, 0.20f, 0.12f));

        _stOn      = MakeBtn(_txOn,      _txOnH,      Color.white,                       FontStyle.Bold);
        _stOff     = MakeBtn(_txOff,     _txOffH,     Color.white,                       FontStyle.Normal);
        _stDim     = MakeBtn(_txDim,     _txDim,      new Color(0.35f, 0.35f, 0.35f),    FontStyle.Normal);
        _stBlocked = MakeBtn(_txBlocked, _txBlocked,  new Color(0.22f, 0.22f, 0.22f),    FontStyle.Normal);

        _stFire = new GUIStyle(GUI.skin.button)
        {
            fontSize  = 20,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
        };
        _stFire.normal.textColor = Color.white;
        _stFire.hover.textColor  = Color.white;

        _stFooter = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 11,
            alignment = TextAnchor.MiddleCenter,
        };
        _stFooter.normal.textColor = new Color(0.6f, 0.6f, 0.6f);

        _stylesReady = true;
    }

    static GUIStyle MakeBtn(Texture2D norm, Texture2D hov, Color text, FontStyle fs)
    {
        var s = new GUIStyle(GUI.skin.button) { fontStyle = fs, fontSize = 11 };
        s.normal.background  = norm;
        s.hover.background   = hov;
        s.active.background  = hov;
        s.normal.textColor   = text;
        s.hover.textColor    = text;
        s.active.textColor   = text;
        return s;
    }

    static Texture2D Tex(Color c)
    {
        var t = new Texture2D(1, 1);
        t.SetPixel(0, 0, c);
        t.Apply();
        return t;
    }

    void OnDestroy()
    {
        if (_fireHeld && rcs != null) rcs.ClearExternalControl();

        // Release dynamic textures
        Texture2D[] all = { _txOn, _txOnH, _txOff, _txOffH, _txDim, _txBlocked, _txFireN, _txFireH };
        foreach (var t in all) if (t != null) Destroy(t);
    }
}
