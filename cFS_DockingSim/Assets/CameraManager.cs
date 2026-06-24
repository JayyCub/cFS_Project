using UnityEngine;

/// <summary>
/// Manages switching between multiple scene cameras at runtime.
///
/// Setup:
///   1. Add this component to any persistent GameObject.
///   2. Populate the Cameras array in the Inspector — one entry per camera,
///      each with a display name and a Camera reference.
///   3. The first entry is active on Start; all others are disabled.
///
/// Controls:
///   1 / 2 / 3 / 4  — switch to that camera slot (ignored during backtick test mode)
///
/// A small HUD in the bottom-left corner always shows the camera list.
/// </summary>
public class CameraManager : MonoBehaviour
{
    [System.Serializable]
    public struct CameraEntry
    {
        public string displayName;
        public Camera camera;
    }

    [Tooltip("Ordered list of cameras. Index 0 is active at startup.")]
    public CameraEntry[] cameras = new CameraEntry[0];

    private int     _activeIndex;
    private bool    _stylesReady;
    private GUIStyle _styleActive;
    private GUIStyle _styleInactive;

    static readonly KeyCode[] _switchKeys =
    {
        KeyCode.Alpha1, KeyCode.Alpha2, KeyCode.Alpha3, KeyCode.Alpha4,
    };

    void Start()
    {
        for (int i = 0; i < cameras.Length; i++)
        {
            if (cameras[i].camera != null)
                cameras[i].camera.enabled = (i == 0);
        }
        _activeIndex = 0;
    }

    void Update()
    {
        // Don't steal number keys while backtick thruster-test mode is active.
        if (Input.GetKey(KeyCode.BackQuote)) return;

        for (int i = 0; i < cameras.Length && i < _switchKeys.Length; i++)
        {
            if (Input.GetKeyDown(_switchKeys[i]))
            {
                SwitchTo(i);
                break;
            }
        }
    }

    void SwitchTo(int index)
    {
        if (index < 0 || index >= cameras.Length) return;

        for (int i = 0; i < cameras.Length; i++)
        {
            if (cameras[i].camera == null) continue;
            cameras[i].camera.enabled = (i == index);

            // Notify ChaseCam so it snaps cleanly on next activation.
            if (i != index)
            {
                var chase = cameras[i].camera.GetComponent<ChaseCam>();
                if (chase != null) chase.OnCameraDeactivated();
            }
        }

        _activeIndex = index;
    }

    void OnGUI()
    {
        if (cameras == null || cameras.Length == 0) return;
        EnsureStyles();

        const float lineH  = 22f;
        const float padX   = 8f;
        const float padY   = 6f;
        float       width  = 210f;
        float       height = cameras.Length * lineH + padY * 2f;
        var         panel  = new Rect(10f, Screen.height - height - 10f, width, height);

        // Background
        var prev = GUI.color;
        GUI.color = new Color(0f, 0f, 0f, 0.55f);
        GUI.DrawTexture(panel, Texture2D.whiteTexture);
        GUI.color = prev;

        for (int i = 0; i < cameras.Length; i++)
        {
            bool   active = (i == _activeIndex);
            string label  = $"[{i + 1}]  {cameras[i].displayName}{(active ? "  ◄" : "")}";
            var    r      = new Rect(panel.x + padX, panel.y + padY + i * lineH, width - padX * 2f, lineH);
            GUI.Label(r, label, active ? _styleActive : _styleInactive);
        }
    }

    void EnsureStyles()
    {
        if (_stylesReady) return;

        _styleActive = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 13,
            fontStyle = FontStyle.Bold,
        };
        _styleActive.normal.textColor = Color.yellow;

        _styleInactive = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 13,
            fontStyle = FontStyle.Normal,
        };
        _styleInactive.normal.textColor = new Color(0.82f, 0.82f, 0.82f);

        _stylesReady = true;
    }
}
