using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

/// <summary>
/// Top-level orchestrator for the dual-mode Canvas HUD (Stream vs Utility). This project has
/// no Canvas/EventSystem anywhere today (DockingHUD is legacy IMGUI) — this builds its own
/// hierarchy procedurally at Awake and owns the mode-toggle button plus the data-source
/// references every panel/overlay script needs.
///
/// Attach to the SimulationManager GameObject alongside RelativeNav/DockingDetector/etc. and
/// wire the same references DockingHUD already uses, plus rcsModel.
/// </summary>
public class DockingUIManager : MonoBehaviour
{
    public enum Mode { Stream, Utility }

    [Header("Data Sources")]
    public RelativeNav        nav;
    public DockingDetector    detector;
    public RateDamping        rateDamping;
    public ApproachCorridor   corridor;
    public VehicleState       chaser;
    public UdpCommandReceiver cfsReceiver;
    public RCSModel           rcsModel;

    [Tooltip("Seconds between HUD data reformatting (matches the old DockingHUD's cadence).")]
    public float refreshInterval = 0.1f;

    public Mode CurrentMode { get; private set; } = Mode.Utility;

    static readonly Color AccentBg  = new Color(0.10f, 0.14f, 0.20f, 0.92f);
    static readonly Color AccentTxt = new Color(0.62f, 0.82f, 1f, 1f);

    private Canvas        _canvas;
    private RectTransform _streamRoot;
    private RectTransform _utilityRoot;
    private Text          _modeButtonLabel;

    void Awake()
    {
        BuildCanvas();
        BuildEventSystem();

        _streamRoot = UIFactory.CreateContainer(_canvas.transform, "StreamRoot");
        UIFactory.SetAnchor(_streamRoot, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        _utilityRoot = UIFactory.CreateContainer(_canvas.transform, "UtilityRoot");
        UIFactory.SetAnchor(_utilityRoot, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        BuildModeButton();
        BuildStreamMode();
        BuildUtilityMode();

        ApplyMode();
    }

    void BuildCanvas()
    {
        var go = new GameObject("DockingUICanvas", typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
        go.transform.SetParent(transform, false);

        _canvas               = go.GetComponent<Canvas>();
        _canvas.renderMode    = RenderMode.ScreenSpaceOverlay;
        _canvas.sortingOrder  = 10;

        // Every existing OnGUI script (DockingHUD, CameraManager, ThrusterTestUI) reasons in raw
        // Screen.width/height pixels with no DPI scaling — match that mental model for consistent
        // placement math rather than ScaleWithScreenSize.
        go.GetComponent<CanvasScaler>().uiScaleMode = CanvasScaler.ScaleMode.ConstantPixelSize;
    }

    void BuildEventSystem()
    {
        if (FindFirstObjectByType<EventSystem>() != null) return;

        new GameObject("EventSystem", typeof(EventSystem), typeof(StandaloneInputModule));
        Debug.Log("[DockingUIManager] Created EventSystem (none existed in the scene).");
    }

    void BuildModeButton()
    {
        var btn = UIFactory.CreateButton(_canvas.transform, "ModeToggleButton", "STREAM MODE",
            AccentBg, AccentTxt, out _modeButtonLabel);

        UIFactory.SetAnchor(btn.GetComponent<RectTransform>(),
            new Vector2(0.5f, 1f), new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
            new Vector2(0f, -12f), new Vector2(170f, 30f));

        btn.onClick.AddListener(ToggleMode);
    }

    void BuildStreamMode()
    {
        var go = new GameObject("StreamModeOverlay", typeof(RectTransform));
        go.transform.SetParent(_streamRoot, false);
        UIFactory.SetAnchor(go.GetComponent<RectTransform>(), Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        go.AddComponent<StreamModeOverlay>().Initialize(nav, rcsModel, refreshInterval);
    }

    void BuildUtilityMode()
    {
        // Left: positional data + attitude gauges (absorbs the old DockingHUD top-left box).
        var leftGo = new GameObject("UtilityPositionalPanel", typeof(RectTransform));
        leftGo.transform.SetParent(_utilityRoot, false);
        leftGo.AddComponent<UtilityPositionalPanel>()
              .Initialize(nav, detector, rateDamping, corridor, chaser, cfsReceiver, refreshInterval);

        // Right: detailed thruster-firing diagram, half screen height.
        var rightGo = new GameObject("UtilityThrusterPanel", typeof(RectTransform));
        rightGo.transform.SetParent(_utilityRoot, false);
        rightGo.AddComponent<UtilityThrusterPanel>().Initialize(rcsModel);

        // Bottom-right: raw control-input/condition debug panel.
        var debugGo = new GameObject("UtilityDebugPanel", typeof(RectTransform));
        debugGo.transform.SetParent(_utilityRoot, false);
        debugGo.AddComponent<UtilityDebugPanel>().Initialize(rcsModel, cfsReceiver, rateDamping, refreshInterval);
    }

    public void ToggleMode()
    {
        CurrentMode = CurrentMode == Mode.Stream ? Mode.Utility : Mode.Stream;
        ApplyMode();
    }

    void ApplyMode()
    {
        _streamRoot.gameObject.SetActive(CurrentMode == Mode.Stream);
        _utilityRoot.gameObject.SetActive(CurrentMode == Mode.Utility);

        if (_modeButtonLabel != null)
            _modeButtonLabel.text = CurrentMode == Mode.Stream ? "UTILITY MODE" : "STREAM MODE";
    }
}
