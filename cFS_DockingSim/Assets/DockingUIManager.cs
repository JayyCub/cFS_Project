using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.EventSystems;

/// <summary>
/// Top-level orchestrator for the UI Toolkit HUD. Builds a UIDocument at Awake --
/// PanelSettings is created at runtime rather than as an on-disk asset, and the layout/style
/// come from Resources.Load'd UXML/USS (Assets/Resources/UI/), so no manual Editor wiring
/// (dragging assets into Inspector slots) is needed -- and owns the data-source references
/// every panel script needs.
///
/// Attach to the SimulationManager GameObject alongside RelativeNav/DockingDetector/etc. and
/// wire the same references this always has, plus rcsModel.
/// </summary>
public class DockingUIManager : MonoBehaviour
{
    /// <summary>
    /// Window height, in UI units, that the HUD is laid out against. Lower = bigger HUD.
    /// The tallest panel is about 705 units, so this has to stay above that for it to fit.
    /// </summary>
    public const float ReferenceHeight = 1050f;

    /// <summary>Screen pixels per UI unit, for IMGUI overlays that need to match the panels.</summary>
    public static float HudScale => Screen.height / ReferenceHeight;

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

    private UIDocument    _document;
    private VisualElement _utilityRoot;

    void Awake()
    {
        BuildUIDocument();
        BuildEventSystem();

        _utilityRoot = _document.rootVisualElement.Q<VisualElement>("UtilityRoot");

        BuildPanels();
    }

    void BuildUIDocument()
    {
        var settings = ScriptableObject.CreateInstance<PanelSettings>();
        // Scale with the window rather than using constant pixels: on a Retina display one
        // UI pixel would otherwise be one physical pixel, which renders the HUD at half size.
        // Matching height keeps the tall positional panel on screen at any window size.
        settings.scaleMode      = PanelScaleMode.ScaleWithScreenSize;
        settings.screenMatchMode = PanelScreenMatchMode.MatchWidthOrHeight;
        settings.match           = 1f;
        settings.referenceResolution = new Vector2Int(
            Mathf.RoundToInt(ReferenceHeight * 16f / 9f), Mathf.RoundToInt(ReferenceHeight));
        settings.sortingOrder   = 10;
        // Without a theme, newer Unity versions refuse to render the panel at all (not just
        // missing default control skins) -- this .tss just re-imports Unity's own built-in
        // runtime theme via its special import URL, same as the file Unity itself generates
        // when you create a UIDocument through the Editor menu.
        settings.themeStyleSheet = Resources.Load<ThemeStyleSheet>("UI/DockingHudTheme");

        _document = gameObject.AddComponent<UIDocument>();
        _document.panelSettings   = settings;
        _document.visualTreeAsset = Resources.Load<VisualTreeAsset>("UI/DockingHud");

        // UIDocument's own root has exactly one child (RootContainer, absolutely positioned),
        // which contributes nothing to flex auto-sizing -- left unset, the root's height
        // collapses toward zero (width alone stretches via the default flex cross-axis
        // stretch), and every panel below it inherits that broken box through its own
        // `inset: 0`, so anything anchored off bottom/vertical-center clusters near the top
        // instead of spreading across the real screen. flexGrow makes the root actually claim
        // the panel's full size.
        _document.rootVisualElement.style.flexGrow = 1;
        _document.rootVisualElement.styleSheets.Add(Resources.Load<StyleSheet>("UI/DockingHud"));
    }

    void BuildEventSystem()
    {
        if (FindFirstObjectByType<EventSystem>() != null) return;

        new GameObject("EventSystem", typeof(EventSystem), typeof(StandaloneInputModule));
        Debug.Log("[DockingUIManager] Created EventSystem (none existed in the scene).");
    }

    void BuildPanels()
    {
        // Left: positional data + Vx/Vy/Vz (absorbs the old DockingHUD top-left box).
        var leftGo = new GameObject("UtilityPositionalPanel");
        leftGo.transform.SetParent(transform, false);
        leftGo.AddComponent<UtilityPositionalPanel>()
              .Initialize(_utilityRoot, nav, detector, rateDamping, corridor, chaser, cfsReceiver, refreshInterval);

        // Right: detailed thruster-firing diagram, half screen height.
        var rightGo = new GameObject("UtilityThrusterPanel");
        rightGo.transform.SetParent(transform, false);
        rightGo.AddComponent<UtilityThrusterPanel>().Initialize(_utilityRoot, rcsModel);

        // Bottom-right: raw control-input/condition debug panel.
        var debugGo = new GameObject("UtilityDebugPanel");
        debugGo.transform.SetParent(transform, false);
        debugGo.AddComponent<UtilityDebugPanel>().Initialize(_utilityRoot, rcsModel, cfsReceiver, rateDamping, refreshInterval);
    }
}
