using UnityEngine;
using UnityEngine.UI;

public enum SlideEdge { Left, Right, Bottom }

/// <summary>
/// Generic, content-agnostic slide-out panel: a tab that tracks the "Body" child 1:1 as it
/// slides between shown and hidden along one screen edge, so the tab always sits right at the
/// body's leading edge — attached to the panel when expanded, poking out from the screen edge
/// like a pull-tab when collapsed. Knows nothing about docking data — the owning script
/// positions this component's own RectTransform at the desired fixed on-screen anchor, calls
/// Initialize(), then populates the returned Content container with whatever it wants to show.
///
/// The tab is a sibling of Body, not a child of it — Body has a RectMask2D so its content
/// doesn't spill past the panel edges while collapsing, and the tab is deliberately positioned
/// just outside Body's own bounds, so parenting it under Body would clip it away entirely.
/// Instead Update() re-derives the tab's position from Body's every frame.
/// </summary>
[RequireComponent(typeof(RectTransform))]
public class SlideOutPanel : MonoBehaviour
{
    const float AnimSpeed    = 10f;
    const float TabSize      = 28f;
    const float TabLength    = 56f;
    const float TabGap       = 2f;
    const float HiddenMargin = 8f; // extra clearance so Body's edge fully clears the screen bound

    public bool           IsExpanded { get; private set; } = true;
    public RectTransform  Content    { get; private set; }

    private SlideEdge      _edge;
    private RectTransform  _body;
    private RectTransform  _tab;
    private Vector2        _tabOffset; // tab's anchoredPosition relative to Body's, held constant
    private Vector2        _shownPos = Vector2.zero;
    private Vector2        _hiddenPos;

    /// sizeDelta is the panel's visible content-area size. Call after this GameObject's own
    /// RectTransform has been anchored to its fixed on-screen position by the caller.
    public void Initialize(SlideEdge edge, Vector2 sizeDelta, Color backgroundColor, bool startExpanded = true)
    {
        _edge = edge;

        _body = UIFactory.CreateContainer(transform, "Body");
        UIFactory.SetAnchor(_body, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), Vector2.zero, sizeDelta);

        var bg = UIFactory.CreateImage(_body, "Background", null, backgroundColor);
        UIFactory.SetAnchor(bg.rectTransform, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);
        _body.gameObject.AddComponent<RectMask2D>(); // belt-and-suspenders: clip any content that overflows the panel

        Content = UIFactory.CreateContainer(_body, "Content");
        UIFactory.SetAnchor(Content, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        _hiddenPos = edge switch
        {
            SlideEdge.Left   => new Vector2(-(sizeDelta.x + HiddenMargin), 0f),
            SlideEdge.Right  => new Vector2(  sizeDelta.x + HiddenMargin,  0f),
            SlideEdge.Bottom => new Vector2(0f, -(sizeDelta.y + HiddenMargin)),
            _                => Vector2.zero,
        };

        BuildTab(sizeDelta);

        IsExpanded = startExpanded;
        _body.anchoredPosition = IsExpanded ? _shownPos : _hiddenPos;
    }

    void BuildTab(Vector2 sizeDelta)
    {
        var tab = UIFactory.CreateButton(transform, "Tab", TabGlyph(), new Color(0.10f, 0.14f, 0.20f, 0.95f), Color.white, out _);
        _tab = tab.GetComponent<RectTransform>();

        switch (_edge)
        {
            case SlideEdge.Left:
                UIFactory.SetAnchor(_tab, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0f, 0.5f),
                    new Vector2(sizeDelta.x / 2f + TabSize / 2f + TabGap, 0f), new Vector2(TabSize, TabLength));
                break;
            case SlideEdge.Right:
                UIFactory.SetAnchor(_tab, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(1f, 0.5f),
                    new Vector2(-(sizeDelta.x / 2f + TabSize / 2f + TabGap), 0f), new Vector2(TabSize, TabLength));
                break;
            case SlideEdge.Bottom:
                UIFactory.SetAnchor(_tab, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 1f),
                    new Vector2(0f, sizeDelta.y / 2f + TabSize / 2f + TabGap), new Vector2(TabLength, TabSize));
                break;
        }

        // Tab tracks Body 1:1 (see Update()) — capture its offset from Body's shared (0.5,0.5)
        // anchor point now, before anything moves.
        _tabOffset = _tab.anchoredPosition;

        tab.onClick.AddListener(Toggle);
    }

    string TabGlyph() => _edge switch
    {
        SlideEdge.Left   => "<",
        SlideEdge.Right  => ">",
        SlideEdge.Bottom => "^",
        _                => "?",
    };

    public void Toggle() { if (IsExpanded) Hide(); else Show(); }
    public void Show()   { IsExpanded = true; }
    public void Hide()   { IsExpanded = false; }

    void Update()
    {
        if (_body == null) return;
        Vector2 target = IsExpanded ? _shownPos : _hiddenPos;
        _body.anchoredPosition = Vector2.Lerp(_body.anchoredPosition, target, Time.deltaTime * AnimSpeed);

        if (_tab != null) _tab.anchoredPosition = _body.anchoredPosition + _tabOffset;
    }
}
