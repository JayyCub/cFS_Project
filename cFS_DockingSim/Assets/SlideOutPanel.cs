using UnityEngine.UIElements;

public enum SlideEdge { Left, Right, Bottom }

/// <summary>
/// Generic, content-agnostic slide-out panel: a `.panel` VisualElement (`Body`) that slides
/// in/out along one screen edge via a USS transition on `translate`. Knows nothing about
/// docking data or how it's toggled (click, key press, etc.) -- the owning script positions
/// Body's fixed on-screen anchor after construction, calls Show()/Hide()/Toggle() however it
/// likes, and populates the returned Content container with whatever it wants to show.
/// </summary>
public class SlideOutPanel
{
    // Slides fully clear of Body's own box (100%) plus a little extra so it's never peeking
    // in at the edge -- mirrors the old HiddenMargin's clearance buffer.
    const float HiddenPercent = 105f;

    public VisualElement Body    { get; private set; }
    public VisualElement Content { get; private set; }
    public bool          IsExpanded { get; private set; }

    private readonly SlideEdge _edge;

    public SlideOutPanel(VisualElement parent, SlideEdge edge, float width, float height, bool startExpanded = true)
    {
        _edge = edge;

        Body = new VisualElement { name = "Body" };
        Body.AddToClassList("panel");
        Body.AddToClassList("slide-panel");
        Body.style.width  = width;
        Body.style.height = height;
        parent.Add(Body);

        Content = new VisualElement { name = "Content" };
        // Never compress below its natural content height, however tall that turns out to be
        // -- if Body's own height ends up shorter than what Content actually needs, this
        // overflows past Body's edge instead of squishing every row inside it to fit.
        Content.style.flexShrink = 0;
        Body.Add(Content);

        IsExpanded = startExpanded;
        ApplyState();
    }

    public void Toggle() { IsExpanded = !IsExpanded; ApplyState(); }
    public void Show()   { IsExpanded = true;  ApplyState(); }
    public void Hide()   { IsExpanded = false; ApplyState(); }

    void ApplyState()
    {
        Body.style.translate = IsExpanded ? new Translate(0, 0) : _edge switch
        {
            SlideEdge.Left   => new Translate(new Length(-HiddenPercent, LengthUnit.Percent), 0),
            SlideEdge.Right  => new Translate(new Length(HiddenPercent, LengthUnit.Percent), 0),
            SlideEdge.Bottom => new Translate(0, new Length(HiddenPercent, LengthUnit.Percent)),
            _                => new Translate(0, 0),
        };
    }
}
