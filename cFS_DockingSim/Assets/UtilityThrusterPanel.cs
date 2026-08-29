using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Right-edge, always-visible panel: detailed thruster-firing ring (T00-T15 labeled).
/// Vertically centered on the right edge -- no slide-out/tab, matching
/// UtilityPositionalPanel's always-visible left-edge panel.
///
/// Centered via flexbox (an invisible full-screen "anchor" with justify-content/align-items)
/// rather than computed pixel offsets: manual positioning proved unreliable here -- `right`
/// alone didn't reliably combine with an explicit width for an absolutely positioned element
/// in this project's Unity version, and a reactive GeometryChangedEvent measurement raced
/// with the diagram's own deferred construction (ThrusterFiringDiagram populates DiagramHost
/// after Content's structure is otherwise complete). Flexbox centering sidesteps both -- it
/// doesn't need to know Body's size, or when it settles, at all.
/// </summary>
public class UtilityThrusterPanel : MonoBehaviour
{
    const float PanelWidth      = 280f; // wide enough that the 220px diagram gets breathing room inside the box's own 15px padding
    const float DiagramDiameter = 220f;
    const float EdgeGap         = 0f;

    public void Initialize(VisualElement parent, RCSModel rcsModel)
    {
        var anchor = new VisualElement { name = "ThrusterAnchor" };
        anchor.style.position = Position.Absolute;
        anchor.style.left = 0; anchor.style.right = 0; anchor.style.top = 0; anchor.style.bottom = 0;
        anchor.style.justifyContent = Justify.Center; // vertical centering (main axis, column direction)
        anchor.style.alignItems     = Align.FlexEnd;  // pin to the right edge (cross axis)
        anchor.style.paddingRight   = EdgeGap;
        parent.Add(anchor);

        var body = new VisualElement { name = "Body" };
        body.AddToClassList("panel");
        // Override .panel's `position: absolute` -- Body needs to be a normal-flow child of
        // `anchor` for justify-content/align-items to actually center it; an absolutely
        // positioned child is removed from flex layout entirely and ignores both.
        body.style.position = Position.Relative;
        body.style.width = PanelWidth;
        anchor.Add(body);

        var template = Resources.Load<VisualTreeAsset>("UI/UtilityThrusterPanel");
        var content  = template.Instantiate();
        body.Add(content);

        var host = content.Q<VisualElement>("DiagramHost");
        gameObject.AddComponent<ThrusterFiringDiagram>().Initialize(host, rcsModel, true, DiagramDiameter);
    }
}
