using UnityEngine;

/// <summary>
/// Right-edge Utility-mode panel: detailed thruster-firing ring (T00-T15 labeled) — the same
/// ThrusterFiringDiagram used compact in StreamModeOverlay. Fixed size rather than a live
/// screen-height percentage (matches the fixed-pixel-box convention every other HUD panel in
/// this project already uses, e.g. DockingHUD sizes its box from content, not from screen %).
/// </summary>
[RequireComponent(typeof(RectTransform))]
public class UtilityThrusterPanel : MonoBehaviour
{
    static readonly Color   PanelBg   = new Color(0f, 0f, 0f, 0.62f);
    static readonly Vector2 PanelSize = new Vector2(260f, 420f); // ~half of a typical 900-1000px window

    public void Initialize(RCSModel rcsModel)
    {
        var rt = (RectTransform)transform;
        UIFactory.SetAnchor(rt, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f), new Vector2(1f, 0.5f),
            new Vector2(-12f, 0f), PanelSize);

        var slide = gameObject.AddComponent<SlideOutPanel>();
        slide.Initialize(SlideEdge.Right, PanelSize, PanelBg);

        var header = UIFactory.CreateText(slide.Content, "Header", "THRUSTERS", 13,
            new Color(0.62f, 0.82f, 1f, 0.88f), TextAnchor.UpperCenter);
        UIFactory.SetAnchor(header.rectTransform, new Vector2(0f, 1f), new Vector2(1f, 1f),
            new Vector2(0.5f, 1f), new Vector2(0f, -10f), new Vector2(0f, 20f));

        var diagramGo = new GameObject("Diagram", typeof(RectTransform));
        diagramGo.transform.SetParent(slide.Content, false);
        UIFactory.SetAnchor((RectTransform)diagramGo.transform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(0f, -14f), Vector2.zero);
        diagramGo.AddComponent<ThrusterFiringDiagram>().Initialize(rcsModel, true, 220f);
    }
}
