using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Minimal SpaceX-broadcast-inspired overlay: big RANGE/RATE readouts plus a compact ring
/// diagram of the 12 active thrusters in the bottom-left corner. The reference's bottom-right
/// "capture complete" reticle is intentionally skipped — not relevant here. Own visual flair
/// beyond this baseline (mission patch, docking port label, elapsed time, ...) is left for a
/// follow-up pass once more reference images are shared — see flairContainer below.
/// </summary>
public class StreamModeOverlay : MonoBehaviour
{
    static readonly Color LabelColor = new Color(0.75f, 0.80f, 0.85f, 0.85f);
    static readonly Color ValueColor = Color.white;

    private RelativeNav _nav;
    private float        _refreshInterval;
    private float        _nextRefresh;

    private Text _rangeValue;
    private Text _rateValue;

    public void Initialize(RelativeNav nav, RCSModel rcsModel, float refreshInterval = 0.1f)
    {
        _nav             = nav;
        _refreshInterval = refreshInterval;

        BuildBottomLeft(rcsModel);
        BuildBottomRight();

        // Reserved for mission-specific flair once more reference images are shared —
        // intentionally empty for now.
        UIFactory.CreateContainer(transform, "flairContainer");
    }

    void BuildBottomLeft(RCSModel rcsModel)
    {
        var group = UIFactory.CreateContainer(transform, "RangeGroup");
        UIFactory.SetAnchor(group, Vector2.zero, Vector2.zero, Vector2.zero, new Vector2(24f, 24f), new Vector2(280f, 90f));

        var ringGo = new GameObject("ThrusterRing", typeof(RectTransform));
        ringGo.transform.SetParent(group, false);
        UIFactory.SetAnchor((RectTransform)ringGo.transform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(45f, 0f), new Vector2(90f, 90f));
        ringGo.AddComponent<ThrusterFiringDiagram>().Initialize(rcsModel, false, 90f);

        var label = UIFactory.CreateText(group, "RangeLabel", "RANGE", 13, LabelColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(label.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f),
            new Vector2(0f, 0.5f), new Vector2(100f, 22f), new Vector2(160f, 20f));

        _rangeValue = UIFactory.CreateText(group, "RangeValue", "-- m", 34, ValueColor, TextAnchor.MiddleLeft);
        UIFactory.SetAnchor(_rangeValue.rectTransform, new Vector2(0f, 0.5f), new Vector2(0f, 0.5f),
            new Vector2(0f, 0.5f), new Vector2(100f, -6f), new Vector2(180f, 44f));
    }

    void BuildBottomRight()
    {
        var group = UIFactory.CreateContainer(transform, "RateGroup");
        UIFactory.SetAnchor(group, Vector2.right, Vector2.right, Vector2.right, new Vector2(-24f, 24f), new Vector2(220f, 90f));

        var label = UIFactory.CreateText(group, "RateLabel", "RATE", 13, LabelColor, TextAnchor.MiddleRight);
        UIFactory.SetAnchor(label.rectTransform, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f),
            new Vector2(1f, 0.5f), new Vector2(0f, 22f), new Vector2(160f, 20f));

        _rateValue = UIFactory.CreateText(group, "RateValue", "-- m/s", 34, ValueColor, TextAnchor.MiddleRight);
        UIFactory.SetAnchor(_rateValue.rectTransform, new Vector2(1f, 0.5f), new Vector2(1f, 0.5f),
            new Vector2(1f, 0.5f), new Vector2(0f, -6f), new Vector2(200f, 44f));
    }

    void Update()
    {
        if (_nav == null || Time.time < _nextRefresh) return;
        _nextRefresh = Time.time + _refreshInterval;

        if (_rangeValue != null) _rangeValue.text = $"{_nav.range:F1} m";
        if (_rateValue  != null) _rateValue.text  = $"{_nav.closingSpeed:+0.00;-0.00} m/s";
    }
}
