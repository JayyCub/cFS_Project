using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Single-axis angle dial: a thin gray/translucent ring track with a small white dot marking
/// the current absolute orientation (0-360, raw — not the signed/normalized error value;
/// vehicle attitude legitimately sweeps the full circle as it tumbles, so a marker orbiting
/// the ring reads more naturally than forcing it into a bounded fill). Center shows the
/// numeric degree value; a smaller line below it shows the current rate of change. The axis
/// label sits above the ring rather than inside it, keeping the circle free for the readouts.
/// </summary>
[RequireComponent(typeof(RectTransform))]
public class RadialGauge : MonoBehaviour
{
    // Dot's orbit radius as a fraction of the gauge's half-size — sits mid-thickness on the
    // thin ring track (GetThinRingSprite's inner edge is ~0.82 of the outer edge, so ~0.91
    // splits the difference).
    const float DotOrbitFraction = 0.91f;
    const float DotSize          = 9f;

    // Exposed so callers (UtilityPositionalPanel) can reserve matching vertical space above
    // the gauge's own sizeDelta when laying out a row of these.
    public const float LabelHeight = 14f;
    public const float LabelGap    = 4f;

    private RectTransform _dot;
    private Text          _degreesText;
    private Text          _rateText;
    private float         _orbitRadiusPx;

    public void Initialize(string label, float diameterPx)
    {
        // Only set this gauge's own SIZE here — anchorMin/anchorMax/pivot/anchoredPosition
        // are the caller's responsibility (UtilityPositionalPanel.CreateGauge sets them
        // before calling Initialize, positioning ROLL/PITCH/YAW at three distinct X
        // offsets). Overwriting them here used to re-center every gauge to (0.5,0.5)/(0,0)
        // — the exact center of the parent panel — which put all three gauges on top of
        // each other regardless of where the caller had placed them.
        var rt = (RectTransform)transform;
        rt.sizeDelta = new Vector2(diameterPx, diameterPx);
        _orbitRadiusPx = diameterPx * 0.5f * DotOrbitFraction;

        var track = UIFactory.CreateImage(transform, "Track", UIFactory.GetThinRingSprite(), new Color(1f, 1f, 1f, 0.28f));
        UIFactory.SetAnchor(track.rectTransform, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        var dotImg = UIFactory.CreateImage(transform, "Dot", UIFactory.GetDotSprite(), Color.white);
        _dot = dotImg.rectTransform;
        UIFactory.SetAnchor(_dot, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(0f, _orbitRadiusPx), new Vector2(DotSize, DotSize));

        var labelText = UIFactory.CreateText(transform, "AxisLabel", label, 10,
            new Color(0.75f, 0.85f, 1f, 0.9f), TextAnchor.MiddleCenter);
        UIFactory.SetAnchor(labelText.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(0f, diameterPx / 2f + LabelGap + LabelHeight / 2f),
            new Vector2(diameterPx, LabelHeight));

        // Freed from sharing the circle with the axis label, so the readouts get more of it.
        _degreesText = UIFactory.CreateText(transform, "DegreesText", "0.0°", 16,
            Color.white, TextAnchor.MiddleCenter);
        UIFactory.SetAnchor(_degreesText.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(0f, 5f), new Vector2(diameterPx, 22f));

        _rateText = UIFactory.CreateText(transform, "RateText", "+0.0 d/s", 10,
            new Color(0.75f, 0.75f, 0.75f, 1f), TextAnchor.MiddleCenter);
        UIFactory.SetAnchor(_rateText.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
            new Vector2(0.5f, 0.5f), new Vector2(0f, -diameterPx * 0.28f), new Vector2(diameterPx, 14f));
    }

    /// rawEulerDeg: 0-360 absolute orientation — drives the dot's position around the ring
    /// (0 = top, sweeping clockwise, matching the old fill sweep's convention) and the center
    /// readout. rateDegPerSec: signed normalized rate, shown as the smaller readout below.
    public void SetValue(float rawEulerDeg, float rateDegPerSec)
    {
        float deg = Mathf.Repeat(rawEulerDeg, 360f);

        if (_dot != null)
        {
            float rad = deg * Mathf.Deg2Rad;
            _dot.anchoredPosition = new Vector2(_orbitRadiusPx * Mathf.Sin(rad), _orbitRadiusPx * Mathf.Cos(rad));
        }

        if (_degreesText != null) _degreesText.text = $"{deg:F1}°";
        if (_rateText != null)    _rateText.text    = $"{rateDegPerSec:+0.0;-0.0} d/s";
    }
}
