using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Procedural sprite generation shared by the HUD's hand-drawn widgets (currently just
/// ThrusterFiringDiagram). These sprites aren't reproducible from a formula-free asset -- dots/
/// rings are baked once and cached, the petal plume traces the Dragon-UI art's own bezier
/// silhouette, and the backplate is loaded from Resources. Consumed as VisualElement background
/// images via ToStyleBackground.
/// </summary>
public static class UIFactory
{
    private static Sprite _dotSprite;
    private static Sprite _ringSprite;
    private static Sprite _thinRingSprite;
    private static Sprite _petalPlumeSprite;
    private static Sprite _backplateSprite;
    private static bool   _backplateLoadAttempted;

    /// Wraps a sprite for use as a VisualElement's background-image.
    public static StyleBackground ToStyleBackground(Sprite sprite) => new StyleBackground(sprite);

    // ── procedural sprites (built once, cached) ─────────────────────────────

    /// Solid filled circle, ~1.5px soft anti-aliased edge. Used for thruster indicator dots,
    /// tinted/faded per-thruster via unity-background-image-tint-color driven by throttle level.
    public static Sprite GetDotSprite()
    {
        if (_dotSprite != null) return _dotSprite;

        const int size   = 64;
        float     radius = size / 2f - 1f;
        Vector2   center = new Vector2(size / 2f, size / 2f);

        var tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            wrapMode   = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
        {
            float d     = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f), center);
            float alpha = Mathf.Clamp01(radius - d + 1.5f);
            tex.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
        }
        tex.Apply();

        _dotSprite = Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 100f, 0, SpriteMeshType.FullRect);
        return _dotSprite;
    }

    /// Annulus (outer radius ~1.0, inner ~0.72 of the texture half-size), soft AA on both edges.
    /// Used as a gauge's static background track, and -- via a second Image with
    /// type=Filled/fillMethod=Radial360 -- as the animated angle-sweep indicator.
    public static Sprite GetRingSprite()
    {
        if (_ringSprite != null) return _ringSprite;

        const int size    = 128;
        float     outerR  = size / 2f - 1f;
        float     innerR  = outerR * 0.72f;
        Vector2   center  = new Vector2(size / 2f, size / 2f);

        var tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            wrapMode   = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
        {
            float d           = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f), center);
            float outerAlpha  = Mathf.Clamp01(outerR - d + 1.2f);
            float innerAlpha  = Mathf.Clamp01(d - innerR + 1.2f);
            float alpha       = Mathf.Min(outerAlpha, innerAlpha);
            tex.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
        }
        tex.Apply();

        _ringSprite = Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 100f, 0, SpriteMeshType.FullRect);
        return _ringSprite;
    }

    /// Thin circular stroke (not the thick annulus GetRingSprite draws) -- used as the halo
    /// outline that appears around a thruster dot while it's firing.
    public static Sprite GetThinRingSprite()
    {
        if (_thinRingSprite != null) return _thinRingSprite;

        const int size    = 64;
        float     outerR  = size / 2f - 1f;
        float     innerR  = outerR * 0.82f;
        Vector2   center  = new Vector2(size / 2f, size / 2f);

        var tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        {
            wrapMode   = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
        {
            float d          = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f), center);
            float outerAlpha = Mathf.Clamp01(outerR - d + 1.2f);
            float innerAlpha = Mathf.Clamp01(d - innerR + 1.2f);
            float alpha      = Mathf.Min(outerAlpha, innerAlpha);
            tex.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
        }
        tex.Apply();

        _thinRingSprite = Sprite.Create(tex, new Rect(0, 0, size, size), new Vector2(0.5f, 0.5f), 100f, 0, SpriteMeshType.FullRect);
        return _thinRingSprite;
    }

    /// Traces the Dragon-UI plume art's own silhouette: a symmetric petal with a smoothly
    /// rounded end (both edges leave that vertex on a horizontal tangent, so it's a dome, not a
    /// cusp) opposite a flat-cut end. Pivot is bottom-center at the ROUNDED end, at full
    /// opacity, so callers anchor that end near the thruster dot; the flat end fades to fully
    /// transparent and extends away from it.
    public static Sprite GetPetalPlumeSprite()
    {
        if (_petalPlumeSprite != null) return _petalPlumeSprite;

        // Bezier control points traced from dragon_ui_small_thruster_plume_full.svg's own
        // clipPath (its right-hand edge; the left edge is a mirror about the centerline).
        Vector2 p0 = new Vector2(35.527f, 142.445f); // bottom point
        Vector2 p1 = new Vector2(54.359f, 142.445f);
        Vector2 p2 = new Vector2(70.105f, 77.918f);
        Vector2 p3 = new Vector2(70.105f, 0.723f);   // top-right corner
        const float petalWidth    = 69.16f;
        const float petalHeight   = 141.72f;
        const float centerlineX   = 35.527f;

        const int sampleCount = 96;
        var rightEdge = new Vector2[sampleCount]; // descending y: point (index 0) -> top (last index)
        for (int i = 0; i < sampleCount; i++)
        {
            float t  = i / (float)(sampleCount - 1);
            float mt = 1f - t;
            rightEdge[i] = mt * mt * mt * p0 + 3f * mt * mt * t * p1 + 3f * mt * t * t * p2 + t * t * t * p3;
        }

        const int texW = 128, texH = 256;
        const float edgeSoft = 1.4f; // shape-space units of antialiasing on the left/right edge

        var tex = new Texture2D(texW, texH, TextureFormat.RGBA32, false)
        {
            wrapMode   = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        for (int ty = 0; ty < texH; ty++)
        {
            // ty=0 (bottom of texture, this sprite's pivot) = shape's rounded end, full opacity;
            // ty=texH-1 (top of texture) = shape's flat end, fully transparent.
            float t        = ty / (float)(texH - 1);
            float sy       = petalHeight * (1f - t);
            float rightX   = SamplePetalEdge(rightEdge, sy);
            float leftX    = 2f * centerlineX - rightX;
            float vGradient = Mathf.Clamp01(1f - t);

            for (int tx = 0; tx < texW; tx++)
            {
                float sx      = (tx / (float)(texW - 1)) * petalWidth;
                float edge    = Mathf.Min(sx - leftX, rightX - sx);
                float hAlpha  = Mathf.Clamp01(edge / edgeSoft + 0.5f);
                tex.SetPixel(tx, ty, new Color(1f, 1f, 1f, hAlpha * vGradient));
            }
        }
        tex.Apply();

        _petalPlumeSprite = Sprite.Create(tex, new Rect(0, 0, texW, texH), new Vector2(0.5f, 0f), 100f, 0, SpriteMeshType.FullRect);
        return _petalPlumeSprite;
    }

    static float SamplePetalEdge(Vector2[] samplesDescendingY, float queryY)
    {
        int n = samplesDescendingY.Length;
        if (queryY >= samplesDescendingY[0].y) return samplesDescendingY[0].x;
        if (queryY <= samplesDescendingY[n - 1].y) return samplesDescendingY[n - 1].x;

        for (int i = 0; i < n - 1; i++)
        {
            Vector2 a = samplesDescendingY[i];
            Vector2 b = samplesDescendingY[i + 1];
            if (queryY <= a.y && queryY >= b.y)
                return Mathf.Lerp(a.x, b.x, (a.y - queryY) / (a.y - b.y));
        }
        return samplesDescendingY[n - 1].x;
    }

    /// Dragon-UI docking-port backplate -- hand-drawn art (scalloped port ring + inner alignment
    /// ring), cropped from a Canva export and loaded from Resources rather than baked
    /// procedurally like the sprites above; its silhouette isn't reproducible with a formula.
    /// Returns null (logging once) if the asset is missing, so callers can fall back gracefully.
    public static Sprite GetBackplateSprite()
    {
        if (_backplateSprite != null || _backplateLoadAttempted) return _backplateSprite;
        _backplateLoadAttempted = true;

        _backplateSprite = Resources.Load<Sprite>("dragon_ui_backplate");
        if (_backplateSprite == null)
            Debug.LogWarning("[UIFactory] Could not load Resources/dragon_ui_backplate — thruster diagram will render without its backplate art.");

        return _backplateSprite;
    }
}
