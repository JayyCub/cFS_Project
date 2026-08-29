using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
/// Ring diagram of the 16 RCS thrusters, built from the Dragon-UI art (Assets/Dragon_UI):
/// a hand-drawn backplate (scalloped port ring + alignment ring) behind three visual tiers of
/// thruster, each animated to match the source art's own keyframes/opacity levels rather than
/// an invented brightness curve:
///
///  - T00-T03 (orbital lockout -- see RCSModel.OrbitalLockoutMask, never fire during docking):
///    a static dim dot. Included for completeness even though they never activate.
///  - T04-T07 ("large" thrusters): a dot that steps through the source art's 7-keyframe
///    opacity sequence (50/75/100/100/84/68/50%) plus a black ring that appears at full
///    activation and fades out during release, driven by a single continuous `_largeProgress`
///    value per thruster (see UpdateLarge) so brief/PWM-style pulses interrupt and reverse
///    smoothly instead of snapping.
///  - T08-T15 ("small" thrusters): a simple two-channel crossfade -- dot opacity 50%->100%,
///    plume opacity 0%->50% -- eased in/out together, matching the source art's two paired
///    layers. The plume points tangentially, not at the diagram's center: yaw (T08-T11) and
///    pitch (T12-T15) fire in opposite rotational directions around the ring to form a torque
///    couple, so their plumes point opposite ways rather than both inward (see BuildDots).
///
/// Reused compact (Stream mode corner) and detailed (Utility mode right panel, larger) via the
/// `detailed` flag passed to Initialize().
///
/// Built from VisualElements rather than RectTransforms: position/rotation math is unchanged
/// from the original, but ported across the Y-up (RectTransform) -> Y-down (UI Toolkit) axis
/// flip -- see the comments in BuildDots/plume setup for the sign flips that requires.
/// </summary>
public class ThrusterFiringDiagram : MonoBehaviour
{
    const int ThrusterCount = 16;
    const int LockedOutUpTo = 4; // T00-T03
    const int LargeFirst    = 4, LargeLast  = 7;  // T04-T07
    const int SmallFirst    = 8, SmallLast  = 15; // T08-T15

    const float FiringThreshold = 0.05f;
    const float DotSizeScale    = 0.67f; // global size trim applied to every dot (both tiers); ring/plume sizes derive from dot size so they shrink to match automatically

    // Layout: traced from dragon_ui_reference.svg, which lays out all 16 dots. That art isn't a
    // uniform 16-spoke ring -- it's 4 quadrant clusters on an outer ring, plus a tight 2x2
    // cluster of the 4 locked-out dots (T00-T03) dead-center, separate from the functional ring
    // entirely. Quadrant membership is NOT consecutive index pairs (T08,T09 are not the same
    // quadrant) -- it's the ThrusterTestUI's own grouping: T04-T07 is one "approach" thruster per
    // quadrant, T08-T11 one "brake-yaw" thruster per quadrant, T12-T15 one "brake-pitch"
    // thruster per quadrant, all four groups in the same quadrant order. Quadrant order/angles
    // are screen-relative (TL/TR/BR/BL), confirmed against the running game rather than assumed
    // from ThrusterTestUI's own NE/NW/SW/SE labels, which don't correspond 1:1 to screen
    // position here.
    const float OuterClusterFrac  = 0.64f; // outer-cluster radius as a fraction of the diagram's half-size -- pulled in from the traced 0.76 so dots sit inside the backplate's ring channel instead of on top of its outer line
    const float LockedClusterFrac = 0.11f; // T00-T03 2x2 cluster half-spacing, same units
    const float QuadrantFlankOffset = 17f; // small thrusters sit this many degrees off their quadrant's large-thruster angle
    static readonly float[] QuadrantAngles = { 135f, 45f, 315f, 225f }; // top-left, top-right, bottom-right, bottom-left
    static readonly float[] LargeAngles = QuadrantAngles; // T04-T07: one large thruster per quadrant
    static readonly float[] SmallAngles = // T08-T15: yaw (T08-T11) then pitch (T12-T15), one of each per quadrant
    {
        QuadrantAngles[0] - QuadrantFlankOffset, QuadrantAngles[1] - QuadrantFlankOffset, QuadrantAngles[2] - QuadrantFlankOffset, QuadrantAngles[3] - QuadrantFlankOffset, // T08-T11 (yaw)
        QuadrantAngles[0] + QuadrantFlankOffset, QuadrantAngles[1] + QuadrantFlankOffset, QuadrantAngles[2] + QuadrantFlankOffset, QuadrantAngles[3] + QuadrantFlankOffset, // T12-T15 (pitch)
    };

    // Large thrusters: sizes traced from dragon_ui_large_thruster_keyframes.svg. The ring sits
    // INSET inside the dot (ring circle radius ~10.02 vs dot radius ~13.4 => ~0.75x) -- it's
    // meant to read as a bullseye (white dot -> black ring -> small white center), not a halo
    // around the outside of the dot.
    const float LargeDotToSmallDotRatio = 1.9f;
    const float LargeDotScale             = 0.90f; // overall size trim on top of the traced ratio -- purely a legibility/visual-balance knob
    const float LargeRingSizeMult         = 0.75f; // ring diameter relative to the large dot at the hold frame (p=3) -- inset, not a halo
    const float LargeRingExpandedSizeMult = 1.00f; // ring diameter once fully released (p=5) -- capped at the dot's own edge so it fades out before it would ever show past the dot's boundary
    const float LargeRingFadeInRate       = 9f;    // alpha units/sec when the ring first appears at the hold frame (~110ms fade-in instead of popping in instantly)
    const float LargeAttackRate         = 40f;   // progress units/sec while firing (0->3 in ~150ms)
    const float LargeReleaseRate        = 30f;   // progress units/sec while off (3->6 in ~200ms)
    static readonly float[] LargeOpacityKeyframes = { 0.50f, 0.75f, 1.00f, 1.00f, 0.84f, 0.68f, 0.50f };

    // Small thrusters. NOTE: the plume is sized relative to the diagram's ring radius, not the
    // dot -- dragon_ui_small_thruster_plume_full.svg's raw units (10.69x the dot's diameter)
    // looked correct on paper but assumed the dot and plume SVGs were exported at directly
    // comparable scales. They weren't: at Stream mode's compact ~6px dot that ratio produced a
    // ~60px plume that swallowed the entire diagram. Sizing off the ring radius instead keeps
    // the plume proportional to the diagram it's actually drawn in. The aspect ratio (how much
    // taller than wide) is still traced faithfully from the source art.
    const float PlumeHeightToRingRadius = 1.5f; // a bit bigger than the traced 0.55 so there's room for the round end to clear the dot
    const float PlumeAspect             = 69.16f / 141.72f; // width = height * this
    const float PlumePivotYOffset       = 0.06f; // pushes the plume's rounded end outward past the dot instead of centering it under the dot -- fraction measured from the sprite's own bottom (rounded) edge
    const float SmallDotScale              = 0.80f; // overall size trim, mirrors LargeDotScale -- visual-balance knob, not traced
    const float SmallAttackRate            = 16f; // activation units/sec while firing (~125ms)
    const float SmallReleaseRate           = 12f; // activation units/sec while off (~165ms)
    const float SmallDotOpacityIdle    = 0.50f, SmallDotOpacityActive    = 1.00f;
    const float SmallPlumeOpacityIdle  = 0.00f, SmallPlumeOpacityActive  = 0.18f;

    const float LockedOutOpacity = 0.35f;

    private RCSModel _rcs;
    private float     _largeDotSize; // T04-T07 are all the same size; cached so UpdateLarge can resize the ring

    private VisualElement[] _dots;
    private VisualElement[] _rings;   // only T04-T07 populated
    private VisualElement[] _plumes;  // only T08-T15 populated
    private float[] _throttleCache;
    private float[] _largeProgress;    // 0..6, only T04-T07 meaningful
    private float[] _ringFadeIn;       // 0..1, only T04-T07 meaningful -- separate from _largeProgress so the ring's appearance always eases in over LargeRingFadeInRate regardless of how p got to the hold frame
    private float[] _smallActivation;  // 0..1, only T08-T15 meaningful

    public void Initialize(VisualElement parent, RCSModel rcsModel, bool detailed, float diameterPx)
    {
        _rcs = rcsModel;

        var root = new VisualElement { name = "ThrusterDiagram" };
        root.style.position = Position.Relative;
        root.style.width  = diameterPx;
        root.style.height = diameterPx;
        parent.Add(root);

        var backplate = UIFactory.GetBackplateSprite();
        if (backplate != null)
        {
            var backplateEl = new VisualElement { name = "Backplate" };
            backplateEl.style.position = Position.Absolute;
            backplateEl.style.width    = new Length(100, LengthUnit.Percent);
            backplateEl.style.height   = new Length(100, LengthUnit.Percent);
            backplateEl.style.backgroundImage = UIFactory.ToStyleBackground(backplate);
            root.Add(backplateEl);
        }

        BuildDots(root, detailed, diameterPx);
    }

    void BuildDots(VisualElement root, bool detailed, float diameterPx)
    {
        _dots            = new VisualElement[ThrusterCount];
        _rings           = new VisualElement[ThrusterCount];
        _plumes          = new VisualElement[ThrusterCount];
        _throttleCache   = new float[ThrusterCount];
        _largeProgress   = new float[ThrusterCount];
        _ringFadeIn      = new float[ThrusterCount];
        _smallActivation = new float[ThrusterCount];

        float baseDotSize  = diameterPx * (detailed ? 0.075f : 0.065f) * DotSizeScale;
        float smallDotSize = baseDotSize * SmallDotScale;
        float largeDotSize = baseDotSize * LargeDotToSmallDotRatio * LargeDotScale;
        float ringR      = diameterPx * 0.5f * OuterClusterFrac;
        float lockOffset = diameterPx * LockedClusterFrac;
        _largeDotSize = largeDotSize;

        float centerX = diameterPx / 2f;
        float centerY = diameterPx / 2f;

        for (int i = 0; i < ThrusterCount; i++)
        {
            Vector2 pos;
            float   angle = 0f; // unused for locked-out dots (no ring/plume to orient)

            if (i < LockedOutUpTo)
            {
                // Tight 2x2 cluster at the diagram's center, matching the Dragon-UI reference
                // art -- T00-T03 never fire, so they're tucked away rather than sharing the
                // functional outer ring with the 12 active thrusters.
                float sx = (i == 1 || i == 2) ? 1f : -1f;
                float sy = (i == 0 || i == 1) ? 1f : -1f;
                pos = new Vector2(sx, sy) * lockOffset;
            }
            else if (i <= LargeLast)
            {
                angle = LargeAngles[i - LargeFirst];
                pos   = AnglePos(angle, ringR);
            }
            else
            {
                angle = SmallAngles[i - SmallFirst];
                pos   = AnglePos(angle, ringR);
            }

            // AnglePos was written for Y-up screen space (positive Y = up); UI Toolkit's
            // top/left grow downward, so Y is negated here to land each thruster in the same
            // on-screen quadrant the original RectTransform layout put it in.
            var container = new VisualElement { name = $"T{i:D2}" };
            container.style.position = Position.Absolute;
            container.style.left = centerX + pos.x;
            container.style.top  = centerY - pos.y;
            root.Add(container);

            bool isLarge = i >= LargeFirst && i <= LargeLast;
            float dotSize = isLarge ? largeDotSize : smallDotSize;

            var dot = new VisualElement { name = "Dot" };
            dot.style.position = Position.Absolute;
            dot.style.width  = dotSize;
            dot.style.height = dotSize;
            dot.style.left   = -dotSize / 2f;
            dot.style.top    = -dotSize / 2f;
            dot.style.backgroundImage = UIFactory.ToStyleBackground(UIFactory.GetDotSprite());
            container.Add(dot);
            _dots[i] = dot;

            if (i < LockedOutUpTo)
            {
                dot.style.unityBackgroundImageTintColor = new Color(1f, 1f, 1f, LockedOutOpacity);
            }
            else if (isLarge)
            {
                var ring = new VisualElement { name = "Ring" };
                ring.style.position = Position.Absolute;
                float ringSize = dotSize * LargeRingSizeMult;
                ring.style.width  = ringSize;
                ring.style.height = ringSize;
                ring.style.left   = -ringSize / 2f;
                ring.style.top    = -ringSize / 2f;
                ring.style.backgroundImage = UIFactory.ToStyleBackground(UIFactory.GetThinRingSprite());
                ring.style.unityBackgroundImageTintColor = new Color(0f, 0f, 0f, 0f);
                container.Add(ring);
                _rings[i] = ring;
            }
            else // small, T08-T15: plume points tangentially, not at the center -- yaw (T08-T11)
                 // and pitch (T12-T15) fire in opposite rotational directions to form a torque
                 // couple, so their plumes point opposite ways around the ring, not both inward.
            {
                float plumeHeight = ringR * PlumeHeightToRingRadius;
                float plumeWidth  = plumeHeight * PlumeAspect;

                bool  isPitch       = i >= 12; // T12-T15
                float plumeRotation = isPitch ? angle : angle + 180f; // inverted from the tangent formula -- was pointing at the neighboring thruster instead of away from it

                var plume = new VisualElement { name = "Plume" };
                plume.style.position = Position.Absolute;
                plume.style.width  = plumeWidth;
                plume.style.height = plumeHeight;
                // Pivot at the rounded end (bottom-center of the sprite, nudged up by
                // PlumePivotYOffset) -- position the element so that point sits at the
                // container's origin (the dot center), and move the rotation pivot
                // (transform-origin) to the same point so it rotates like the old
                // RectTransform's pivot did, instead of around its own center.
                plume.style.left = -plumeWidth / 2f;
                plume.style.top  = -plumeHeight * (1f - PlumePivotYOffset);
                plume.style.transformOrigin = new TransformOrigin(
                    new Length(50, LengthUnit.Percent),
                    new Length(100f * (1f - PlumePivotYOffset), LengthUnit.Percent));
                // USS rotation is clockwise-positive in UI Toolkit's Y-down space, opposite the
                // RectTransform Z-euler (counter-clockwise-positive, Y-up) the original angle was
                // computed for -- negate to preserve the original sweep direction.
                plume.style.rotate = new Rotate(-plumeRotation);
                plume.style.backgroundImage = UIFactory.ToStyleBackground(UIFactory.GetPetalPlumeSprite());
                plume.style.unityBackgroundImageTintColor = new Color(1f, 1f, 1f, 0f);
                container.Insert(0, plume); // render behind the dot
                _plumes[i] = plume;
            }
        }
    }

    void Update()
    {
        if (_dots == null) return;

        // Sampled every frame, NOT gated behind a refresh interval like the text-readout panels
        // use -- GetThrottle() is just an array read (cheap), and a firing pulse shorter than
        // that ~100ms text-refresh cadence could get sampled late or missed entirely between
        // polls. The animation needs to track the physical firing state as tightly as the frame
        // rate allows, not the HUD's number-formatting cadence.
        if (_rcs != null)
        {
            for (int i = LockedOutUpTo; i < ThrusterCount; i++)
                _throttleCache[i] = _rcs.GetThrottle(ThrottleSourceIndex(i));
        }

        float dt = Time.deltaTime;
        for (int i = LargeFirst; i <= LargeLast; i++)
            UpdateLarge(i, dt);
        for (int i = SmallFirst; i <= SmallLast; i++)
            UpdateSmall(i, dt);
    }

    void UpdateLarge(int i, float dt)
    {
        bool  firing = _throttleCache[i] > FiringThreshold;
        float p      = _largeProgress[i];

        if (firing)
        {
            p = Mathf.MoveTowards(p, 3f, LargeAttackRate * dt);
        }
        else if (p > 0f)
        {
            // Mid-release (or just finished firing) -- keep sweeping through the release
            // keyframes back to idle. Once idle (p==0), leave it alone: without this guard,
            // idle would keep re-targeting 6 every frame and the dot would perpetually cycle
            // through the keyframe sequence instead of resting.
            p = Mathf.MoveTowards(p, 6f, LargeReleaseRate * dt);
            if (p >= 6f) p = 0f;
        }
        _largeProgress[i] = p;

        float opacity = SampleKeyframes(LargeOpacityKeyframes, p);

        // Ring: absent until the hold frame (p=3). While held, it eases in (LargeRingFadeInRate)
        // instead of popping in instantly. Over the release (p=3..5) it expands outward while
        // fading -- capped at the dot's own edge (LargeRingExpandedSizeMult=1.0) so it's always
        // fully transparent by the time it would reach the dot's boundary, never visible past it.
        bool  ringActive = p >= 3f;
        _ringFadeIn[i] = Mathf.MoveTowards(_ringFadeIn[i], ringActive ? 1f : 0f, LargeRingFadeInRate * dt);

        float ringGrowT        = Mathf.Clamp01(Mathf.InverseLerp(3f, 5f, p));
        float ringReleaseAlpha = p < 3f ? 0f : 1f - ringGrowT;
        float ringAlpha        = Mathf.Min(_ringFadeIn[i], ringReleaseAlpha);
        float ringSizeMult     = Mathf.Lerp(LargeRingSizeMult, LargeRingExpandedSizeMult, ringGrowT);

        if (_dots[i] != null)
            _dots[i].style.unityBackgroundImageTintColor = new Color(1f, 1f, 1f, opacity);

        if (_rings[i] != null)
        {
            _rings[i].style.unityBackgroundImageTintColor = new Color(0f, 0f, 0f, ringAlpha);
            float ringSize = _largeDotSize * ringSizeMult;
            _rings[i].style.width  = ringSize;
            _rings[i].style.height = ringSize;
            _rings[i].style.left   = -ringSize / 2f;
            _rings[i].style.top    = -ringSize / 2f;
        }
    }

    void UpdateSmall(int i, float dt)
    {
        bool  firing = _throttleCache[i] > FiringThreshold;
        float target = firing ? 1f : 0f;
        float rate   = firing ? SmallAttackRate : SmallReleaseRate;
        float a = Mathf.MoveTowards(_smallActivation[i], target, rate * dt);
        _smallActivation[i] = a;

        if (_dots[i]   != null) _dots[i].style.unityBackgroundImageTintColor   = new Color(1f, 1f, 1f, Mathf.Lerp(SmallDotOpacityIdle, SmallDotOpacityActive, a));
        if (_plumes[i] != null) _plumes[i].style.unityBackgroundImageTintColor = new Color(1f, 1f, 1f, Mathf.Lerp(SmallPlumeOpacityIdle, SmallPlumeOpacityActive, a));
    }

    /// Piecewise-linear sample of a 7-point keyframe table across progress domain [0,6].
    static float SampleKeyframes(float[] keyframes, float progress)
    {
        int lo = Mathf.Clamp(Mathf.FloorToInt(progress), 0, keyframes.Length - 1);
        int hi = Mathf.Clamp(lo + 1, 0, keyframes.Length - 1);
        return Mathf.Lerp(keyframes[lo], keyframes[hi], progress - lo);
    }

    static Vector2 AnglePos(float angleDeg, float radius)
    {
        float rad = angleDeg * Mathf.Deg2Rad;
        return new Vector2(Mathf.Cos(rad), Mathf.Sin(rad)) * radius;
    }

    /// The small-thruster (T08-T15) visual slots were wired straight to the matching RCSModel
    /// throttle index, but only two of the four quadrant yaw/pitch pairs were actually crossed
    /// -- firing physical thruster 8 animated the dot sitting in slot 12's position (and 10/14
    /// likewise), while 9/13 and 11/15 were already correct. Swaps only 8<->12 and 10<->14;
    /// 9, 11, 13, 15 (and T00-T07) pass through unchanged.
    static int ThrottleSourceIndex(int visualIndex)
    {
        if (visualIndex < SmallFirst || visualIndex > SmallLast) return visualIndex;
        int offset = visualIndex - SmallFirst; // 0..7
        if (offset % 2 != 0) return visualIndex; // 9, 11, 13, 15: no swap
        return SmallFirst + (offset + 4) % 8; // 8<->12, 10<->14
    }
}
