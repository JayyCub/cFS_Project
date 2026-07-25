using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Drives the per-thruster RCS plume visuals based on live throttle data from RCSModel.
///
/// Primary look: two nested procedural cone meshes per thruster — a tight bright "core
/// jet" and a wider, dimmer "gas halo" — both shaded with Custom/GasCone (Assets/GasCone.shader),
/// which fakes a translucent volumetric gas plume via fresnel rim shading and scrolling
/// noise, no particles or textures involved. Built and driven entirely at runtime; no
/// per-thruster setup required beyond adding this component alongside RCSModel.
///
/// Legacy: the original point-particle plume (ParticleSystem per thruster, created via
/// RCSModel's "Create/Add Thruster Plumes" gear-menu commands) is still supported but
/// disabled by default — set driveLegacyParticlePlumes to layer it back in underneath
/// the cones if useful, e.g. for close-up sparkle.
/// </summary>
[RequireComponent(typeof(RCSModel))]
public class ThrusterPlumes : MonoBehaviour
{
    [Tooltip("Layer the legacy point-particle plume in underneath the cones. Off by default — " +
             "the cones alone are the intended look.")]
    public bool driveLegacyParticlePlumes = false;

    [Tooltip("One ParticleSystem per thruster, matching the RCSModel thruster index order. " +
             "Leave empty to auto-discover from Thruster_XX/Plume children.")]
    [SerializeField] private ParticleSystem[] _plumes;

    [Header("Legacy Particle Plume (only used if driveLegacyParticlePlumes)")]
    [Tooltip("Soft gas puffs emitted per second at full throttle.")]
    public float maxEmissionRate  = 900f;
    [Tooltip("Puff start speed (m/s) at full throttle.")]
    public float maxStartSpeed    = 70f;
    [Tooltip("Puff start size at full throttle.")]
    public float maxStartSize     = 0.9f;
    [Tooltip("Puff lifetime range (seconds) before a puff despawns.")]
    public float particleLifetimeMin = 0.6f;
    public float particleLifetimeMax = 1.0f;
    [Tooltip("Half-angle of the emission cone (degrees) — how much puffs spread out from the nozzle axis.")]
    public float particleShapeAngleDeg = 12f;
    [Tooltip("Puff size multiplier at birth vs. at the end of its life — puffs grow as they drift outward.")]
    public float particleSizeGrowthStart = 0.6f;
    public float particleSizeGrowthEnd   = 2.2f;
    [Tooltip("Puff tint (alpha controls overall translucency).")]
    public Color particleColor = new Color(0.92f, 0.95f, 1f, 0.55f);

    [Tooltip("Normalized throttle fraction below which the plume is suppressed entirely.")]
    public float throttleThreshold = 0.02f;

    [Header("Core Jet Cone")]
    [Tooltip("Length of the bright core cone along the exhaust axis (meters).")]
    public float coreLengthMeters   = 0.9f;
    [Tooltip("Half-angle of the core cone (degrees) — tight, like the hot jet near the throat.")]
    public float coreAngleDeg       = 5f;
    [Tooltip("Intensity multiplier at full throttle.")]
    public float corePeakIntensity  = 1.4f;
    [Tooltip("How broken-up the core surface looks. Low — stays fairly coherent near the nozzle.")]
    [Range(0f, 1f)] public float coreNoiseStrength = 0.15f;
    [Tooltip("How quickly opacity falls off from the centerline toward the edge — higher = " +
             "tighter/brighter core, lower = broader/softer falloff.")]
    public float coreFresnelPower   = 2.0f;
    [Tooltip("Minimum opacity right at the outer edge before it hits the true cutoff at the " +
             "cone boundary. Keep this low/near-zero for a clean fade-to-nothing at the edge.")]
    [Range(0f, 1f)] public float coreFresnelMin = 0.05f;
    [Tooltip("Seconds for the core's overall brightness to pop in/out at the very start/stop of a " +
             "pulse. Keep this short — the actual emerge/dissipate look comes from plumeSpeed below, " +
             "not this; it's just here to avoid a single-frame flash.")]
    public float coreFadeTime       = 0.05f;
    [Tooltip("How fast the noise wisps scroll along the core — higher reads as faster-moving gas.")]
    public float coreScrollSpeed    = 4f;
    [Tooltip("Softness of the growing/receding leading and trailing edges (higher = fuzzier, more gradual).")]
    [Range(0.01f, 0.5f)] public float coreEdgeSoftness = 0.18f;
    [Tooltip("How much opacity drops off the farther the gas has traveled from the nozzle. 0 = no " +
             "falloff, 1 = linear fade to nothing at the tip, higher = holds up near the nozzle then " +
             "drops off sharply near the tip.")]
    [Range(0f, 4f)] public float coreDistanceFalloffPower = 1.2f;

    [Header("Gas Halo Cone")]
    [Tooltip("Length of the diffuse outer cone along the exhaust axis (meters).")]
    public float haloLengthMeters   = 2.4f;
    [Tooltip("Half-angle of the halo cone (degrees) — wider, the expanding gas cloud.")]
    public float haloAngleDeg       = 14f;
    [Tooltip("Intensity multiplier at full throttle. Dimmer than the core.")]
    public float haloPeakIntensity  = 0.6f;
    [Tooltip("How broken-up the halo surface looks. Higher — wispy, dispersing gas.")]
    [Range(0f, 1f)] public float haloNoiseStrength = 0.6f;
    [Tooltip("How quickly opacity falls off from the centerline toward the edge. Lower than the " +
             "core so the halo reads as a broad, filled cloud rather than a tight beam.")]
    public float haloFresnelPower   = 1.0f;
    [Tooltip("Minimum opacity right at the outer edge before it hits the true cutoff at the " +
             "cone boundary. Keep this low/near-zero for a clean fade-to-nothing at the edge.")]
    [Range(0f, 1f)] public float haloFresnelMin = 0.1f;
    [Tooltip("Seconds for the halo's overall brightness to pop in/out at the very start/stop of a " +
             "pulse. Keep this short — the actual emerge/dissipate look comes from plumeSpeed below, " +
             "not this; it's just here to avoid a single-frame flash.")]
    public float haloFadeTime       = 0.08f;
    [Tooltip("How fast the noise wisps scroll along the halo — higher reads as faster-moving gas.")]
    public float haloScrollSpeed    = 8f;
    [Tooltip("Softness of the growing/receding leading and trailing edges (higher = fuzzier, more gradual).")]
    [Range(0.01f, 0.5f)] public float haloEdgeSoftness = 0.3f;
    [Tooltip("How much opacity drops off the farther the gas has traveled from the nozzle. 0 = no " +
             "falloff, 1 = linear fade to nothing at the tip, higher = holds up near the nozzle then " +
             "drops off sharply near the tip.")]
    [Range(0f, 4f)] public float haloDistanceFalloffPower = 0.8f;

    [Header("Shared cone settings")]
    [Tooltip("Number of triangular facets around the cone circumference.")]
    public int coneSegments = 20;
    [Tooltip("How fast the plume's visible leading edge travels outward from the nozzle (m/s), and " +
             "the trailing edge chases it once firing stops. This is what makes the plume actually " +
             "emerge/recede instead of the whole cone just fading its brightness in place. Lower = " +
             "slower, more visible growth/drift; higher = snappier but harder to actually see happen.")]
    public float plumeSpeed = 9f;

    private RCSModel              _rcs;
    private MeshRenderer[]        _coreRenderers;
    private MeshRenderer[]        _haloRenderers;
    private float[]               _coreIntensities;
    private float[]               _haloIntensities;
    private float[]               _coreFront;
    private float[]               _coreTail;
    private float[]               _haloFront;
    private float[]               _haloTail;
    private MaterialPropertyBlock _propBlock;

    void Start()
    {
        _rcs       = GetComponent<RCSModel>();
        _propBlock = new MaterialPropertyBlock();

        AutoCreatePlumeCones();

        if (!driveLegacyParticlePlumes) return;

        if (_plumes == null || _plumes.Length == 0)
            AutoDiscoverPlumes();
        if (_plumes == null) return;

        foreach (var ps in _plumes)
        {
            if (ps == null) continue;
            ApplyLegacyParticleSettings(ps);
            var e = ps.emission;
            e.rateOverTime = 0f;
            ps.Play();
        }
    }

    // Applies the Inspector-tunable knobs to a legacy particle plume at runtime — mirrors
    // how the cones are fully configured from Inspector fields in AutoCreatePlumeCones,
    // rather than leaving these baked into RCSModel.ConfigureThrusterPlume at edit time.
    void ApplyLegacyParticleSettings(ParticleSystem ps)
    {
        var main = ps.main;
        main.startLifetime = new ParticleSystem.MinMaxCurve(particleLifetimeMin, particleLifetimeMax);
        main.startColor    = particleColor;

        var shape = ps.shape;
        shape.angle = particleShapeAngleDeg;

        var size = ps.sizeOverLifetime;
        size.enabled = true;
        size.size = new ParticleSystem.MinMaxCurve(1f,
            AnimationCurve.Linear(0f, particleSizeGrowthStart, 1f, particleSizeGrowthEnd));
    }

    void AutoDiscoverPlumes()
    {
        var transforms = _rcs.ThrusterTransforms;
        if (transforms == null) return;

        _plumes = new ParticleSystem[transforms.Length];
        for (int i = 0; i < transforms.Length; i++)
            if (transforms[i] != null)
                _plumes[i] = transforms[i].GetComponentInChildren<ParticleSystem>();
    }

    void AutoCreatePlumeCones()
    {
        var transforms = _rcs.ThrusterTransforms;
        if (transforms == null) return;

        var shader = Shader.Find("Custom/GasCone");
        if (shader == null)
        {
            Debug.LogWarning("[ThrusterPlumes] Custom/GasCone shader not found — skipping plume cones. " +
                             "Ensure GasCone.shader is in the Assets folder.");
            return;
        }

        // Two shared materials (core/halo), each reused across all 16 thrusters — per-instance
        // fade/style uses MaterialPropertyBlock (see UpdateCone) so there is no per-object
        // material instance cost, and the style knobs stay live-editable in Play mode.
        var coreMat = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        coreMat.SetColor("_Color", new Color(0.95f, 0.97f, 1f, 1f));
        coreMat.SetFloat("_NoiseScale", 3f);

        var haloMat = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
        haloMat.SetColor("_Color", new Color(0.85f, 0.9f, 0.95f, 1f));
        haloMat.SetFloat("_NoiseScale", 1.6f);

        var coreMesh = BuildConeMesh(coreLengthMeters, coreAngleDeg, coneSegments);
        var haloMesh = BuildConeMesh(haloLengthMeters, haloAngleDeg, coneSegments);

        _coreRenderers   = new MeshRenderer[transforms.Length];
        _haloRenderers   = new MeshRenderer[transforms.Length];
        _coreIntensities = new float[transforms.Length];
        _haloIntensities = new float[transforms.Length];
        _coreFront       = new float[transforms.Length];
        _coreTail        = new float[transforms.Length];
        _haloFront       = new float[transforms.Length];
        _haloTail        = new float[transforms.Length];

        for (int i = 0; i < transforms.Length; i++)
        {
            if (transforms[i] == null) continue;

            _coreRenderers[i] = MakeConeRenderer(transforms[i], "CoreJet", coreMesh, coreMat);
            _haloRenderers[i] = MakeConeRenderer(transforms[i], "GasHalo", haloMesh, haloMat);
        }
    }

    MeshRenderer MakeConeRenderer(Transform parent, string name, Mesh mesh, Material mat)
    {
        var go = new GameObject(name) { hideFlags = HideFlags.DontSave };
        go.transform.SetParent(parent, false);
        // No local rotation: cone mesh is built with tip along local +Z,
        // which already aligns with the thruster's exhaust direction.

        var mf = go.AddComponent<MeshFilter>();
        mf.sharedMesh = mesh;

        var mr = go.AddComponent<MeshRenderer>();
        mr.sharedMaterial    = mat;
        mr.shadowCastingMode = ShadowCastingMode.Off;
        mr.receiveShadows    = false;

        // Start invisible and fully collapsed; LateUpdate drives all three each frame.
        _propBlock.SetFloat("_Intensity", 0f);
        _propBlock.SetFloat("_FrontT", 0f);
        _propBlock.SetFloat("_TailT", 0f);
        mr.SetPropertyBlock(_propBlock);

        return mr;
    }

    // Builds a cone with multiple evenly-spaced rings whose alpha follows a sine bell
    // curve — zero at both ends, peaking in the middle. More rings = smoother, fuzzier
    // gradient with no hard edges at the nozzle or the rim.
    static Mesh BuildConeMesh(float length, float angleDeg, int segments)
    {
        const int   ringCount = 9;
        const byte  peakAlpha = 130;
        float tanAngle = Mathf.Tan(angleDeg * Mathf.Deg2Rad);

        // Vertex layout: tip (index 0) + ringCount rings of 'segments' vertices
        var vertices  = new Vector3[1 + ringCount * segments];
        var colors    = new Color32[1 + ringCount * segments];
        var uvs       = new Vector2[1 + ringCount * segments];

        // Nozzle tip — alpha 0 so no hard circle at the base
        vertices[0] = Vector3.zero;
        colors[0]   = new Color32(255, 255, 255, 0);
        uvs[0]      = Vector2.zero;

        for (int r = 0; r < ringCount; r++)
        {
            float t      = (float)r / (ringCount - 1);           // 0..1 along the cone
            float z      = t * length;
            float radius = z * tanAngle;
            byte  alpha  = (byte)(peakAlpha * Mathf.Sin(Mathf.PI * t)); // bell: 0→peak→0

            for (int s = 0; s < segments; s++)
            {
                float a    = 2f * Mathf.PI * s / segments;
                int   vIdx = 1 + r * segments + s;
                vertices[vIdx] = new Vector3(Mathf.Cos(a) * radius, Mathf.Sin(a) * radius, z);
                colors[vIdx]   = new Color32(255, 255, 255, alpha);
                uvs[vIdx]      = new Vector2(t, 0f); // t carried to the shader for the traveling front/tail edge
            }
        }

        // Triangles: tip fan to ring 0, then quad bands between consecutive rings
        var triangles = new int[segments * 3 + (ringCount - 1) * segments * 6];
        int ti = 0;

        for (int s = 0; s < segments; s++)
        {
            triangles[ti++] = 0;
            triangles[ti++] = 1 + s;
            triangles[ti++] = 1 + (s + 1) % segments;
        }

        for (int r = 0; r < ringCount - 1; r++)
        {
            int baseA = 1 + r * segments;
            int baseB = 1 + (r + 1) * segments;
            for (int s = 0; s < segments; s++)
            {
                int s1 = (s + 1) % segments;
                triangles[ti++] = baseA + s;
                triangles[ti++] = baseB + s;
                triangles[ti++] = baseA + s1;
                triangles[ti++] = baseA + s1;
                triangles[ti++] = baseB + s;
                triangles[ti++] = baseB + s1;
            }
        }

        var mesh = new Mesh
        {
            name      = "GasConeMesh",
            hideFlags = HideFlags.HideAndDontSave,
            vertices  = vertices,
            colors32  = colors,
            uv        = uvs,
            triangles = triangles
        };
        mesh.RecalculateNormals();
        return mesh;
    }

    void LateUpdate()
    {
        if (_rcs == null) return;

        int count = _coreRenderers != null ? _coreRenderers.Length : 0;
        if (driveLegacyParticlePlumes && _plumes != null && _plumes.Length > count)
            count = _plumes.Length;

        for (int i = 0; i < count; i++)
        {
            float t      = _rcs.GetThrottle(i);
            bool  firing = t >= throttleThreshold;

            if (driveLegacyParticlePlumes && _plumes != null && i < _plumes.Length)
            {
                var ps = _plumes[i];
                if (ps != null)
                {
                    var emission = ps.emission;
                    if (!firing)
                    {
                        emission.rateOverTime = 0f;
                    }
                    else
                    {
                        var main = ps.main;
                        emission.rateOverTime = maxEmissionRate * t;
                        main.startSpeed       = maxStartSpeed   * t;
                        main.startSize        = maxStartSize    * t;
                    }
                }
            }

            UpdateCone(_coreRenderers, _coreIntensities, _coreFront, _coreTail, i, firing,
                       corePeakIntensity, coreFadeTime, coreLengthMeters,
                       new ConeStyle(coreFresnelPower, coreFresnelMin, coreNoiseStrength,
                                      coreScrollSpeed, coreEdgeSoftness, coreDistanceFalloffPower));
            UpdateCone(_haloRenderers, _haloIntensities, _haloFront, _haloTail, i, firing,
                       haloPeakIntensity, haloFadeTime, haloLengthMeters,
                       new ConeStyle(haloFresnelPower, haloFresnelMin, haloNoiseStrength,
                                      haloScrollSpeed, haloEdgeSoftness, haloDistanceFalloffPower));
        }
    }

    // Bundles the shader style knobs so they can be tuned live in Play mode (pushed via
    // MaterialPropertyBlock every frame in UpdateCone) instead of baked once in Start().
    readonly struct ConeStyle
    {
        public readonly float fresnelPower, fresnelMin, noiseStrength, scrollSpeed, edgeSoftness, distanceFalloffPower;
        public ConeStyle(float fresnelPower, float fresnelMin, float noiseStrength, float scrollSpeed,
                          float edgeSoftness, float distanceFalloffPower)
        {
            this.fresnelPower         = fresnelPower;
            this.fresnelMin           = fresnelMin;
            this.noiseStrength        = noiseStrength;
            this.distanceFalloffPower = distanceFalloffPower;
            this.scrollSpeed   = scrollSpeed;
            this.edgeSoftness  = edgeSoftness;
        }
    }

    // Drives four things per cone: overall brightness fade (intensity), the shader style
    // knobs (fresnel/noise/scroll/edge softness — live-tunable in Play mode), and a
    // traveling leading/trailing edge pair (front/tail, 0=nozzle..1=tip) so the plume
    // visibly grows out of the nozzle when firing starts and the tail-end sails off and
    // clears out when firing stops, instead of the whole static cone just fading in place.
    void UpdateCone(MeshRenderer[] renderers, float[] intensities, float[] front, float[] tail,
                     int i, bool firing, float peakIntensity, float fadeTime, float lengthMeters, ConeStyle style)
    {
        if (renderers == null || i >= renderers.Length) return;
        var mr = renderers[i];
        if (mr == null) return;

        // Fraction of the cone's length the gas front/tail crosses per second.
        float travelRate = plumeSpeed / Mathf.Max(lengthMeters, 0.01f);

        if (firing)
        {
            // Fresh gas keeps feeding from the nozzle: tail stays pinned at the base
            // while the leading edge grows out toward the tip.
            tail[i]  = 0f;
            front[i] = Mathf.Min(1f, front[i] + travelRate * Time.deltaTime);
        }
        else if (front[i] > 0f || tail[i] > 0f)
        {
            // No more gas being fed: the already-expelled packet keeps traveling as a
            // unit until the trailing edge clears the tip, then it's fully gone.
            front[i] = Mathf.Min(1.15f, front[i] + travelRate * Time.deltaTime);
            tail[i]  = Mathf.Min(1.15f, tail[i]  + travelRate * Time.deltaTime);
            if (tail[i] >= 1.15f)
            {
                front[i] = 0f;
                tail[i]  = 0f;
            }
        }

        // Intensity ramps to peak fast on ignition (just enough to avoid a single-frame
        // pop). On release it must NOT hold flat at peak for the whole drift — that reads
        // as "still firing," which is exactly the "extra fraction-of-a-second" complaint —
        // so instead it dims in step with how much of the packet the trailing edge has
        // already eaten through, finishing at 0 right as the tail clears the tip.
        bool  packetActive = firing || front[i] > 0f || tail[i] > 0f;
        float fadeTarget;
        if (firing)
            fadeTarget = peakIntensity;
        else if (packetActive)
            fadeTarget = peakIntensity * (1f - Mathf.Clamp01(tail[i] / 1.15f));
        else
            fadeTarget = 0f;

        float fadeSpeed = peakIntensity / Mathf.Max(fadeTime, 0.001f);
        intensities[i] = Mathf.MoveTowards(intensities[i], fadeTarget, fadeSpeed * Time.deltaTime);

        _propBlock.SetFloat("_Intensity", intensities[i]);
        _propBlock.SetFloat("_FrontT", front[i]);
        _propBlock.SetFloat("_TailT", tail[i]);
        _propBlock.SetFloat("_FresnelPower", style.fresnelPower);
        _propBlock.SetFloat("_FresnelMin", style.fresnelMin);
        _propBlock.SetFloat("_NoiseStrength", style.noiseStrength);
        _propBlock.SetFloat("_ScrollSpeed", style.scrollSpeed);
        _propBlock.SetFloat("_EdgeSoftness", style.edgeSoftness);
        _propBlock.SetFloat("_DistanceFalloffPower", style.distanceFalloffPower);
        mr.SetPropertyBlock(_propBlock);
    }
}
