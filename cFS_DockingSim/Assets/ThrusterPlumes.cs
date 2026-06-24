using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Drives the per-thruster ParticleSystems based on live throttle data from RCSModel.
///
/// Setup: run RCSModel → "Create Thruster Child Objects" from the gear menu (it creates
/// a Plume child on each Thruster_XX with a pre-configured ParticleSystem), then add
/// this component to the same GameObject as RCSModel.  No manual wiring needed — plumes
/// are auto-discovered from the thruster hierarchy on Start().
///
/// Override the _plumes array in the Inspector if you want to point at custom
/// ParticleSystem assets instead of the auto-created ones.
///
/// Glow cones are created automatically at runtime under each Thruster_XX as a sibling
/// to the Plume child.  They use the Custom/ConeGlow shader (Assets/ConeGlow.shader).
/// </summary>
[RequireComponent(typeof(RCSModel))]
public class ThrusterPlumes : MonoBehaviour
{
    [Tooltip("One ParticleSystem per thruster, matching the RCSModel thruster index order. " +
             "Leave empty to auto-discover from Thruster_XX/Plume children.")]
    [SerializeField] private ParticleSystem[] _plumes;

    [Header("Plume scaling")]
    [Tooltip("Particles emitted per second at full throttle.")]
    public float maxEmissionRate  = 2000f;
    [Tooltip("Particle start speed (m/s) at full throttle.")]
    public float maxStartSpeed    = 160f;
    [Tooltip("Particle start size at full throttle.")]
    public float maxStartSize     = 0.64f;
    [Tooltip("Normalized throttle fraction below which the plume is suppressed entirely.")]
    public float throttleThreshold = 0.02f;

    [Header("Glow Cone")]
    [Tooltip("Length of the glow cone along the exhaust axis (meters).")]
    public float coneLengthMeters  = 1.2f;
    [Tooltip("Half-angle of the glow cone (degrees). Slightly wider than the 4° particle cone.")]
    public float coneAngleDeg      = 7f;
    [Tooltip("Number of triangular facets around the cone circumference.")]
    public int   coneSegments      = 20;
    [Tooltip("Intensity multiplier at full throttle. Reduce if the glow is too bright.")]
    public float conePeakIntensity = 1.0f;
    [Tooltip("Seconds for the cone to fade in or out when a thruster starts or stops.")]
    public float coneFadeTime = 0.2f;

    private RCSModel              _rcs;
    private MeshRenderer[]        _glowConeRenderers;
    private float[]               _coneIntensities;
    private MaterialPropertyBlock _propBlock;

    void Start()
    {
        _rcs       = GetComponent<RCSModel>();
        _propBlock = new MaterialPropertyBlock();

        if (_plumes == null || _plumes.Length == 0)
            AutoDiscoverPlumes();

        AutoCreateGlowCones();

        if (_plumes == null) return;

        foreach (var ps in _plumes)
        {
            if (ps == null) continue;
            var e = ps.emission;
            e.rateOverTime = 0f;
            ps.Play();
        }
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

    void AutoCreateGlowCones()
    {
        var transforms = _rcs.ThrusterTransforms;
        if (transforms == null) return;

        var shader = Shader.Find("Custom/ConeGlow");
        if (shader == null)
        {
            Debug.LogWarning("[ThrusterPlumes] Custom/ConeGlow shader not found — skipping glow cones. " +
                             "Ensure ConeGlow.shader is in the Assets folder.");
            return;
        }

        // One material shared across all cone renderers; _Intensity is set per-renderer
        // via MaterialPropertyBlock so there is no per-object material instance cost.
        var mat = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };

        // One mesh shared across all 16 MeshFilters — same cone shape, different transforms.
        var mesh = BuildConeMesh(coneLengthMeters, coneAngleDeg, coneSegments);

        _glowConeRenderers = new MeshRenderer[transforms.Length];
        _coneIntensities   = new float[transforms.Length];
        for (int i = 0; i < transforms.Length; i++)
        {
            if (transforms[i] == null) continue;

            var go = new GameObject("GlowCone") { hideFlags = HideFlags.DontSave };
            go.transform.SetParent(transforms[i], false);
            // No local rotation: cone mesh is built with tip along local +Z,
            // which already aligns with the thruster's exhaust direction.

            var mf = go.AddComponent<MeshFilter>();
            mf.sharedMesh = mesh;

            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial      = mat;
            mr.shadowCastingMode   = ShadowCastingMode.Off;
            mr.receiveShadows      = false;

            // Start invisible; LateUpdate will set intensity each frame.
            _propBlock.SetFloat("_Intensity", 0f);
            mr.SetPropertyBlock(_propBlock);

            _glowConeRenderers[i] = mr;
        }
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

        // Nozzle tip — alpha 0 so no hard circle at the base
        vertices[0] = Vector3.zero;
        colors[0]   = new Color32(255, 255, 255, 0);

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
            name      = "ConeGlowMesh",
            hideFlags = HideFlags.HideAndDontSave,
            vertices  = vertices,
            colors32  = colors,
            triangles = triangles
        };
        mesh.RecalculateNormals();
        return mesh;
    }

    void LateUpdate()
    {
        if (_rcs == null) return;

        int count = _plumes != null ? _plumes.Length : 0;
        if (_glowConeRenderers != null && _glowConeRenderers.Length > count)
            count = _glowConeRenderers.Length;

        for (int i = 0; i < count; i++)
        {
            float t      = _rcs.GetThrottle(i);
            bool  firing = t >= throttleThreshold;

            // Particle plume
            if (_plumes != null && i < _plumes.Length)
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

            // Glow cone — fades in/out independently of throttle level
            if (_glowConeRenderers != null && i < _glowConeRenderers.Length)
            {
                var mr = _glowConeRenderers[i];
                if (mr != null)
                {
                    float target      = firing ? conePeakIntensity : 0f;
                    float speed       = conePeakIntensity / Mathf.Max(coneFadeTime, 0.001f);
                    _coneIntensities[i] = Mathf.MoveTowards(_coneIntensities[i], target, speed * Time.deltaTime);
                    _propBlock.SetFloat("_Intensity", _coneIntensities[i]);
                    mr.SetPropertyBlock(_propBlock);
                }
            }
        }
    }
}
