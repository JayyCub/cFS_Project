using UnityEngine;

/// <summary>
/// RCS thruster HARDWARE model for the chaser — valves and nozzles, nothing more.
///
/// Since realism phase 2 all thruster decisions live in flight software: the cFS RCS app
/// allocates GNC's impulse request across the thrusters (from its own thruster table),
/// converts it into per-thruster valve on-times (PWM, minimum impulse bit), and those
/// on-times arrive here via SIM_IO → UdpCommandReceiver.SetThrusterOnTimes. This component
/// only opens each valve at the start of the cycle, applies full rated thrust at the
/// nozzle while it is open, and closes it after its on-time. Draco thrusters don't
/// throttle — a thruster is either firing at thrusterForce or off.
///
/// Valve timing is resolved within a physics step: a valve that closes part-way through a
/// step contributes the matching fraction of that step's thrust, so the delivered impulse
/// equals thrusterForce × on-time regardless of the 0.02 s physics tick.
///
/// Crew keyboard input no longer fires thrusters directly — see HandController (it goes
/// through cFS like the real vehicle's hand controllers).
/// </summary>
public class RCSModel : MonoBehaviour
{
    public VehicleState vehicle;

    [Header("Thruster Hardware")]
    [Tooltip("Rated thrust of every thruster (N) — the real hardware value. The cFS RCS app has " +
             "its own copy (RCS_ThrTbl.Thrust_N); a mismatch shows up as a thrust-magnitude modelling " +
             "error for flight software to cope with, not as a broken link.")]
    public float thrusterForce = 400f;

    [Tooltip("One Transform per physical thruster. Local +Z = exhaust direction (nozzle out). " +
             "Force is applied in the −Z direction at that world position.")]
    [SerializeField] private Transform[] thrusterTransforms;

    [Header("Debug")]
    [Tooltip("Suppress all thruster forces (T key). Valves still report open for gizmos/UI.")]
    public bool suppressForces = false;

    // Time.fixedTime at which each valve closes; at or before "now" = closed.
    private float[]   _valveCloseTime = new float[0];
    // Thrust (N) each thruster applied in the most recent physics step — UI/plumes/gizmos.
    private float[]   _throttles      = new float[0];
    private Rigidbody _rb;

    public int         ThrusterCount      => thrusterTransforms != null ? thrusterTransforms.Length : 0;
    public Transform[] ThrusterTransforms => thrusterTransforms;

    /// <summary>Thrust fraction (0–1) thruster i applied last physics step. Safe to call before init.</summary>
    public float GetThrottle(int i) =>
        (_throttles != null && i >= 0 && i < _throttles.Length && thrusterForce > 0f)
            ? _throttles[i] / thrusterForce
            : 0f;

    /// <summary>Returns true if thruster i exists and its GameObject is active in the scene.</summary>
    public bool IsThrusterActive(int index) =>
        thrusterTransforms != null &&
        index >= 0 && index < thrusterTransforms.Length &&
        thrusterTransforms[index] != null &&
        thrusterTransforms[index].gameObject.activeInHierarchy;

    /// <summary>Bit i set when thruster i fired at ≥5% of a full step last physics step (gizmos/UI).</summary>
    public int CurrentThrusterMask
    {
        get
        {
            int m = 0;
            float threshold = thrusterForce * 0.05f;
            for (int i = 0; i < _throttles.Length && i < 32; i++)
                if (_throttles[i] >= threshold) m |= (1 << i);
            return m;
        }
    }

    void Awake()
    {
        EnsureArrays();
    }

    void Start()
    {
        if (vehicle != null)
            _rb = vehicle.GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.T)) suppressForces = !suppressForces;
    }

    void FixedUpdate()
    {
        EnsureArrays();
        float now = Time.fixedTime;
        float dt  = Time.fixedDeltaTime;

        for (int i = 0; i < _throttles.Length; i++)
        {
            // Fraction of this step the valve is open: 1 for a full step, partial on the step
            // it closes in, 0 once closed.
            float open = Mathf.Clamp01((_valveCloseTime[i] - now) / dt);
            _throttles[i] = IsThrusterActive(i) ? thrusterForce * open : 0f;
        }

        if (_rb == null || suppressForces) return;

        for (int i = 0; i < _throttles.Length; i++)
        {
            if (_throttles[i] <= 0f) continue;
            Transform t = thrusterTransforms[i];
            // Exhaust leaves along the nozzle's +Z; the vehicle is pushed along −Z.
            _rb.AddForceAtPosition(-t.forward * _throttles[i], t.position, ForceMode.Force);
        }
    }

    // ── Valve command API ─────────────────────────────────────────────────────

    /// <summary>
    /// Open thruster i's valve for onTimes[i] seconds starting now (0 = stay closed).
    /// Called by UdpCommandReceiver with the cFS RCS app's on-times each GNC cycle.
    /// </summary>
    public void SetThrusterOnTimes(float[] onTimes)
    {
        EnsureArrays();
        float start = ValveStartTime();
        for (int i = 0; i < _valveCloseTime.Length; i++)
        {
            float t = (onTimes != null && i < onTimes.Length) ? onTimes[i] : 0f;
            _valveCloseTime[i] = t > 0f ? start + t : start;
        }
    }

    /// <summary>
    /// Bench-test path (ThrusterTestUI, ThrusterDiagnostic): fire the thrusters in
    /// <paramref name="mask"/> at full thrust for <paramref name="duration"/> seconds, bypassing
    /// flight software entirely — like firing valves from ground support equipment.
    /// </summary>
    public void SetThrusterCommand(int mask, float duration)
    {
        EnsureArrays();
        var onTimes = new float[_valveCloseTime.Length];
        for (int i = 0; i < onTimes.Length && i < 32; i++)
            onTimes[i] = (mask & (1 << i)) != 0 ? duration : 0f;
        SetThrusterOnTimes(onTimes);
    }

    /// <summary>Close every valve immediately.</summary>
    public void ClearExternalControl()
    {
        EnsureArrays();
        for (int i = 0; i < _valveCloseTime.Length; i++) _valveCloseTime[i] = -1f;
        System.Array.Clear(_throttles, 0, _throttles.Length);
    }

    /// <summary>
    /// Body-frame linear (N·s) and angular (N·m·s, about the CoM) impulse the given on-times
    /// deliver with this vehicle's actual thruster geometry. For HUD/diagnostics — flight
    /// software never sees this; it has its own table.
    /// </summary>
    public void ImpulseFromOnTimes(float[] onTimes, out Vector3 linear, out Vector3 angular)
    {
        linear = angular = Vector3.zero;
        if (onTimes == null) return;
        Vector3 com = WorldCenterOfMass();
        for (int i = 0; i < onTimes.Length && i < ThrusterCount; i++)
        {
            if (onTimes[i] <= 0f || !IsThrusterActive(i)) continue;
            BodyGeometry(i, com, out Vector3 arm, out Vector3 dir);
            float J = thrusterForce * onTimes[i];
            linear  += dir * J;
            angular += Vector3.Cross(arm, dir) * J;
        }
    }

    // A valve command received inside FixedUpdate (lock-step path) opens in this physics
    // step; one received from Update (free-running fallback) opens in the next one.
    static float ValveStartTime() =>
        Time.inFixedTimeStep ? Time.fixedTime : Time.fixedTime + Time.fixedDeltaTime;

    void EnsureArrays()
    {
        int n = ThrusterCount;
        if (_valveCloseTime == null || _valveCloseTime.Length != n)
        {
            _valveCloseTime = new float[n];
            for (int i = 0; i < n; i++) _valveCloseTime[i] = -1f;
        }
        if (_throttles == null || _throttles.Length != n)
            _throttles = new float[n];
    }

    // ── Geometry (for HUD impulse and regenerating the cFS thruster table) ─────

    Vector3 WorldCenterOfMass()
    {
        if (_rb != null) return _rb.worldCenterOfMass;
        // Edit mode: VehicleState applies its CoM override at Start, so use it directly.
        if (vehicle != null && vehicle.centerOfMassOverride != Vector3.zero)
            return vehicle.transform.TransformPoint(vehicle.centerOfMassOverride);
        return transform.position;
    }

    // Body frame (this transform): moment arm from CoM, and the unit push direction.
    void BodyGeometry(int i, Vector3 worldCoM, out Vector3 arm, out Vector3 dir)
    {
        Transform t = thrusterTransforms[i];
        arm = transform.InverseTransformDirection(t.position - worldCoM);
        dir = -transform.InverseTransformDirection(t.forward).normalized;
    }

    /// <summary>
    /// Prints this scene's thruster geometry as the C initializer for cFS
    /// apps/rcs/fsw/tables/rcs_thr_tbl.c — run after moving/re-canting any thruster so
    /// flight software's table matches the hardware again.
    /// </summary>
    [ContextMenu("Log cFS thruster table (rcs_thr_tbl.c)")]
    void LogCfsThrusterTable()
    {
        Vector3 com = WorldCenterOfMass();
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"[RCSModel] cFS RCS table rows (body frame, relative to CoM {transform.InverseTransformPoint(com):F3} local):");
        for (int i = 0; i < ThrusterCount; i++)
        {
            if (thrusterTransforms[i] == null) continue;
            BodyGeometry(i, com, out Vector3 r, out Vector3 d);
            int enabled = (i >= 4 && IsThrusterActive(i)) ? 1 : 0; // T00-T03: deorbit, never for docking
            sb.AppendLine($"        {{ {{ {r.x:+0.0000;-0.0000}f, {r.y:+0.0000;-0.0000}f, {r.z:+0.0000;-0.0000}f }}, " +
                          $"{{ {d.x:+0.0000;-0.0000}f, {d.y:+0.0000;-0.0000}f, {d.z:+0.0000;-0.0000}f }}, {enabled} }}, /* T{i:D2} */");
        }
        Debug.Log(sb.ToString());
    }

    // ── Gizmos ───────────────────────────────────────────────────────────────
    // Uses CurrentThrusterMask (derived from _throttles) so gizmos reflect the
    // actual proportional state rather than a stale bitmask.

    void OnDrawGizmos()
    {
        if (thrusterTransforms == null) return;
        int mask = CurrentThrusterMask;

        for (int i = 0; i < thrusterTransforms.Length; i++)
        {
            if (thrusterTransforms[i] == null) continue;

            bool    active   = (mask & (1 << i)) != 0;
            Vector3 worldPos = thrusterTransforms[i].position;
            Vector3 worldDir = thrusterTransforms[i].forward;

            Gizmos.color = active
                ? new Color(1f, 0.9f, 0f, 1f)
                : new Color(0.3f, 0.6f, 1f, 0.7f);

            Gizmos.DrawSphere(worldPos, 0.06f);
            Gizmos.DrawRay(worldPos, worldDir * 0.5f);

#if UNITY_EDITOR
            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(worldPos + worldDir * 0.65f, $"T{i}");
#endif
        }
    }

    // ── Editor helper ─────────────────────────────────────────────────────────

#if UNITY_EDITOR
    /// <summary>
    /// Gear menu → Create Thruster Child Objects.
    /// Spawns 16 placeholder GameObjects directly under this RCSModel (no pod parents).
    /// Each thruster's local +Z = exhaust direction; thrust is applied in −Z.
    ///
    /// Bit→thruster mapping:
    ///   +X group: T00–T03   −X group: T04–T07
    ///   +Y group: T08–T11   −Y group: T12–T15
    /// </summary>
    [ContextMenu("Create Thruster Child Objects")]
    void CreateThrusterChildObjects()
    {
        var toDelete = new System.Collections.Generic.List<GameObject>();
        foreach (Transform child in transform)
            if (child.name.StartsWith("ThrusterPod_") || child.name.StartsWith("Thruster_"))
                toDelete.Add(child.gameObject);
        foreach (var go in toDelete)
            DestroyImmediate(go);

        var thrusterData = new (Vector3 pos, Vector3 exhaustDir)[]
        {
            // ── +X group ───────────────────────────────────────────────────────
            ( new Vector3( 1.5f,  0f,  0f), new Vector3( 1,  0, -1).normalized ),  // T00
            ( new Vector3( 1.5f,  0f,  0f), new Vector3( 1,  0,  1).normalized ),  // T01
            ( new Vector3( 1.5f,  0f,  0f), new Vector3( 1, -1,  0).normalized ),  // T02
            ( new Vector3( 1.5f,  0f,  0f), new Vector3( 1,  1,  0).normalized ),  // T03
            // ── −X group ───────────────────────────────────────────────────────
            ( new Vector3(-1.5f,  0f,  0f), new Vector3(-1,  0, -1).normalized ),  // T04
            ( new Vector3(-1.5f,  0f,  0f), new Vector3(-1,  0,  1).normalized ),  // T05
            ( new Vector3(-1.5f,  0f,  0f), new Vector3(-1, -1,  0).normalized ),  // T06
            ( new Vector3(-1.5f,  0f,  0f), new Vector3(-1,  1,  0).normalized ),  // T07
            // ── +Y group ───────────────────────────────────────────────────────
            ( new Vector3( 0f,  1.5f,  0f), new Vector3( 0,  1, -1).normalized ),  // T08
            ( new Vector3( 0f,  1.5f,  0f), new Vector3( 0,  1,  1).normalized ),  // T09
            ( new Vector3( 0f,  1.5f,  0f), new Vector3(-1,  1,  0).normalized ),  // T10
            ( new Vector3( 0f,  1.5f,  0f), new Vector3( 1,  1,  0).normalized ),  // T11
            // ── −Y group ───────────────────────────────────────────────────────
            ( new Vector3( 0f, -1.5f,  0f), new Vector3( 0, -1, -1).normalized ),  // T12
            ( new Vector3( 0f, -1.5f,  0f), new Vector3( 0, -1,  1).normalized ),  // T13
            ( new Vector3( 0f, -1.5f,  0f), new Vector3(-1, -1,  0).normalized ),  // T14
            ( new Vector3( 0f, -1.5f,  0f), new Vector3( 1, -1,  0).normalized ),  // T15
        };

        var gasPuffMat = CreateGasPuffMaterial();

        var transforms = new Transform[thrusterData.Length];
        for (int i = 0; i < thrusterData.Length; i++)
        {
            var go = new GameObject($"Thruster_{i:D2}");
            go.transform.SetParent(transform, false);
            go.transform.localPosition = thrusterData[i].pos;
            go.transform.localRotation = Quaternion.LookRotation(thrusterData[i].exhaustDir);
            transforms[i] = go.transform;

            // Plume child: rotated +90° on X so its local +Y (PS default emit axis) aligns with parent +Z (exhaust).
            var plumeGo = new GameObject("Plume");
            plumeGo.transform.SetParent(go.transform, false);
            plumeGo.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
            ConfigureThrusterPlume(plumeGo.AddComponent<ParticleSystem>(), gasPuffMat);
        }

        thrusterTransforms = transforms;
        UnityEditor.EditorUtility.SetDirty(this);
        Debug.Log($"[RCSModel] Created {thrusterData.Length} thruster GameObjects.\n" +
                  "Drag each Thruster_XX to its visual position on the Dragon model.\n" +
                  "Do NOT change local rotations — exhaust cant angles are pre-set for 6-DOF.\n" +
                  "Each thruster has a Plume child ParticleSystem — add ThrusterPlumes to this GameObject to drive them.");
    }

    // Shared soft-alpha material for all plume puffs — see GasPuff.shader.
    // Must be a real saved asset (not HideAndDontSave): this runs at edit time and the
    // resulting material gets assigned into the scene's ParticleSystemRenderer, so it has
    // to survive serialization — an unsaved object reference goes stale on reload and
    // Unity shows the pink "missing material" placeholder instead.
    const string GasPuffMatPath = "Assets/GasPuff.mat";

    static Material CreateGasPuffMaterial()
    {
        var existing = UnityEditor.AssetDatabase.LoadAssetAtPath<Material>(GasPuffMatPath);
        if (existing != null)
            return existing;

        var shader = Shader.Find("Custom/GasPuff");
        if (shader == null)
        {
            Debug.LogWarning("[RCSModel] Custom/GasPuff shader not found — falling back to Default-Particle material. " +
                              "Ensure GasPuff.shader is in the Assets folder.");
            return UnityEditor.AssetDatabase.GetBuiltinExtraResource<Material>("Default-Particle.mat");
        }

        var mat = new Material(shader);
        UnityEditor.AssetDatabase.CreateAsset(mat, GasPuffMatPath);
        UnityEditor.AssetDatabase.SaveAssets();
        return mat;
    }

    static void ConfigureThrusterPlume(ParticleSystem ps, Material gasPuffMat)
    {
        var main = ps.main;
        // No atmosphere out here: no drag, no turbulence (see noise module below) — puffs
        // keep roughly their launch velocity all the way out, they just expand as they go.
        main.startLifetime   = new ParticleSystem.MinMaxCurve(2.0f, 2.5f);
        main.startSpeed      = new ParticleSystem.MinMaxCurve(200f, 250f);
        main.startSize       = new ParticleSystem.MinMaxCurve(2.5f, 3f); // puff diameter at birth
        main.startRotation   = new ParticleSystem.MinMaxCurve(0f, 360f * Mathf.Deg2Rad); // vary billboard spin so puffs don't look identical
        main.startColor      = new ParticleSystem.MinMaxGradient(
            new Color(0.92f, 0.95f, 1f, 0.55f),
            new Color(1f,    1f,    1f, 0.4f));
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.simulationSpeed = 1f;
        main.maxParticles    = 1500;
        main.loop            = true;

        var emission = ps.emission;
        emission.rateOverTime = 0f;

        // Wider cone than the old beam — gas expands as it leaves the nozzle.
        var shape = ps.shape;
        shape.enabled   = true;
        shape.shapeType = ParticleSystemShapeType.Cone;
        shape.angle     = 12f;
        shape.radius    = 0.05f;
        shape.rotation  = new Vector3(-90f, 0f, 0f);

        // Fade in quickly, dwell, then dissolve — reads as gas dispersing rather than a hard cutoff.
        var col = ps.colorOverLifetime;
        col.enabled = true;
        var gradient = new Gradient();
        gradient.SetKeys(
            new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(new Color(0.85f, 0.9f, 0.95f), 1f) },
            new[] { new GradientAlphaKey(0f, 0f), new GradientAlphaKey(1f, 0.15f), new GradientAlphaKey(0f, 1f) }
        );
        col.color = new ParticleSystem.MinMaxGradient(gradient);

        // Puffs grow as they drift — the core "gas cloud expanding" look.
        var size = ps.sizeOverLifetime;
        size.enabled = true;
        size.size = new ParticleSystem.MinMaxCurve(1f, AnimationCurve.Linear(0f, 0.6f, 1f, 2.2f));

        // No drag, no noise/turbulence — this is vacuum. Free expansion only (size curve above).
        var limit = ps.limitVelocityOverLifetime;
        limit.enabled = false;

        var noise = ps.noise;
        noise.enabled = false;

        // Billboard puffs using the procedural soft-round GasPuff shader (no texture asset needed).
        var r = ps.GetComponent<ParticleSystemRenderer>();
        r.renderMode     = ParticleSystemRenderMode.Billboard;
        r.sharedMaterial = gasPuffMat;

        ps.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
    }

    [ContextMenu("Add Plumes to Existing Thrusters")]
    void AddPlumesToExistingThrusters()
    {
        int count = 0;
        var gasPuffMat = CreateGasPuffMaterial();

        foreach (Transform child in transform)
        {
            if (!child.name.StartsWith("Thruster_")) continue;

            // Always replace so re-running this menu picks up the latest settings.
            var existing = child.Find("Plume");
            if (existing != null)
                DestroyImmediate(existing.gameObject);

            var plumeGo = new GameObject("Plume");
            plumeGo.transform.SetParent(child, false);
            plumeGo.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
            ConfigureThrusterPlume(plumeGo.AddComponent<ParticleSystem>(), gasPuffMat);
            count++;
        }

        UnityEditor.EditorUtility.SetDirty(this);
        Debug.Log($"[RCSModel] Add Plumes: {count} plumes created. " +
                  "Add ThrusterPlumes to this GameObject to drive them.");
    }
#endif
}
