using UnityEngine;

/// <summary>
/// Physical thruster definition — position and thrust direction in body frame.
/// Position is the offset from the vehicle CoM (meters).
/// Direction is the unit thrust vector (OPPOSITE to local +Z of the scene Transform, which is the exhaust direction).
/// When fired, a thruster produces:
///   Force  = direction * throttle_N
///   Torque = AddForceAtPosition handles this via world-space position
/// </summary>
[System.Serializable]
public struct ThrusterDef
{
    public Vector3 position;   // body-frame offset from CoM (m)
    public Vector3 direction;  // body-frame unit thrust vector

    public ThrusterDef(Vector3 pos, Vector3 dir)
    {
        position  = pos;
        direction = dir.normalized;
    }
}

/// <summary>
/// Proportional RCS thruster control for the chaser.
///
/// Each thruster fires at an independently computed force level (0..thrusterForce N)
/// rather than binary on/off at full power.  This is essential for clean 6-DOF
/// translation: the pseudo-inverse allocator assigns fractional throttles whose
/// torques cancel each other — but only if those fractions are actually respected
/// when the forces are applied.  Binary on/off destroys that cancellation and
/// produces the unwanted rotations during lateral translation.
///
/// Throttle sources (in priority order):
///   1. External control    — cFS or ThrusterTestUI via SetWrenchCommand /
///                            SetThrusterCommand.  Expires after burnEndTime.
///   2. Keyboard (WASD etc) — binary; any thruster with a positive pseudo-inverse
///                            allocation fires at full thrusterForce, all others off.
/// </summary>
public class RCSModel : MonoBehaviour
{
    public VehicleState vehicle;

    [Header("Thruster Authority")]
    [Tooltip("Newtons per thruster — must match ThrusterForce in the cFS parameter table.")]
    public float thrusterForce = 10f;

    [Header("Thruster Geometry")]
    [Tooltip("One Transform per physical thruster. Local +Z = exhaust direction (nozzle out). " +
             "Force is applied in the −Z direction at that world position.")]
    [SerializeField] private Transform[] thrusterTransforms;

    // Body-frame ThrusterDef array built from thrusterTransforms at Awake.
    private ThrusterDef[]     _thrusters = new ThrusterDef[0];
    // Per-thruster throttle in Newtons (0..thrusterForce).  Primary command state.
    private float[]           _throttles = new float[0];
    private Rigidbody         _rb;
    private ThrusterAllocator _allocator;

    public ThrusterDef[] GetThrusters() => _thrusters;
    public int ThrusterCount => _thrusters.Length;
    public Transform[] ThrusterTransforms => thrusterTransforms;

    /// <summary>Returns the normalized throttle (0–1) for thruster i. Safe to call before init.</summary>
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

    /// <summary>
    /// Bitmask derived from _throttles for gizmos and UI display.
    /// Bit i is set when thruster i is firing at ≥5% of full power.
    /// </summary>
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

    [Header("Debug")]
    [Tooltip("Suppress all thruster forces (T key). Thrusters still appear active in gizmos.")]
    public bool suppressForces = false;

    private bool  externalControl = false;
    private float burnEndTime     = -1f;

    void Awake()
    {
        // Only pre-allocate here. BuildThrusterArray() is deferred to Start() via coroutine
        // so that VehicleState.Start() has already applied centerOfMassOverride before we
        // compute moment arms for the B matrix.
        _throttles = new float[0];
    }

    void Start()
    {
        StartCoroutine(DelayedInit());
    }

    System.Collections.IEnumerator DelayedInit()
    {
        // Wait one frame so all Start() calls complete — specifically VehicleState.Start(),
        // which applies rb.centerOfMass = centerOfMassOverride. Without this, the B matrix
        // is built against the wrong CoM and torque-cancellation throttles are wrong.
        yield return null;

        if (vehicle != null)
            _rb = vehicle.GetComponent<Rigidbody>();

        BuildThrusterArray();
        _throttles = new float[_thrusters.Length];

        if (_thrusters.Length > 0)
        {
            _allocator = new ThrusterAllocator();
            _allocator.Initialize(_thrusters);
        }
    }

    void BuildThrusterArray()
    {
        if (thrusterTransforms == null || thrusterTransforms.Length == 0)
        {
            _thrusters = new ThrusterDef[0];
            return;
        }

        // The B matrix torque column is r × direction, where r is the moment arm from
        // the CoM to the thruster.  Using transform.origin instead of rb.worldCenterOfMass
        // gives the wrong moment arms when centerOfMassOverride is non-zero, causing the
        // allocator's torque-cancellation throttles to not match what Unity actually applies.
        Vector3 worldCoM = _rb != null
            ? vehicle.transform.TransformPoint(_rb.centerOfMass)
            : transform.position;

#if UNITY_EDITOR
        Vector3 comLocal = transform.InverseTransformPoint(worldCoM);
        Debug.Log($"[RCSModel] Building B-matrix.  CoM in body frame: {comLocal:F3}");
#endif

        _thrusters = new ThrusterDef[thrusterTransforms.Length];
        for (int i = 0; i < thrusterTransforms.Length; i++)
        {
            if (thrusterTransforms[i] == null || !thrusterTransforms[i].gameObject.activeInHierarchy)
            {
                // Zero direction → zero B-matrix column → allocator never picks this thruster.
                _thrusters[i] = new ThrusterDef(Vector3.zero, Vector3.zero);
                continue;
            }
            // Child's local +Z is the exhaust direction; force is in −Z (Newton's 3rd law).
            // Moment arm = thruster world pos − CoM world pos, rotated into body frame.
            Vector3 momentArm = transform.InverseTransformDirection(
                thrusterTransforms[i].position - worldCoM);
            Vector3 localDir = -transform.InverseTransformDirection(thrusterTransforms[i].forward);
            _thrusters[i] = new ThrusterDef(momentArm, localDir);
        }
    }

    // ── Test mode ─────────────────────────────────────────────────────────────
    // Hold backtick (`) to enter test mode.  Number keys 1-9 = T00-T08,
    // 0 = T09, - = T10, = = T11, F1-F4 = T12-T15.  Multiple keys fire simultaneously.
    // Test mode uses binary full-power so you can feel each thruster's individual effect.

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.T)) suppressForces = !suppressForces;

        // ── Keyboard / cFS control ────────────────────────────────────────
        if (externalControl) return;

        // Body-frame: +Z = toward ISS, +X = right, +Y = up.
        Vector3 desiredForce  = Vector3.zero;
        Vector3 desiredTorque = Vector3.zero;

        if (Input.GetKey(KeyCode.W))           desiredForce  += Vector3.forward;
        if (Input.GetKey(KeyCode.S))           desiredForce  -= Vector3.forward;
        if (Input.GetKey(KeyCode.D))           desiredForce  += Vector3.right;
        if (Input.GetKey(KeyCode.A))           desiredForce  -= Vector3.right;
        if (Input.GetKey(KeyCode.Space))       desiredForce  += Vector3.up;
        if (Input.GetKey(KeyCode.LeftControl)) desiredForce  -= Vector3.up;
        if (Input.GetKey(KeyCode.R))           desiredTorque += Vector3.right;   // pitch nose up
        if (Input.GetKey(KeyCode.F))           desiredTorque -= Vector3.right;   // pitch nose down
        if (Input.GetKey(KeyCode.E))           desiredTorque += Vector3.up;      // yaw right
        if (Input.GetKey(KeyCode.Q))           desiredTorque -= Vector3.up;      // yaw left
        if (Input.GetKey(KeyCode.Z))           desiredTorque += Vector3.forward; // roll CW
        if (Input.GetKey(KeyCode.X))           desiredTorque -= Vector3.forward; // roll CCW

        EnsureThrottleArray();

        if (desiredForce == Vector3.zero && desiredTorque == Vector3.zero)
        {
            System.Array.Clear(_throttles, 0, _throttles.Length);
            return;
        }

        if (_allocator == null)
        {
            System.Array.Clear(_throttles, 0, _throttles.Length);
            return;
        }

        float[] raw = _allocator.Allocate(desiredForce, desiredTorque);

        // Clip negative throttles (thrusters can only push).
        float maxRaw = 0f;
        for (int i = 0; i < raw.Length; i++)
        {
            if (raw[i] < 0f) raw[i] = 0f;
            if (raw[i] > maxRaw) maxRaw = raw[i];
        }

        if (maxRaw < 1e-6f)
        {
            System.Array.Clear(_throttles, 0, _throttles.Length);
            return;
        }

        // Binary on/off: fire any thruster the allocator gave a positive allocation.
        // Realistic to Draco hardware — no analog throttle, just full power or off.
        for (int i = 0; i < raw.Length && i < _throttles.Length; i++)
            _throttles[i] = raw[i] > 1e-4f ? thrusterForce : 0f;

        // T00–T03 are orbital retrograde thrusters — never used for docking maneuvers.
        for (int i = 0; i < Mathf.Min(4, _throttles.Length); i++)
            _throttles[i] = 0f;
    }

    void FixedUpdate()
    {
        // Let an external burn coast once its duration expires.
        if (externalControl && Time.fixedTime >= burnEndTime)
            System.Array.Clear(_throttles, 0, _throttles.Length);

        if (vehicle == null || _thrusters.Length == 0 || _rb == null) return;
        if (suppressForces) return;

        for (int i = 0; i < _throttles.Length && i < 32; i++)
        {
            if (_throttles[i] < 1e-6f) continue;
            if (thrusterTransforms[i] == null) continue;
            if (!thrusterTransforms[i].gameObject.activeInHierarchy) continue;

            // Proportional force: respects the pseudo-inverse ratios that cancel torques.
            float   f          = Mathf.Min(_throttles[i], thrusterForce);
            Vector3 worldForce = transform.TransformDirection(_thrusters[i].direction) * f;
            Vector3 worldPos   = thrusterTransforms[i].position;
            _rb.AddForceAtPosition(worldForce, worldPos, ForceMode.Force);
        }
    }

    // ── External control API ──────────────────────────────────────────────────

    /// <summary>
    /// Binary mask command used by ThrusterTestUI and legacy paths.
    /// Selected thrusters fire at full thrusterForce; others are zeroed.
    /// </summary>
    public void SetThrusterCommand(int mask, float duration)
    {
        externalControl = true;
        burnEndTime     = Time.fixedTime + duration;
        EnsureThrottleArray();
        for (int i = 0; i < _throttles.Length && i < 32; i++)
            _throttles[i] = (mask & (1 << i)) != 0 ? thrusterForce : 0f;
    }

    /// <summary>
    /// Proportional wrench command from cFS.  The pseudo-inverse allocates the
    /// desired force/torque across all thrusters and stores fractional throttle
    /// levels so their torques cancel — achieving clean 6-DOF translation.
    /// </summary>
    // Force magnitude below this threshold is treated as a light brake command (T08-T11 only).
    // cFS sends BrakeAccel_Light_mss * VehicleMass (0.136 × 4500 = 612 N) for fine corrections
    // and BrakeAccel_Hard_mss * VehicleMass (0.281 × 4500 = 1265 N) for hard stops.
    // Threshold = midpoint = 938 N.  Update if BrakeAccel values in gnc_param_tbl.c change.
    private const float SoftBrakeThreshold_N = 938f;

    public void SetWrenchCommand(Vector3 force, Vector3 torque, float duration)
    {
        if (_allocator == null || !_allocator.IsReady)
        {
            SetThrusterCommand(0, 0f);
            return;
        }
        if (force.sqrMagnitude < 1e-6f && torque.sqrMagnitude < 1e-6f)
        {
            SetThrusterCommand(0, 0f);
            return;
        }

        externalControl = true;
        burnEndTime     = Time.fixedTime + duration;
        EnsureThrottleArray();

        float[] raw = _allocator.Allocate(force, torque);

        // Binary on/off: consistent with Draco hardware behaviour.
        for (int i = 0; i < raw.Length && i < _throttles.Length; i++)
            _throttles[i] = raw[i] > 1e-4f ? thrusterForce : 0f;

        // T00–T03 are orbital retrograde thrusters — never used for docking maneuvers.
        for (int i = 0; i < Mathf.Min(4, _throttles.Length); i++)
            _throttles[i] = 0f;

        // Brake group selection: cFS sends a smaller force magnitude for gentle corrections
        // (soft brake = T08-T11 only) vs hard stops (all 8 = T08-T15).
        // If purely braking (−Z, no significant lateral demand) and force is below the soft
        // threshold, suppress T12-T15 (the inner brake ring) for a lighter impulse.
        if (force.z < -0.5f && force.z > -SoftBrakeThreshold_N &&
            Mathf.Abs(force.x) < 1f && Mathf.Abs(force.y) < 1f)
        {
            for (int i = 12; i < Mathf.Min(16, _throttles.Length); i++)
                _throttles[i] = 0f;
        }
    }

    public void ClearExternalControl()
    {
        externalControl = false;
        burnEndTime     = -1f;
        EnsureThrottleArray();
        System.Array.Clear(_throttles, 0, _throttles.Length);
    }

    void EnsureThrottleArray()
    {
        if (_throttles == null || _throttles.Length != _thrusters.Length)
            _throttles = new float[_thrusters.Length];
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
            ConfigureThrusterPlume(plumeGo.AddComponent<ParticleSystem>());
        }

        thrusterTransforms = transforms;
        UnityEditor.EditorUtility.SetDirty(this);
        Debug.Log($"[RCSModel] Created {thrusterData.Length} thruster GameObjects.\n" +
                  "Drag each Thruster_XX to its visual position on the Dragon model.\n" +
                  "Do NOT change local rotations — exhaust cant angles are pre-set for 6-DOF.\n" +
                  "Each thruster has a Plume child ParticleSystem — add ThrusterPlumes to this GameObject to drive them.");
    }

    static void ConfigureThrusterPlume(ParticleSystem ps)
    {
        var main = ps.main;
        // Short lifetime + high speed = particles stay close to nozzle and form a tight beam.
        // Cone length ≈ speed × lifetime: 40 m/s × 0.12s ≈ 5m reach.
        main.startLifetime   = new ParticleSystem.MinMaxCurve(0.8f, 1.4f);
        main.startSpeed      = new ParticleSystem.MinMaxCurve(160f, 160f);
        main.startSize       = new ParticleSystem.MinMaxCurve(0.32f, 0.64f); // controls streak WIDTH
        main.startColor      = new ParticleSystem.MinMaxGradient(
            new Color(0.9f, 0.96f, 1f, 1f),
            new Color(1f,   1f,   1f, 0.85f));
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.simulationSpeed = 5f;
        main.maxParticles    = 1000;
        main.loop            = true;

        var emission = ps.emission;
        emission.rateOverTime = 0f;

        // Very tight cone so streaks form a coherent beam, not a wide spray.
        var shape = ps.shape;
        shape.enabled   = true;
        shape.shapeType = ParticleSystemShapeType.Cone;
        shape.angle     = 4f;
        shape.radius    = 0.01f;
        shape.rotation  = new Vector3(-90f, 0f, 0f);

        // Fade out in the back half — bright near nozzle, invisible at the tip.
        var col = ps.colorOverLifetime;
        col.enabled = true;
        var gradient = new Gradient();
        gradient.SetKeys(
            new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(new Color(0.8f, 0.92f, 1f), 1f) },
            new[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(1f, 0.4f), new GradientAlphaKey(0f, 1f) }
        );
        col.color = new ParticleSystem.MinMaxGradient(gradient);

        // No size growth — streaks should stay narrow like rays, not expand into puffs.
        var size = ps.sizeOverLifetime;
        size.enabled = false;

        // No noise — straight rays look like high-pressure gas, turbulence looks like smoke.
        var noise = ps.noise;
        noise.enabled = false;

        // Stretched Billboard: each particle becomes a streak aligned with its velocity.
        // velocityScale stretches it proportional to speed — faster = longer streak.
        // Result: ~80 streaks/sec forming a solid-looking beam, far cheaper than thousands of spheres.
        var r = ps.GetComponent<ParticleSystemRenderer>();
        r.renderMode     = ParticleSystemRenderMode.Stretch;
        r.velocityScale  = 0.06f;
        r.lengthScale    = 5.0f;
        r.sharedMaterial = UnityEditor.AssetDatabase.GetBuiltinExtraResource<Material>("Default-Particle.mat");

        ps.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
    }

    [ContextMenu("Add Plumes to Existing Thrusters")]
    void AddPlumesToExistingThrusters()
    {
        int count = 0;

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
            ConfigureThrusterPlume(plumeGo.AddComponent<ParticleSystem>());
            count++;
        }

        UnityEditor.EditorUtility.SetDirty(this);
        Debug.Log($"[RCSModel] Add Plumes: {count} plumes created. " +
                  "Add ThrusterPlumes to this GameObject to drive them.");
    }
#endif
}
