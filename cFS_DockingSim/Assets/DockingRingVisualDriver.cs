using UnityEngine;

/// <summary>
/// Puppets the visual docking ring (Docking_Port, carrying Petal_Rim/Petals as
/// children) and the 6 strut meshes (Strut1..Strut6) off DockingRingPhysics's
/// live simulated transform every frame. Mirrors ThrusterPlumes.cs's pattern:
/// per-index rest state captured once at Start, driven every LateUpdate from a
/// live physics source. No colliders/rigidbodies on the visual objects
/// themselves — DockingRingPhysics (a separate, invisible object created by
/// the "Create Docking Ring Physics Rig" editor tool) is what's actually
/// simulated; this only makes the art follow it.
///
/// Struts: each strut's pivot sits at the SAME point as Dragon's own origin
/// (not at its own base) — a side effect of the "Apply Transform" fix that
/// resolved an earlier Blender/FBX export offset bug — so this can't simply
/// move each strut's Transform.position to its base point (the mesh data
/// doesn't know about that reassignment and would render at the wrong
/// location). Instead, each strut's fixed base and moving ring-side tip are
/// located from its own mesh bounds once at Start, and every frame a new
/// local transform is solved so the mesh's base point stays fixed while its
/// tip point tracks the ring (see UpdateStrut). Scale is uniform rather than
/// aligned to just the strut's long axis, so it gets very slightly
/// fatter/thinner as it stretches/compresses — an acceptable simplification
/// given the joint's travel range is small (see DockingRingSetup's linear/
/// angular limits).
/// </summary>
public class DockingRingVisualDriver : MonoBehaviour
{
    [Header("Physics source")]
    [Tooltip("The simulated ring proxy created by DockingRingSetup (Tools > Docking menu).")]
    public Transform ringPhysics;

    [Header("Visual targets (children of Dragon)")]
    [Tooltip("The visual ring node — Docking_Port, carrying Petal_Rim/Petals.")]
    public Transform dockingPortVisual;
    [Tooltip("Strut1..Strut6, in any order — each is matched to its own base/tip independently.")]
    public Transform[] struts;
    [Tooltip("Dragon — the fixed reference frame the struts' bases are anchored to.")]
    public Transform dragon;

    private Vector3    _ringRestPosition;
    private Quaternion _ringRestRotation;

    private Vector3[] _strutBaseDragonLocal; // fixed, hull-side, in DRAGON-LOCAL space — see note below
    private Vector3[] _strutRestTipWorld;    // ring-side, AT REST — captured once
    private Vector3[] _strutMeshLocalBase;   // base point in raw mesh-local space
    private float[]   _strutRestLength;
    private Vector3[] _strutRestAxis;        // rest direction, in mesh-local space

    void Start()
    {
        if (ringPhysics == null || dockingPortVisual == null || dragon == null || struts == null)
        {
            Debug.LogError("[DockingRingVisualDriver] Missing references — assign ringPhysics, " +
                            "dockingPortVisual, dragon, and struts in the Inspector.", this);
            enabled = false;
            return;
        }

        _ringRestPosition = ringPhysics.position;
        _ringRestRotation = ringPhysics.rotation;

        // The ring's collider sits right next to the capsule's own hull collider and is
        // mechanically joined to it via ConfigurableJoint — without this, PhysX treats them as
        // two separate overlapping solid bodies and shoves them apart the instant Play mode
        // starts. Physics.IgnoreCollision doesn't persist across Play sessions, so this has to
        // run here (Start, every time Play begins) rather than once in the editor setup tool.
        var ringCollider = ringPhysics.GetComponent<Collider>();
        var hullCollider  = dragon.GetComponent<Collider>();
        if (ringCollider != null && hullCollider != null)
            Physics.IgnoreCollision(ringCollider, hullCollider, true);

        int n = struts.Length;
        _strutBaseDragonLocal = new Vector3[n];
        _strutRestTipWorld    = new Vector3[n];
        _strutMeshLocalBase   = new Vector3[n];
        _strutRestLength      = new float[n];
        _strutRestAxis        = new Vector3[n];

        for (int i = 0; i < n; i++)
        {
            Transform strut = struts[i];
            if (strut == null) continue;

            var mf = strut.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null)
            {
                Debug.LogWarning($"[DockingRingVisualDriver] {strut.name} has no mesh — skipping.", strut);
                continue;
            }

            FindMeshEndpoints(mf.sharedMesh, out Vector3 endA, out Vector3 endB);

            // World-space rest positions of both ends, using the strut's transform as it is right
            // now — this must run before anything has perturbed it from its authored rest pose.
            Vector3 worldA = strut.TransformPoint(endA);
            Vector3 worldB = strut.TransformPoint(endB);

            // Base (hull-side, fixed) is whichever end is farther from the ring; tip (moving) is
            // whichever is closer to it.
            float distA = Vector3.Distance(worldA, _ringRestPosition);
            float distB = Vector3.Distance(worldB, _ringRestPosition);
            bool  aIsBase = distA > distB;

            Vector3 baseWorld = aIsBase ? worldA : worldB;
            Vector3 tipWorld  = aIsBase ? worldB : worldA;
            Vector3 baseLocal = aIsBase ? endA : endB;
            Vector3 tipLocal  = aIsBase ? endB : endA;

            // Stored in DRAGON-LOCAL space, not world space: the base is only fixed relative to
            // the (moving) capsule, not to an absolute point in the world. A frozen world-space
            // point would fall further behind Dragon's actual position the farther the ship
            // travels, making the computed base-to-tip length (and so the strut's scale) grow
            // without bound with travel distance — exactly the "struts growing" bug this fixes.
            _strutBaseDragonLocal[i] = dragon.InverseTransformPoint(baseWorld);
            _strutRestTipWorld[i]    = tipWorld;
            _strutMeshLocalBase[i]   = baseLocal;
            _strutRestLength[i]      = Vector3.Distance(baseLocal, tipLocal);
            _strutRestAxis[i]        = _strutRestLength[i] > 1e-6f
                ? (tipLocal - baseLocal) / _strutRestLength[i]
                : Vector3.forward;
        }
    }

    // Finds the two extreme points of a mesh along its longest bounding-box axis, at the
    // cross-sectional center of the other two axes — a rod-shaped mesh's two ends.
    static void FindMeshEndpoints(Mesh mesh, out Vector3 endA, out Vector3 endB)
    {
        Bounds  b    = mesh.bounds;
        Vector3 size = b.size;
        int longAxis = 0;
        if (size.y > size[longAxis]) longAxis = 1;
        if (size.z > size[longAxis]) longAxis = 2;

        endA = b.center;
        endB = b.center;
        endA[longAxis] = b.min[longAxis];
        endB[longAxis] = b.max[longAxis];
    }

    void LateUpdate()
    {
        if (ringPhysics == null) return;

        // Puppet the visual ring rigidly onto the physics proxy's live pose.
        dockingPortVisual.SetPositionAndRotation(ringPhysics.position, ringPhysics.rotation);

        Quaternion ringRotDelta = ringPhysics.rotation * Quaternion.Inverse(_ringRestRotation);

        for (int i = 0; i < struts.Length; i++)
        {
            Transform strut = struts[i];
            if (strut == null || _strutRestLength[i] <= 1e-6f) continue;
            UpdateStrut(strut, i, ringRotDelta);
        }
    }

    void UpdateStrut(Transform strut, int i, Quaternion ringRotDelta)
    {
        // Current world-space tip: the ring-side attachment point rotates/translates rigidly
        // with the ring, about the ring's own rest position.
        Vector3 currentTipWorld = ringPhysics.position +
                                   ringRotDelta * (_strutRestTipWorld[i] - _ringRestPosition);

        Vector3 dLocalBase = _strutBaseDragonLocal[i]; // constant — already in Dragon-local space
        Vector3 dLocalTip  = dragon.InverseTransformPoint(currentTipWorld);

        Vector3 newVec    = dLocalTip - dLocalBase;
        float   lengthNew = newVec.magnitude;
        if (lengthNew <= 1e-6f) return; // degenerate — leave the strut where it was last frame

        Vector3    axisNew = newVec / lengthNew;
        float      s       = lengthNew / _strutRestLength[i];
        Quaternion rot     = Quaternion.FromToRotation(_strutRestAxis[i], axisNew);

        strut.localScale    = Vector3.one * s;
        strut.localRotation = rot;
        strut.localPosition = dLocalBase - rot * (_strutMeshLocalBase[i] * s);
    }
}
