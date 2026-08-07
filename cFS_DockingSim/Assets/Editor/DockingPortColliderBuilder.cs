#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

/// <summary>
/// Precise docking-port colliders for the ISS_Original model (unlike ISS_Reduced, which
/// StationColliderBuilder.cs had to approximate with hand-placed primitives because it was
/// one fused ~1.67M-vert mesh -- ISS_Original ships PMA2/IDA2/IDA2_Details_Petals as
/// separate, already-isolated submeshes, per ModelHierarchyDumper's "Dump ISS_Original
/// PMA2+IDA2 Subtree").
///
/// Adds a real (non-convex) MeshCollider directly on each target node, using that node's
/// own MeshFilter.sharedMesh -- same pattern as HullColliderBuilder.cs, except convex isn't
/// required here: TargetVehicle's Rigidbody is kinematic (m_IsKinematic: 1 in Scene2.unity),
/// and PhysX only forces convex-only MeshColliders under a NON-kinematic Rigidbody. That
/// means the actual concave petal/flange geometry is preserved instead of being puffed out
/// to a convex hull, which matters here specifically because the docking mechanism's contact
/// shape IS the concave detail.
///
/// Deliberately scoped to just PMA2 + IDA2 + IDA2_Details_Petals for now (the precise
/// dev-testing target) -- the rest of the station still uses StationColliderBuilder.cs's
/// generalized primitives, to be revisited later.
/// </summary>
static class DockingPortColliderBuilder
{
    [MenuItem("Tools/Docking/Add Precise PMA2+IDA2 Colliders")]
    static void AddColliders()
    {
        var target = GameObject.Find("TargetVehicle");
        if (target == null)
        {
            Debug.LogError("[DockingPortColliderBuilder] Could not find 'TargetVehicle' in the active scene.");
            return;
        }

        Transform pma2 = FindChildRecursive(target.transform, "PMA2");
        if (pma2 == null)
        {
            Debug.LogError("[DockingPortColliderBuilder] Could not find 'PMA2' under TargetVehicle. " +
                            "Check ISS_Original is present/active in the scene.");
            return;
        }

        Transform ida2 = FindChildRecursive(pma2, "IDA2");
        Transform petals = ida2 != null ? FindChildRecursive(ida2, "IDA2_Details_Petals") : null;

        int added = 0;
        added += AddMeshCollider(pma2, "PMA2");
        if (ida2 != null) added += AddMeshCollider(ida2, "IDA2");
        else Debug.LogWarning("[DockingPortColliderBuilder] Could not find 'IDA2' under PMA2 — skipped.");
        if (petals != null) added += AddMeshCollider(petals, "IDA2_Details_Petals");
        else Debug.LogWarning("[DockingPortColliderBuilder] Could not find 'IDA2_Details_Petals' under IDA2 — skipped.");

        EditorUtility.SetDirty(target);
        EditorSceneManager.MarkSceneDirty(target.scene);

        Debug.Log($"[DockingPortColliderBuilder] Added/updated {added} precise MeshCollider(s) " +
                  "(non-convex — TargetVehicle's Rigidbody is kinematic, so this is allowed) on " +
                  "PMA2/IDA2/IDA2_Details_Petals. Note StationColliderBuilder's 'PortReceptacle' " +
                  "placeholder capsule may now overlap these — worth checking whether to remove it " +
                  "once these are verified in Play mode.");
    }

    static int AddMeshCollider(Transform t, string label)
    {
        var mf = t.GetComponent<MeshFilter>();
        if (mf == null || mf.sharedMesh == null)
        {
            Debug.LogWarning($"[DockingPortColliderBuilder] '{label}' has no MeshFilter/sharedMesh — skipped.", t);
            return 0;
        }

        var collider = t.GetComponent<MeshCollider>();
        if (collider == null) collider = Undo.AddComponent<MeshCollider>(t.gameObject);

        collider.sharedMesh = mf.sharedMesh;
        collider.convex     = false; // real concave detail preserved — kinematic ancestor allows this
        collider.isTrigger  = false;

        EditorUtility.SetDirty(t.gameObject);
        Debug.Log($"[DockingPortColliderBuilder] '{label}': MeshCollider <- {mf.sharedMesh.vertexCount} verts (non-convex).");
        return 1;
    }

    static Transform FindChildRecursive(Transform root, string name)
    {
        foreach (Transform child in root)
        {
            if (child.name == name) return child;
            var found = FindChildRecursive(child, name);
            if (found != null) return found;
        }
        return null;
    }
}
#endif
