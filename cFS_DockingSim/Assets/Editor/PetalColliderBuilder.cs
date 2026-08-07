#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

/// <summary>
/// Replaces the compliant docking-ring rig (separate DockingRingPhysics Rigidbody +
/// ConfigurableJoint + puppeted visuals, formerly DockingRingSetup.cs / DockingRingVisualDriver.cs,
/// both deleted -- migration already run, this is just the collider setup now) with a plain
/// rigid setup: convex MeshColliders directly on
/// the real Petal_Rim/Petals.NNN nodes in the Dragon model, moving rigidly with
/// ChaserVehicle like the rest of the hull. No spring/damper "give", no separate physics
/// body, no manual self-collision bookkeeping between a proxy and the hull -- just real
/// geometry colliding with real geometry, letting PhysX's own contact response do the work.
/// Traded away: the ring no longer physically compresses on contact, and the strut meshes
/// (previously stretched frame-by-frame to visually follow the compliant ring) now stay
/// static in their rest pose.
///
/// Convex is still required here (not a choice): Petal_Rim/Petals.NNN are descendants of
/// ChaserVehicle's non-kinematic Rigidbody, and PhysX only allows convex MeshColliders under
/// a non-kinematic body. One hull per petal (rather than one over the whole assembly)
/// approximates the true concave interlocking shape via compound convex decomposition.
///
/// Deliberately does NOT add a collider to Petal_Rim itself. Petal_Rim is a ring (a frame
/// with a hole in the middle) -- a convex hull of a ring fills in the hole, turning it into
/// a solid disc. That disc was blocking dead-on approach like a flat wall before the petal
/// tabs (which don't have this problem individually) ever got a chance to interlock with the
/// station's petals. The station's IDA2 ring doesn't have this issue since it's non-convex
/// there (kinematic body, real hole preserved) -- this is specific to the capsule side.
///
/// Safe to re-run: skips the DockingRingPhysics/DockingRingVisualDriver cleanup if already
/// done, and updates colliders in place rather than duplicating them.
/// </summary>
static class PetalColliderBuilder
{
    [MenuItem("Tools/Docking/Simplify Docking Ring To Rigid Colliders")]
    static void Simplify()
    {
        var chaser = GameObject.Find("ChaserVehicle");
        if (chaser == null) { Debug.LogError("[PetalColliderBuilder] Could not find 'ChaserVehicle'."); return; }

        Transform petalRim = FindChildRecursive(chaser.transform, "Petal_Rim");
        if (petalRim == null)
        { Debug.LogError("[PetalColliderBuilder] Could not find 'Petal_Rim' under ChaserVehicle."); return; }

        // --- Cleanup: remove the compliant rig, if it's somehow come back (DockingRingSetup.cs
        // and DockingRingVisualDriver.cs were deleted after the migration, so there's no longer
        // a component type to search for here -- only the GameObject-by-name part still applies) ---

        GameObject ringGo = GameObject.Find("DockingRingPhysics");
        if (ringGo != null)
        {
            Debug.Log("[PetalColliderBuilder] Deleting 'DockingRingPhysics' (Rigidbody/Joint/placeholder " +
                      "capsule/proxy petal colliders) — superseded by rigid colliders on the real mesh.");
            Undo.DestroyObjectImmediate(ringGo);
        }

        // Remove Petal_Rim's own collider if an earlier run added one (see class remarks —
        // it fills in the ring's hole and blocks dead-on approach like a flat disc).
        var staleRimCollider = petalRim.GetComponent<MeshCollider>();
        if (staleRimCollider != null)
        {
            Debug.Log("[PetalColliderBuilder] Removing 'Petal_Rim's own MeshCollider — a convex hull of " +
                      "a ring fills in the hole, which was blocking approach like a solid disc.");
            Undo.DestroyObjectImmediate(staleRimCollider);
        }

        // --- Setup: convex colliders on the individual petal tabs only (not the ring itself) ---

        int added = 0;

        foreach (Transform petal in petalRim)
        {
            if (!petal.name.StartsWith("Petals")) continue;
            added += AddConvexCollider(petal, petal.name);
        }

        EditorUtility.SetDirty(chaser);
        EditorSceneManager.MarkSceneDirty(chaser.scene);

        Debug.Log($"[PetalColliderBuilder] Added/updated {added} convex MeshCollider(s) directly on the ring " +
                  "(rigidly attached to ChaserVehicle now — no separate physics body). Save the scene, then test in Play mode.");
    }

    static int AddConvexCollider(Transform t, string label)
    {
        var mf = t.GetComponent<MeshFilter>();
        if (mf == null || mf.sharedMesh == null)
        {
            Debug.LogWarning($"[PetalColliderBuilder] '{label}' has no MeshFilter/sharedMesh — skipped.", t);
            return 0;
        }

        var collider = t.GetComponent<MeshCollider>();
        if (collider == null) collider = Undo.AddComponent<MeshCollider>(t.gameObject);
        collider.sharedMesh = mf.sharedMesh;
        collider.convex     = true; // required — descendant of ChaserVehicle's non-kinematic Rigidbody
        collider.isTrigger  = false;

        EditorUtility.SetDirty(t.gameObject);
        Debug.Log($"[PetalColliderBuilder] '{label}': convex MeshCollider <- {mf.sharedMesh.vertexCount} verts.");
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
