#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

/// <summary>
/// Adds the capsule hull collision shape used for physical docking contact.
///
/// REVISED again: the nosecone panel (previously fused into 'Dragon' in its
/// flipped-open docking pose, which ballooned out any convex hull of the whole
/// mesh) has now been split into its own 'NoseCone' child object in Blender.
/// 'Dragon' is body-only now, so a convex MeshCollider over it follows the real
/// tapered capsule silhouette instead of approximating it with a CapsuleCollider
/// — this replaces the CapsuleCollider placeholder (which lived on
/// ChaserVehicle) with a MeshCollider on 'Dragon' itself. Convex is still
/// required (not a choice): this node sits under ChaserVehicle's non-kinematic
/// Rigidbody, and PhysX only allows convex MeshColliders there. The open
/// NoseCone panel itself still isn't collided — add it separately later if
/// that's wanted.
/// </summary>
static class HullColliderBuilder
{
    [MenuItem("Tools/Docking/Add Capsule Hull Collider")]
    static void AddCapsuleHullCollider()
    {
        var chaser = GameObject.Find("ChaserVehicle");
        if (chaser == null)
        {
            Debug.LogError("[HullColliderBuilder] Could not find 'ChaserVehicle' in the active scene.");
            return;
        }

        // Remove the CapsuleCollider placeholder from the previous approach — it lived directly
        // on ChaserVehicle.
        var staleCapsule = chaser.GetComponent<CapsuleCollider>();
        if (staleCapsule != null)
        {
            Undo.DestroyObjectImmediate(staleCapsule);
            Debug.Log("[HullColliderBuilder] Removed placeholder CapsuleCollider from 'ChaserVehicle' " +
                      "(superseded by convex MeshCollider on 'Dragon' now that NoseCone is split out).");
        }

        Transform dragon = FindChildRecursive(chaser.transform, "Dragon");
        if (dragon == null)
        {
            Debug.LogError("[HullColliderBuilder] Could not find 'Dragon' under ChaserVehicle. " +
                            "Check the Dragon model hierarchy hasn't been renamed.");
            return;
        }

        var meshFilter = dragon.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            Debug.LogError("[HullColliderBuilder] 'Dragon' has no MeshFilter/sharedMesh.", dragon);
            return;
        }

        var collider = dragon.GetComponent<MeshCollider>();
        if (collider == null)
            collider = Undo.AddComponent<MeshCollider>(dragon.gameObject);

        collider.sharedMesh = meshFilter.sharedMesh;
        collider.convex     = true; // required — non-kinematic Rigidbody ancestor
        collider.isTrigger  = false;

        EditorUtility.SetDirty(dragon.gameObject);
        EditorSceneManager.MarkSceneDirty(dragon.gameObject.scene);

        Debug.Log($"[HullColliderBuilder] Added convex MeshCollider to 'Dragon' " +
                  $"({meshFilter.sharedMesh.vertexCount} verts -> convex hull, nosecone-free). " +
                  "Save the scene to keep it.");
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
