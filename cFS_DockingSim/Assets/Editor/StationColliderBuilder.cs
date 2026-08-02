#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

/// <summary>
/// Creates placeholder primitive colliders for station-side docking contact.
/// ISS_Reduced.fbx turned out to be a single ~1.67M-vertex mesh with no
/// isolated port submesh and far too dense for any MeshCollider (convex or
/// not), so this uses hand-placed primitives instead — same approach as the
/// rest of the hull. Sizes/positions below are rough starting guesses; this
/// tool has no visual access to the rendered model, so resize/reposition
/// these in the Scene view against the actual ISS_Reduced mesh afterward.
/// Re-running this menu item updates the existing colliders in place rather
/// than duplicating them, so it's safe to re-run after manual tweaks to
/// other fields.
/// </summary>
static class StationColliderBuilder
{
    [MenuItem("Tools/Docking/Add Station Colliders (Placeholder)")]
    static void AddStationColliders()
    {
        var target = GameObject.Find("TargetVehicle");
        if (target == null)
        {
            Debug.LogError("[StationColliderBuilder] Could not find 'TargetVehicle' in the active scene.");
            return;
        }

        Transform targetPort = FindChildRecursive(target.transform, "TargetPort");
        if (targetPort == null)
        {
            Debug.LogError("[StationColliderBuilder] Could not find 'TargetPort' under TargetVehicle.");
            return;
        }

        // Parent for the new colliders — a sibling of ISS/ISS_Reduced, never nested inside
        // either. ISSModelSwitcher.cs SetActive-toggles those two by name per machine;
        // anything nested inside one would silently vanish depending on which is active.
        Transform rig = target.transform.Find("StationColliders");
        if (rig == null)
        {
            var rigGo = new GameObject("StationColliders");
            Undo.RegisterCreatedObjectUndo(rigGo, "Create StationColliders");
            rigGo.transform.SetParent(target.transform, false);
            rig = rigGo.transform;
        }

        // Port receptacle: capsule oriented along TargetPort's forward (docking) axis —
        // local +Z, matching the project's forward-axis convention (RCSModel.cs). Rough
        // placeholder for the funnel/receptacle the ring makes contact with.
        // Centered *behind* TargetPort by half its height, so its front face sits at the
        // port rather than the whole capsule straddling it and poking into open approach
        // space (that was the "invisible wall before the docking port" bug — a capsule
        // centered exactly on TargetPort pokes forward of the actual port face by design).
        // Recess distance fit from two real Play-mode measurements rather than another guess:
        // recess=0.0 -> contact at RelativeNav.range=0.58m; recess=0.4 -> range=0.41m. That's a
        // slope of ~0.425m range reduction per 1m of added recess (non-1:1, likely because range
        // is measured ChaserPort-to-TargetPort while contact happens at the ring/capsule surfaces,
        // which aren't coincident with those reference points). Extrapolating to target
        // range~0.175m (just above DockingDetector's existing 0.15m "docked" threshold) gives
        // ~0.95m. Re-measure after this and adjust if the fit doesn't hold exactly.
        float portReceptacleHeight = 1.5f;
        float portReceptacleRecess = 0.95f;
        Vector3 portReceptacleCenter = targetPort.position -
                                        targetPort.forward * (portReceptacleHeight * 0.5f + portReceptacleRecess);
        AddOrUpdateCapsule(rig, "PortReceptacle", portReceptacleCenter, targetPort.rotation,
                           radius: 1.0f, height: portReceptacleHeight);

        // Generic hull: one box extending back from the port along -forward, approximating
        // the module body behind it. Front face flush with the port (same centering fix as
        // PortReceptacle above), sized generously — Dump Station Hull Bounds showed the real
        // ISS model is 108x30.6x73.4m, so the original 4x4x8m placeholder (roughly capsule-scale)
        // was off by more than an order of magnitude, letting the capsule fly straight through.
        // This is still just an approximate local-module proxy, not a fit to the real structure —
        // shrink/reshape once visible in-Editor if it overlaps something it shouldn't.
        float hullDepth = 20f;
        Vector3 hullCenter = targetPort.position - targetPort.forward * (hullDepth * 0.5f);
        AddOrUpdateBox(rig, "GenericHull", hullCenter, targetPort.rotation, size: new Vector3(12f, 12f, hullDepth));

        EditorUtility.SetDirty(rig.gameObject);
        EditorSceneManager.MarkSceneDirty(target.scene);

        Debug.Log("[StationColliderBuilder] Added/updated 'PortReceptacle' (capsule) and 'GenericHull' (box) " +
                  "under TargetVehicle/StationColliders. These are placeholder sizes — resize/reposition in " +
                  "the Scene view against the real ISS_Reduced mesh, then save the scene.");
    }

    static void AddOrUpdateCapsule(Transform parent, string name, Vector3 worldPos, Quaternion worldRot,
                                    float radius, float height)
    {
        GameObject go = GetOrCreateChild(parent, name);
        go.transform.SetPositionAndRotation(worldPos, worldRot);

        var cc = go.GetComponent<CapsuleCollider>();
        if (cc == null) cc = Undo.AddComponent<CapsuleCollider>(go);
        cc.direction = 2; // Z-axis, so the capsule's long axis follows local +Z (forward)
        cc.radius    = radius;
        cc.height    = height;
        cc.isTrigger = false;

        EditorUtility.SetDirty(go);
    }

    static void AddOrUpdateBox(Transform parent, string name, Vector3 worldPos, Quaternion worldRot, Vector3 size)
    {
        GameObject go = GetOrCreateChild(parent, name);
        go.transform.SetPositionAndRotation(worldPos, worldRot);

        var bc = go.GetComponent<BoxCollider>();
        if (bc == null) bc = Undo.AddComponent<BoxCollider>(go);
        bc.size      = size;
        bc.isTrigger = false;

        EditorUtility.SetDirty(go);
    }

    static GameObject GetOrCreateChild(Transform parent, string name)
    {
        Transform existing = parent.Find(name);
        if (existing != null) return existing.gameObject;

        var go = new GameObject(name);
        Undo.RegisterCreatedObjectUndo(go, $"Create {name}");
        go.transform.SetParent(parent, false);
        return go;
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
