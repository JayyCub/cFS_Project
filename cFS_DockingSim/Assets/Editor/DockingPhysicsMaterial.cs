#if UNITY_EDITOR
using System.IO;
using UnityEngine;
using UnityEditor;

/// <summary>
/// Shared low-friction PhysicMaterial for docking contact surfaces (chaser petals, PMA2/
/// IDA2/IDA2_Details_Petals on the station). Used by PetalColliderBuilder and
/// DockingPortColliderBuilder instead of leaving those colliders on Unity's Default Physic
/// Material (0.6 static/dynamic friction, Average combine) -- that much friction between
/// interlocking petal geometry is enough to overpower the chaser's RCS thrust on contact,
/// which read as the ship "freezing"/getting hard to control the moment the petals touched,
/// rather than sliding against each other as they're meant to during capture.
/// </summary>
static class DockingPhysicsMaterial
{
    const string AssetPath = "Assets/Physics/DockingPetalSlip.physicMaterial";

    public static PhysicsMaterial GetOrCreate()
    {
        var mat = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(AssetPath);
        if (mat != null) return mat;

        var dir = Path.GetDirectoryName(AssetPath).Replace('\\', '/');
        if (!AssetDatabase.IsValidFolder(dir))
            AssetDatabase.CreateFolder("Assets", "Physics");

        mat = new PhysicsMaterial("DockingPetalSlip")
        {
            dynamicFriction = 0.05f,
            staticFriction  = 0.05f,
            bounciness      = 0f,
            frictionCombine = PhysicsMaterialCombine.Minimum,
            bounceCombine   = PhysicsMaterialCombine.Average,
        };
        AssetDatabase.CreateAsset(mat, AssetPath);
        AssetDatabase.SaveAssets();
        Debug.Log($"[DockingPhysicsMaterial] Created '{AssetPath}' (friction 0.05, Minimum combine).");
        return mat;
    }
}
#endif
