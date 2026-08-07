#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Text;

/// <summary>
/// Prints a model's full node hierarchy (local transform + mesh info) to the
/// Console. Used to answer questions about imported FBX structure without
/// needing to open/expand the asset by hand — e.g. whether ISS_Reduced has
/// the docking port isolated as its own submesh, where a non-1.0 scale node
/// lives in the Dragon hierarchy, or whether a strut's pivot sits at its
/// hull-side base as intended.
/// </summary>
static class ModelHierarchyDumper
{
    [MenuItem("Tools/Docking/Dump ISS_Reduced Hierarchy")]
    static void DumpISSReduced() => Dump("Assets/Models/ISS/ISS_Reduced.fbx");

    [MenuItem("Tools/Docking/Dump Dragon Hierarchy")]
    static void DumpDragon() => Dump("Assets/Models/Dragon/Dragon_packaged.fbx");

    [MenuItem("Tools/Docking/Dump ISS_Original Hierarchy")]
    static void DumpISSOriginal() => Dump("Assets/Models/ISS_Original/International Space Station (ISS) (D) (IGOAL).fbx");

    const string ISSOriginalPath = "Assets/Models/ISS_Original/International Space Station (ISS) (D) (IGOAL).fbx";

    // Full ISS_Original dump runs to thousands of lines (payloads, ELCs, etc.) -- these
    // dump just the named subtree(s) so the PMA2/IDA2 docking-port structure (and its real
    // parent chain, needed to know whether IDA2 sits under PMA2 or is a separate sibling
    // node) is readable without scrolling past everything else.
    [MenuItem("Tools/Docking/Dump ISS_Original PMA2+IDA2 Subtree")]
    static void DumpPMA2IDA2() => DumpNamedSubtrees(ISSOriginalPath, "PMA2", "IDA2");

    const string DragonPath = "Assets/Models/Dragon/Dragon_packaged.fbx";

    [MenuItem("Tools/Docking/Dump Dragon Docking_Port Subtree")]
    static void DumpDockingPort() => DumpNamedSubtrees(DragonPath, "Docking_Port");

    [MenuItem("Tools/Docking/Dump Dragon Petals Subtree")]
    static void DumpPetals() => DumpNamedSubtrees(DragonPath, "Petals", "Petal_Rim", "petal_bracket1", "petal_bracket2");

    static void DumpNamedSubtrees(string assetPath, params string[] names)
    {
        var asset = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (asset == null)
        {
            Debug.LogError($"[ModelHierarchyDumper] Could not load '{assetPath}'.");
            return;
        }

        var sb = new StringBuilder();
        sb.AppendLine($"[ModelHierarchyDumper] {assetPath} -- named subtrees: {string.Join(", ", names)}");
        sb.AppendLine("name | localPos | localEuler | localScale | mesh(verts) | full path from asset root");

        foreach (var name in names)
        {
            Transform found = FindChildRecursive(asset.transform, name);
            if (found == null)
            {
                sb.AppendLine($"'{name}' not found under asset root.");
                continue;
            }
            sb.AppendLine($"--- {name}  (path: {GetPath(found)}) ---");
            DumpNode(found, sb, 0);
        }

        Debug.Log(sb.ToString());
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

    static string GetPath(Transform t)
    {
        string path = t.name;
        while (t.parent != null)
        {
            t = t.parent;
            path = t.name + "/" + path;
        }
        return path;
    }

    static void Dump(string assetPath)
    {
        var asset = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (asset == null)
        {
            Debug.LogError($"[ModelHierarchyDumper] Could not load '{assetPath}'.");
            return;
        }

        var sb = new StringBuilder();
        sb.AppendLine($"[ModelHierarchyDumper] {assetPath}");
        sb.AppendLine("name | localPos | localEuler | localScale | mesh(verts)");
        DumpNode(asset.transform, sb, 0);
        Debug.Log(sb.ToString());
    }

    static void DumpNode(Transform t, StringBuilder sb, int depth)
    {
        var mf = t.GetComponent<MeshFilter>();
        string meshInfo = mf != null && mf.sharedMesh != null
            ? $" | mesh({mf.sharedMesh.vertexCount}v)"
            : "";

        string scaleFlag = (t.localScale != Vector3.one) ? "  <== NON-UNIT SCALE" : "";

        sb.AppendLine(
            new string(' ', depth * 2) + t.name +
            $" | pos({t.localPosition.x:F3},{t.localPosition.y:F3},{t.localPosition.z:F3})" +
            $" | euler({t.localEulerAngles.x:F1},{t.localEulerAngles.y:F1},{t.localEulerAngles.z:F1})" +
            $" | scale({t.localScale.x:F3},{t.localScale.y:F3},{t.localScale.z:F3})" +
            meshInfo + scaleFlag);

        foreach (Transform child in t)
            DumpNode(child, sb, depth + 1);
    }
}
#endif
