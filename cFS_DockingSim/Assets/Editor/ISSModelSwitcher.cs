#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Per-machine toggle between the full ISS and ISS_Reduced GameObjects.
/// The preference is stored in EditorPrefs, which is local to each machine,
/// so your main PC and laptop can each have their own default without affecting git.
/// The setting re-applies automatically every time Unity loads or scripts recompile.
/// </summary>
[InitializeOnLoad]
public static class ISSModelSwitcher
{
    const string PrefKey     = "cFS.ISSModelSwitcher.UseReduced";
    const string MenuFull    = "Tools/ISS Model/Full ISS — Main PC";
    const string MenuReduced = "Tools/ISS Model/Reduced ISS — Laptop";
    const string FullName    = "ISS";
    const string ReducedName = "ISS_Reduced";

    static bool UseReduced => EditorPrefs.GetBool(PrefKey, false);

    static ISSModelSwitcher()
    {
        // delayCall defers until the scene is actually loaded.
        EditorApplication.delayCall += Apply;
    }

    [MenuItem(MenuFull)]
    static void SetFull()
    {
        EditorPrefs.SetBool(PrefKey, false);
        Apply();
    }

    [MenuItem(MenuReduced)]
    static void SetReduced()
    {
        EditorPrefs.SetBool(PrefKey, true);
        Apply();
    }

    // Validators run before the menu opens — used here to draw the checkmark.
    [MenuItem(MenuFull, validate = true)]
    static bool ValidateFull()
    {
        Menu.SetChecked(MenuFull,    !UseReduced);
        Menu.SetChecked(MenuReduced,  UseReduced);
        return true;
    }

    [MenuItem(MenuReduced, validate = true)]
    static bool ValidateReduced() => true;

    static void Apply()
    {
        bool reduced = UseReduced;
        if (!SceneManager.GetActiveScene().isLoaded) return;

        // FindObjectsByType with FindObjectsInactive.Include finds objects anywhere in the
        // hierarchy regardless of active state — needed since one model is always disabled.
        var all = Object.FindObjectsByType<GameObject>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);

        bool foundFull    = false;
        bool foundReduced = false;

        foreach (GameObject go in all)
        {
            if (go.name == FullName)    { go.SetActive(!reduced); foundFull    = true; }
            else if (go.name == ReducedName) { go.SetActive(reduced);  foundReduced = true; }
        }

        if (!foundFull)
            Debug.LogWarning($"[ISSModelSwitcher] Could not find a GameObject named '{FullName}' in the scene.");
        if (!foundReduced)
            Debug.LogWarning($"[ISSModelSwitcher] Could not find a GameObject named '{ReducedName}' in the scene.");

        if (foundFull || foundReduced)
            Debug.Log($"[ISSModelSwitcher] Active model: {(reduced ? ReducedName : FullName)}");
    }
}
#endif
