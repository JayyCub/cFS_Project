#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

public class TextureAutoAssigner : EditorWindow
{
    private string materialsPath = "Assets/Materials";
    private string texturesPath  = "Assets/Models";
    private Vector2 scrollPos;
    private List<AssignmentPreview> previews = new List<AssignmentPreview>();
    private bool previewGenerated = false;

    private class AssignmentPreview
    {
        public Material material;
        public string   materialPath;
        public Dictionary<string, Texture2D> assignments = new Dictionary<string, Texture2D>();
        public bool apply = true;
    }

    // When a material's name doesn't match its texture filename prefix, map it here.
    // Keys are case-insensitive.
    private static readonly Dictionary<string, string> TexturePrefixOverrides =
        new Dictionary<string, string>(System.StringComparer.OrdinalIgnoreCase)
    {
        // Blender appends .001/.002 to duplicate material slots — strip the suffix
        { "BEAM.001",        "BEAM"          },
        { "PMA.002",         "PMA"           },
        // Material name differs from texture prefix
        { "BishopMaterial",  "Bishop"        },
        { "Node 1",          "Node1"         },
        { "Node 2_3",        "Node2_Node3"   },
        { "Node.Cupola",     "Cupola"        },
        { "Node.MRM1",       "MRM1"          },
        { "Node.MRM2",       "MRM2"          },
        { "Zvezda.SM",       "Zvezda"        },
        { "ArgUS_2-1",       "ArgUS_M2"      },
        { "ISS_iSEEP2",      "ISS_i-SEEP2"  },
        // Material named "JEM_EF_Diffuse" — its textures are JEM_EF_Diffuse.jpg, JEM_EF_Normal.jpg etc.
        // Map to "JEM_EF" so the suffix pass can find JEM_EF_Diffuse → _MainTex, JEM_EF_Normal → _BumpMap
        { "JEM_EF_Diffuse",  "JEM_EF"        },
        // Lettering
        { "Lettering_1",     "letters"       },
    };

    // Explicit rules for Dragon materials. matKeyword matched against material name
    // (case-insensitive). texKeyword finds the best texture by name. First match per slot wins.
    private static readonly List<(string matKeyword, string slot, string texKeyword)> ExplicitRules =
        new List<(string, string, string)>
    {
        ("main",         "_MainTex",          "capsule_silver"),
        ("main",         "_BumpMap",          "capsule_nrm"),
        ("main",         "_EmissionMap",      "capsule_emessive"),
        ("ctinside",     "_MainTex",          "ct_inside"),
        ("ct_inside",    "_MainTex",          "ct_inside"),
        ("trunk",        "_MainTex",          "trunk"),
        ("trunk",        "_BumpMap",          "trunk_normal"),
        ("trunk",        "_MetallicGlossMap", "trunk_specular"),
        ("dockingport",  "_MainTex",          "a5"),
        ("dockingport",  "_BumpMap",          "n5"),
        ("parachute",    "_MainTex",          "parachute"),
    };

    // Suffix patterns for the convention-based pass. Each entry is (suffixes, shader slot).
    // Checked after explicit rules. First suffix match per slot wins.
    private static readonly List<(string[] suffixes, string slot)> SuffixRules =
        new List<(string[], string)>
    {
        (new[] { "_diffuse", "_diff", "_albedo", "_color", "_col", "_albedotransparency" }, "_MainTex"),
        (new[] { "_normal", "_nrm", "_nor", "_nrml" },                                      "_BumpMap"),
        (new[] { "_metallic", "_metal", "_met", "_metallicsmoothness" },                    "_MetallicGlossMap"),
        (new[] { "_emission", "_emit", "_emissive" },                                        "_EmissionMap"),
        (new[] { "_occlusion", "_ao", "_occ" },                                             "_OcclusionMap"),
        (new[] { "_specc", "_spec" },                                                        "_SpecGlossMap"),
    };

    [MenuItem("Tools/Texture Auto-Assigner")]
    public static void ShowWindow()
    {
        GetWindow<TextureAutoAssigner>("Texture Auto-Assigner");
    }

    private void OnGUI()
    {
        GUILayout.Label("Texture Auto-Assigner", EditorStyles.boldLabel);
        EditorGUILayout.Space();

        materialsPath = EditorGUILayout.TextField("Materials Folder", materialsPath);
        texturesPath  = EditorGUILayout.TextField("Textures Folder",  texturesPath);

        EditorGUILayout.HelpBox(
            "Pass 1: Explicit Dragon rules.\n" +
            "Pass 2: Convention — finds textures starting with the material name (or its prefix override) " +
            "followed by a known role suffix (_Diffuse, _Normal, _Metallic, etc.).\n" +
            "Pass 3: Fuzzy fallback for _MainTex only.",
            MessageType.Info);

        EditorGUILayout.Space();

        if (GUILayout.Button("Generate Preview"))
            GeneratePreview();

        if (!previewGenerated)
            return;

        EditorGUILayout.Space();

        int matchCount = previews.Count(p => p.assignments.Count > 0);
        GUILayout.Label($"Preview — {matchCount} materials will receive textures:", EditorStyles.boldLabel);

        scrollPos = EditorGUILayout.BeginScrollView(scrollPos, GUILayout.Height(380));
        foreach (var preview in previews)
        {
            if (preview.assignments.Count == 0)
                continue;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            preview.apply = EditorGUILayout.Toggle(preview.apply, GUILayout.Width(20));
            GUILayout.Label(preview.material.name, EditorStyles.boldLabel);
            EditorGUILayout.EndHorizontal();

            EditorGUI.indentLevel++;
            foreach (var kvp in preview.assignments)
                EditorGUILayout.LabelField(kvp.Key, kvp.Value != null ? kvp.Value.name : "(none)");
            EditorGUI.indentLevel--;

            EditorGUILayout.EndVertical();
        }
        EditorGUILayout.EndScrollView();

        EditorGUILayout.Space();

        int toApply = previews.Count(p => p.apply && p.assignments.Count > 0);
        GUI.enabled = toApply > 0;
        if (GUILayout.Button($"Apply to {toApply} Material(s)"))
            ApplyAssignments();
        GUI.enabled = true;
    }

    private void GeneratePreview()
    {
        previews.Clear();

        string[] texGuids = AssetDatabase.FindAssets("t:Texture2D", new[] { texturesPath });
        var textures = texGuids
            .Select(g => AssetDatabase.LoadAssetAtPath<Texture2D>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(t => t != null)
            .ToList();

        if (textures.Count == 0)
            Debug.LogWarning("[TextureAutoAssigner] No textures found in: " + texturesPath);

        string[] matGuids = AssetDatabase.FindAssets("t:Material", new[] { materialsPath });

        foreach (string guid in matGuids)
        {
            string   path = AssetDatabase.GUIDToAssetPath(guid);
            Material mat  = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat == null) continue;

            var preview = new AssignmentPreview { material = mat, materialPath = path };
            string nameLower = mat.name.ToLower();

            // Pass 1: explicit Dragon rules.
            foreach (var (matKeyword, slot, texKeyword) in ExplicitRules)
            {
                if (!nameLower.Contains(matKeyword.ToLower())) continue;
                if (preview.assignments.ContainsKey(slot)) continue;
                if (!mat.HasProperty(slot)) continue;

                Texture2D tex = FindBestTexture(textures, texKeyword);
                if (tex != null)
                    preview.assignments[slot] = tex;
            }

            // Resolve the texture prefix: use override if one exists, otherwise the material name.
            string texPrefix = TexturePrefixOverrides.TryGetValue(mat.name, out string ov) ? ov : mat.name;

            // Pass 2: convention-based suffix matching.
            foreach (var (suffixes, slot) in SuffixRules)
            {
                if (preview.assignments.ContainsKey(slot)) continue;
                if (!mat.HasProperty(slot)) continue;

                Texture2D tex = FindTextureBySuffix(textures, texPrefix, suffixes);
                if (tex != null)
                    preview.assignments[slot] = tex;
            }

            // Pass 2b: exact name match → treat as diffuse (handles e.g. ELC_Base.png).
            if (!preview.assignments.ContainsKey("_MainTex") && mat.HasProperty("_MainTex"))
            {
                string prefixLower = texPrefix.ToLower();
                Texture2D exact = textures.FirstOrDefault(t => t.name.ToLower() == prefixLower);
                if (exact != null)
                    preview.assignments["_MainTex"] = exact;
            }

            // Pass 3: fuzzy fallback for _MainTex only.
            if (!preview.assignments.ContainsKey("_MainTex") && mat.HasProperty("_MainTex"))
            {
                Texture2D tex = FuzzyMatch(textures, texPrefix);
                if (tex != null)
                    preview.assignments["_MainTex"] = tex;
            }

            previews.Add(preview);
        }

        previewGenerated = true;
    }

    // Finds a texture whose normalized name starts with the normalized prefix and whose
    // remainder after the prefix starts with one of the given suffixes.
    private static Texture2D FindTextureBySuffix(List<Texture2D> textures, string prefix, string[] suffixes)
    {
        // Normalize: lowercase, spaces and dots → underscores.
        string prefixNorm = Normalize(prefix);

        var candidates = textures.Where(t =>
        {
            string tNorm = Normalize(t.name);
            if (!tNorm.StartsWith(prefixNorm)) return false;
            string remainder = tNorm.Substring(prefixNorm.Length);
            return suffixes.Any(s => remainder.StartsWith(s, System.StringComparison.OrdinalIgnoreCase));
        }).ToList();

        // Among candidates, prefer the one whose name is shortest (most specific match).
        return candidates.OrderBy(t => t.name.Length).FirstOrDefault();
    }

    private static string Normalize(string s) =>
        s.ToLower().Replace(' ', '_').Replace('.', '_').Replace('-', '_');

    // Exact name match first, then substring.
    private static Texture2D FindBestTexture(List<Texture2D> textures, string keyword)
    {
        string kw = keyword.ToLower();
        return textures.FirstOrDefault(t => t.name.ToLower() == kw)
            ?? textures.FirstOrDefault(t => t.name.ToLower().Contains(kw));
    }

    // Score textures by how many words from the (stripped) prefix appear in the texture name.
    // Skips textures that have a role suffix (they belong to the convention pass).
    private static Texture2D FuzzyMatch(List<Texture2D> textures, string prefix)
    {
        var roleSuffixes = new[] { "_normal", "_nrm", "_diffuse", "_diff", "_metallic",
                                   "_emission", "_emit", "_occlusion", "_spec", "_albedo" };
        var candidates = textures.Where(t =>
        {
            string tl = t.name.ToLower();
            return !roleSuffixes.Any(s => tl.Contains(s));
        }).ToList();

        string stripped = prefix
            .Replace("KK_", "").Replace("SPX_", "").Replace("SPXCD_", "").Replace("SpXCD_", "")
            .Split('.')[0]
            .ToLower();

        string[] words = stripped.Split(new[] { '_', '-', ' ' }, System.StringSplitOptions.RemoveEmptyEntries)
                                 .Where(w => w.Length > 2)
                                 .ToArray();

        if (words.Length == 0) return null;

        return candidates
            .Select(t => (tex: t, score: words.Count(w => t.name.ToLower().Contains(w))))
            .Where(x => x.score > 0)
            .OrderByDescending(x => x.score)
            .Select(x => x.tex)
            .FirstOrDefault();
    }

    private void ApplyAssignments()
    {
        int count = 0;
        foreach (var preview in previews.Where(p => p.apply && p.assignments.Count > 0))
        {
            foreach (var kvp in preview.assignments)
            {
                preview.material.SetTexture(kvp.Key, kvp.Value);

                if (kvp.Key == "_BumpMap")
                    preview.material.EnableKeyword("_NORMALMAP");

                if (kvp.Key == "_EmissionMap")
                {
                    preview.material.EnableKeyword("_EMISSION");
                    preview.material.globalIlluminationFlags =
                        MaterialGlobalIlluminationFlags.BakedEmissive;
                }
            }
            EditorUtility.SetDirty(preview.material);
            count++;
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"[TextureAutoAssigner] Applied textures to {count} material(s).");

        previewGenerated = false;
        previews.Clear();
    }
}
#endif
