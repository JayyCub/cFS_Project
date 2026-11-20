using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// Batch-mode entry point for build-mac.sh. Names the scene directly because the project's
/// scene list is empty (the Editor build gets its scene from a Build Profile in Library/).
/// </summary>
public static class BuildSimulator
{
    const string Scene  = "Assets/Scene2.unity";
    const string Output = "Builds/SimulatorScene.app";

    public static void Mac()
    {
        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes           = new[] { Scene },
            locationPathName = Output,
            target           = BuildTarget.StandaloneOSX,
            options          = BuildOptions.None,
        });

        Debug.Log($"[BuildSimulator] {report.summary.result}, {report.summary.totalErrors} errors");
        if (report.summary.result != BuildResult.Succeeded) EditorApplication.Exit(1);
    }
}
