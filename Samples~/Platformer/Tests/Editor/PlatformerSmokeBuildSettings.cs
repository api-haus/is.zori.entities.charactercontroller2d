#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEditor;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Tests.Editor
{
    /// <summary>
    /// Registers the Platformer course scene in <see cref="EditorBuildSettings"/> so the PlayMode smoke gate's
    /// <c>SceneManager.LoadScene("PlatformerSample")</c> can resolve it. Editor 6000.6 reads the build-profile scene
    /// list at PlayMode ENTRY, so registering it from inside the PlayMode test's <c>[OneTimeSetUp]</c> is too late —
    /// this runs as a SEPARATE editor batchmode pass BEFORE the test run
    /// (<c>-executeMethod …PlatformerSmokeBuildSettings.Register</c>). The temp-Assets-copy dance cleanup restores the
    /// build-settings asset afterward, so the registration does not persist.
    /// </summary>
    public static class PlatformerSmokeBuildSettings
    {
        const string ParentScenePath = "Assets/_PlatformerSmoke/Scenes/PlatformerSample.unity";
        const string SubScenePath = "Assets/_PlatformerSmoke/Scenes/PlatformerSample_Sub.unity";

        // The IMPORTED sample's own scene paths (Window → Package Manager → import samples puts them here). When the
        // sample is imported (it is in this project), the course scene already lives in Assets and needs no temp
        // copy — just register it at its real path so SceneManager.LoadScene("PlatformerSample") resolves it.
        const string ImportedParentScenePath =
            "Assets/Samples/Zori Entities Character Controller 2D/0.1.0/Platformer Character/Scenes/PlatformerSample.unity";
        const string ImportedSubScenePath =
            "Assets/Samples/Zori Entities Character Controller 2D/0.1.0/Platformer Character/Scenes/PlatformerSample_Sub.unity";

        public static void Register()
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            EnsurePresent(scenes, ParentScenePath);
            EnsurePresent(scenes, SubScenePath);
            EditorBuildSettings.scenes = scenes.ToArray();
            UnityEngine.Debug.Log(
                $"[PlatformerSmoke] registered {ParentScenePath} + sub in build settings ({scenes.Count} total)"
            );
        }

        // Register the IMPORTED sample scene (its real Assets path) so the smoke gate can load it without the
        // temp-Assets-copy dance. Run as a separate editor batchmode pass before the PlayMode test run
        // (-executeMethod …PlatformerSmokeBuildSettings.RegisterImported).
        public static void RegisterImported()
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            EnsurePresent(scenes, ImportedParentScenePath);
            EnsurePresent(scenes, ImportedSubScenePath);
            EditorBuildSettings.scenes = scenes.ToArray();
            UnityEngine.Debug.Log(
                $"[PlatformerSmoke] registered IMPORTED {ImportedParentScenePath} + sub ({scenes.Count} total)"
            );
        }

        static void EnsurePresent(List<EditorBuildSettingsScene> scenes, string path)
        {
            foreach (var s in scenes)
            {
                if (s.path == path)
                    return;
            }
            scenes.Add(new EditorBuildSettingsScene(path, true));
        }
    }
}
#endif
