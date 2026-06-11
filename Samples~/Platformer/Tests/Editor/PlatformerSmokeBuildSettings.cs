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

        public static void Register()
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            EnsurePresent(scenes, ParentScenePath);
            EnsurePresent(scenes, SubScenePath);
            EditorBuildSettings.scenes = scenes.ToArray();
            UnityEngine.Debug.Log($"[PlatformerSmoke] registered {ParentScenePath} + sub in build settings ({scenes.Count} total)");
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
