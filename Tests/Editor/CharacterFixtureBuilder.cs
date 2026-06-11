using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.CharacterController2D.Authoring;

namespace Zori.Entities.CharacterController2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the C4a behavioural-gate PlayMode fixtures — one parent scene per
    /// gate, each carrying a SubScene with a kinematic <see cref="CharacterController2DAuthoring"/> character and
    /// the static world geometry (floor / wall / slope / overlap) the gate exercises. Mirrors the substrate's
    /// fixture builders (<c>FallingBodyFixtureBuilder</c>, <c>ColliderShapeFixtureBuilder</c>): the static world is
    /// collider-only <see cref="BoxCollider2D"/> GameObjects (no <see cref="Rigidbody2D"/>) so the substrate's
    /// static-body fallback bakes them, the character is a <see cref="CharacterController2DAuthoring"/> that the
    /// controller baker turns into a live kinematic Box2D body + the character archetype, and both parent + child are
    /// registered in build settings so the runtime test reaches them by <c>SceneManager.LoadScene</c>.
    /// </summary>
    /// <remarks>
    /// SubScene authoring is edit-time, so it lives in this editor-only assembly, not the runtime PlayMode test.
    /// Run via the menu or
    /// <c>-executeMethod Zori.Entities.CharacterController2D.Tests.Editor.CharacterFixtureBuilder.BuildAll</c>
    /// before the PlayMode gate. The runtime test (CharacterSolveGate) adds the <c>DefaultCharacterController2DTag</c>
    /// to the baked character at runtime (the baker does not author it — the tag is the opt-in to the default solve),
    /// then drives the FixedStepSimulationSystemGroup and asserts LocalToWorld invariants.
    /// </remarks>
    public static class CharacterFixtureBuilder
    {
        public const string FixtureRoot = "Assets/CharacterController2DFixture";

        public const string GroundingParent = FixtureRoot + "/CC2D_Grounding.unity";
        public const string GroundingChild = FixtureRoot + "/CC2D_Grounding_Sub.unity";
        public const string WallParent = FixtureRoot + "/CC2D_Wall.unity";
        public const string WallChild = FixtureRoot + "/CC2D_Wall_Sub.unity";
        public const string SlopeParent = FixtureRoot + "/CC2D_Slope.unity";
        public const string SlopeChild = FixtureRoot + "/CC2D_Slope_Sub.unity";
        public const string WallGroundedParent = FixtureRoot + "/CC2D_WallGrounded.unity";
        public const string WallGroundedChild = FixtureRoot + "/CC2D_WallGrounded_Sub.unity";
        public const string OverlapShallowParent = FixtureRoot + "/CC2D_OverlapShallow.unity";
        public const string OverlapShallowChild = FixtureRoot + "/CC2D_OverlapShallow_Sub.unity";
        public const string OverlapDeepParent = FixtureRoot + "/CC2D_OverlapDeep.unity";
        public const string OverlapDeepChild = FixtureRoot + "/CC2D_OverlapDeep_Sub.unity";
        public const string OverlapGroundParent = FixtureRoot + "/CC2D_OverlapGround.unity";
        public const string OverlapGroundChild = FixtureRoot + "/CC2D_OverlapGround_Sub.unity";

        // The character circle proxy radius used across all fixtures — the runtime test asserts the grounded settle
        // height against this.
        public const float CharacterRadius = 0.5f;

        // Floor top surface sits at Y=0 (a thin box centred just below 0). The runtime gate
        // (CharacterFixtureBuilderConstants.FloorTopY) hard-codes the same 0 — kept in sync by convention rather than
        // a cross-assembly reference (the editor builder and the all-platforms runtime test do not reference each
        // other).
        public const float FloorTopY = 0f;

        [MenuItem("Tools/Zori/Build Character Controller 2D Gate Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildGrounding();
            BuildWall();
            BuildSlope();
            BuildWallGrounded();
            BuildOverlapShallow();
            BuildOverlapDeep();
            BuildOverlapGround();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Character Controller 2D gate fixtures built.");
        }

        // ---- individual fixtures ---------------------------------------------------------------------------

        // Grounding: a character dropped a little above a flat floor → settles at ~radius above the floor top.
        static void BuildGrounding()
        {
            BuildFixture(
                GroundingChild,
                GroundingParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddCharacter(root, new Vector3(0f, 3f, 0f));
                });
        }

        // Collide-and-slide into a wall: character starts grounded near the floor, the test drives +X velocity into
        // a vertical wall whose left face is at X=3. X must clamp before the proxy penetrates the wall.
        static void BuildWall()
        {
            BuildFixture(
                WallChild,
                WallParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Wall: a tall thin box whose left face is at X = 3 (centre at 3 + 0.5).
                    AddFloor(root, new Vector2(3.5f, 3f), new Vector2(1f, 10f));
                    AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                });
        }

        // Slope within limit: a 30-degree ramp the character climbs. The ramp is a rotated thin box; the character
        // starts on the low (left) end and the test drives +X velocity up it.
        static void BuildSlope()
        {
            BuildFixture(
                SlopeChild,
                SlopeParent,
                root =>
                {
                    var ramp = AddFloor(root, new Vector2(2f, 0f), new Vector2(20f, 1f));
                    ramp.transform.rotation = Quaternion.Euler(0f, 0f, 30f);
                    // Character starts above the low end of the ramp (left side), close to its top surface.
                    AddCharacter(root, new Vector3(-4f, 1.5f, 0f));
                });
        }

        // Wall while grounded: character grounded on a flat floor, walks into a vertical wall. It must slide along
        // (stay grounded, stay near floor height) rather than climb or penetrate.
        static void BuildWallGrounded()
        {
            BuildFixture(
                WallGroundedChild,
                WallGroundedParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddFloor(root, new Vector2(3.5f, 3f), new Vector2(1f, 10f));
                    AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                });
        }

        // Depenetration shallow: character spawned overlapping a wall by a little less than the radius.
        static void BuildOverlapShallow()
        {
            BuildFixture(
                OverlapShallowChild,
                OverlapShallowParent,
                root =>
                {
                    // Wall left face at X = 0. Character centre at X = 0.2 → overlapping by ~0.3 (radius 0.5).
                    AddFloor(root, new Vector2(2.5f, 5f), new Vector2(5f, 20f));
                    var c = AddCharacter(root, new Vector3(0.2f, 5f, 0f));
                    // No grounding/gravity confound: this is a pure horizontal-overlap probe.
                    DisableGrounding(c);
                });
        }

        // Depenetration DEEP (the adversarial probe): character centre well INSIDE the wall, deeper than the
        // collision offset and approaching the radius. The cast-back must still recover a push-out direction and the
        // iterative loop converge to no residual overlap, with no NaN / teleport.
        static void BuildOverlapDeep()
        {
            BuildFixture(
                OverlapDeepChild,
                OverlapDeepParent,
                root =>
                {
                    // Wall left face at X = 0; character centre at X = -0.45 → the proxy centre is 0.45 INSIDE the
                    // wall, much deeper than CollisionOffset (0.01) and nearly the full radius.
                    AddFloor(root, new Vector2(2.5f, 5f), new Vector2(5f, 20f));
                    var c = AddCharacter(root, new Vector3(-0.45f, 5f, 0f));
                    DisableGrounding(c);
                    // Give depenetration more iterations per step so a deep overlap resolves quickly and the test can
                    // assert convergence within a bounded number of fixed steps.
                    BumpOverlapIterations(c, 8);
                });
        }

        // Depenetration against the GROUND while grounded: a character spawned sunk into the floor. It must pop
        // straight UP out of the floor (the grounded vertical-decollide branch), not sideways.
        static void BuildOverlapGround()
        {
            BuildFixture(
                OverlapGroundChild,
                OverlapGroundParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Character sunk so its centre is at Y = 0.1 — well below the radius-above-floor rest height
                    // (~0.51), so it is penetrating the floor by ~0.4.
                    var c = AddCharacter(root, new Vector3(0f, 0.1f, 0f));
                    BumpOverlapIterations(c, 8);
                });
        }

        // ---- authoring helpers -----------------------------------------------------------------------------

        static GameObject AddFloor(GameObject parent, Vector2 center, Vector2 size)
        {
            var go = new GameObject("Static");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = new Vector3(center.x, center.y, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = size;
            return go;
        }

        static CharacterController2DAuthoring AddCharacter(GameObject parent, Vector3 position)
        {
            var go = new GameObject("Character");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var auth = go.AddComponent<CharacterController2DAuthoring>();

            var props = AuthoringKinematicCharacterProperties2D.GetDefault();
            // The default mass-interaction with dynamic bodies is irrelevant to the C4a core gates and there are no
            // dynamic bodies in these fixtures; leave SimulateDynamicBody default. Use a circle proxy at the shared
            // radius so the grounded settle-height assertion is exact.
            auth.CharacterProperties = props;
            auth.ProxyShape = CharacterProxyShape2D.Circle;
            auth.ProxyRadius = CharacterRadius;
            // Render interpolation lags LocalToWorld by a render frame; turn it off so the test reads the raw
            // fixed-step pose directly off LocalToWorld without a smoothing offset confound.
            auth.InterpolateRendering = false;
            return auth;
        }

        static void DisableGrounding(CharacterController2DAuthoring auth)
        {
            var props = auth.CharacterProperties;
            props.EvaluateGrounding = false;
            props.SnapToGround = false;
            auth.CharacterProperties = props;
        }

        static void BumpOverlapIterations(CharacterController2DAuthoring auth, byte iterations)
        {
            var props = auth.CharacterProperties;
            props.MaxOverlapDecollisionIterations = iterations;
            auth.CharacterProperties = props;
        }

        // ---- scene plumbing (mirrors the substrate builders) -----------------------------------------------

        static void BuildFixture(string childPath, string parentPath, System.Action<GameObject> populate)
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("FixtureRoot");
            populate(root);
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(Path.GetFileNameWithoutExtension(childPath) + " SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            RegisterSceneInBuildSettings(parentPath);
            RegisterSceneInBuildSettings(childPath);
        }

        static void RegisterSceneInBuildSettings(string scenePath)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(s => s.path == scenePath))
                return;
            scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
