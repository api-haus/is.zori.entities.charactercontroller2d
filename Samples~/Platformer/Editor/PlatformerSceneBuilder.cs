using System.IO;
using TMPro;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.CharacterController2D.Authoring;
using Zori.Entities.CharacterController2D.Diagnostics;
using PhysicsBody2DAuthoring = Zori.Entities.Physics2D.Authoring.PhysicsBody2DAuthoring;
using PhysicsBody2DMotionType = Zori.Entities.Physics2D.Authoring.PhysicsBody2DMotionType;
using PhysicsCollisionResponse2D = Zori.Entities.Physics2D.PhysicsCollisionResponse2D;
using PhysicsShape2DAuthoring = Zori.Entities.Physics2D.Authoring.PhysicsShape2DAuthoring;
using PhysicsShape2DKind = Zori.Entities.Physics2D.PhysicsShape2DKind;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the Platformer sample course — the same proven pattern the SideScroller's
    /// <see cref="Zori.Entities.CharacterController2D.Samples.Editor.SideScrollerSceneBuilder"/> uses (programmatic
    /// <c>AddComponent&lt;T&gt;()</c> authoring + an <c>EditorSceneManager</c> SubScene save), rather than hand-authored
    /// fragile <c>.unity</c> YAML. It builds a parent scene (the one you open, carrying the SubScene reference, the 2D
    /// orthographic follow camera, and <see cref="DebugPhysicsWorld2D"/>) and a SubScene child (the baked ECS course: a
    /// capsule character, the static world, the three friction floor segments + a bouncy crate, two moving platforms, a
    /// wind sensor, a rope anchor on a dedicated category, a teleporter sensor + destination, and pushable crates).
    ///
    /// <para>Run via the menu (Tools/Zori/Build Platformer Sample Scene) or
    /// <c>-executeMethod Zori.Entities.CharacterController2D.Samples.Platformer.Editor.PlatformerSceneBuilder.Build</c>.
    /// The scene writes into the sample's own <c>Scenes/</c> folder so the importable sample carries it.</para>
    ///
    /// <para><b>EditorBuildSettings is NOT touched.</b> Unlike the SideScroller builder — which registered both its
    /// scenes into <c>ProjectSettings/EditorBuildSettings.asset</c> and so polluted the consuming project's build list
    /// with sample scenes (a prior commit had to restore that file) — this builder registers NOTHING. In-editor PlayMode
    /// SubScene loading needs no build-settings registration: the parent scene's <see cref="SubScene"/> references the
    /// child by its <c>SceneAsset</c> GUID and Unity loads/bakes it from that reference when the parent scene plays,
    /// regardless of whether either scene is in the build list. A player BUILD that ships this sample would add the
    /// parent scene itself, but that is a build-time decision for the consuming project, not something a sample
    /// scene-builder should force into every project that imports the package.</para>
    /// </summary>
    public static class PlatformerSceneBuilder
    {
        // Vertical reference: the main floor's top surface sits at Y = 0.
        const float FloorTopY = 0f;

        // Capsule character: full size 1 x 2 (width, height), vertical caps. Spawned standing on the floor.
        const float CharacterWidth = 1f;
        const float CharacterHeight = 2f;
        const float CharacterHalfHeight = CharacterHeight * 0.5f;

        // Step/slope limits the character authoring is tuned to (carried from the SideScroller's verified course).
        const float MaxStepHeight = 0.5f;
        const float MaxSlopeDeg = 60f;

        // The rope anchor lives on a dedicated Box2D contact category so the grab query selects ONLY anchors, never
        // floors/walls (which sit on category bit 0, the Default layer). Both the anchor's authored category bit and
        // the character's RopeAnchorLayerMask use this same bit. Bit 6 is unused by the project's named layers.
        const int RopeAnchorCategoryBit = 6;
        const ulong RopeAnchorCategoryMask = 1ul << RopeAnchorCategoryBit;

        /// <summary>
        /// The <c>Scenes/</c> directory next to this builder, resolved from the builder's OWN asset path so the scene
        /// is written wherever the sample currently lives (the importable copy under <c>Assets/Samples/…</c>, or any
        /// other location it is dropped). A hard-coded <c>Samples~/</c> path would NOT work: a tilde folder is not an
        /// <c>AssetDatabase</c> path (Unity ignores <c>Samples~</c> entirely), so <c>SaveScene</c> /
        /// <c>LoadAssetAtPath&lt;SceneAsset&gt;</c> against it would fail. The scene is therefore a GENERATED artifact,
        /// produced by running this builder once after import — the same model the SideScroller builder uses.
        /// </summary>
        static string SceneDir
        {
            get
            {
                // Find this builder's own MonoScript asset path (its …/Editor folder), then its sibling …/Scenes. The
                // script name is unique in the project, so FindAssets returns exactly this script.
                string[] guids = AssetDatabase.FindAssets("PlatformerSceneBuilder t:MonoScript");
                string scriptPath = guids.Length > 0 ? AssetDatabase.GUIDToAssetPath(guids[0]) : null;
                if (string.IsNullOrEmpty(scriptPath))
                {
                    // Not yet under an AssetDatabase path (e.g. still in Samples~ before import) — nothing to build.
                    throw new System.InvalidOperationException(
                        "PlatformerSceneBuilder is not under an AssetDatabase path; import the sample into "
                            + "Assets/Samples/ first (or copy the sample tree into Assets/), then run the builder."
                    );
                }

                string editorDir = Path.GetDirectoryName(scriptPath).Replace('\\', '/');
                string sampleRoot = Path.GetDirectoryName(editorDir).Replace('\\', '/');
                return sampleRoot + "/Scenes";
            }
        }

        static string ParentScene => SceneDir + "/PlatformerSample.unity";
        static string ChildScene => SceneDir + "/PlatformerSample_Sub.unity";

        [MenuItem("Tools/Zori/Build Platformer Sample Scene")]
        public static void Build()
        {
            string sceneDir = SceneDir;
            if (!AssetDatabase.IsValidFolder(sceneDir))
            {
                string parent = Path.GetDirectoryName(sceneDir).Replace('\\', '/');
                AssetDatabase.CreateFolder(parent, "Scenes");
            }

            BuildChild();
            BuildParent();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Platformer sample scene built at " + ParentScene);
        }

        // The SubScene child: the baked ECS course content.
        static void BuildChild()
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // === Station 1 — spawn + flat ground, then the three friction segments ============================
            //
            // A normal-friction segment under the spawn, then an ICE segment (slippery) and a STICKY segment, all at
            // floor top Y = 0 and butted edge-to-edge so the character walks straight across and FEELS the change in
            // GroundMove sharpness. The three are one continuous walking surface (the FrictionModifier2D is read off
            // the character's ground hit, so each segment's modifier applies while standing on it).
            AddFrictionFloor(centerX: -3f, width: 14f, friction: 1f, name: "Floor_Normal"); // X in [-10, 4]
            AddFrictionFloor(centerX: 9f, width: 10f, friction: 0.15f, name: "Floor_Ice"); // X in [4, 14] — slippery
            AddFrictionFloor(centerX: 19f, width: 10f, friction: 3f, name: "Floor_Sticky"); // X in [14, 24] — sticky

            // A bouncy DYNAMIC crate sitting on the normal segment: a high-bounciness PhysicsMaterial2D-style surface
            // (authored inline via the shape's bounciness override) so it visibly bounces when dropped / shoved, and a
            // plain pushable crate the character can shove onto the ICE to show the crate's own material response.
            AddBouncyCrate(
                new Vector2(-6f, FloorTopY + 0.5f + 0.05f),
                size: new Vector2(1f, 1f),
                mass: 0.5f,
                "BouncyCrate"
            );
            AddPushableCrate(
                new Vector2(2f, FloorTopY + 0.5f + 0.05f),
                size: new Vector2(1f, 1f),
                mass: 1f,
                "PushCrate"
            );

            // === Station 2 — step + slope course (carried from the SideScroller, verified green) ===============
            //
            // A step within the limit (mountable), a step over it (a wall), a slope within the limit (climbable), a
            // slope over it (slide-back). Placed just past the sticky segment so the core traversal stays demonstrated.
            AddStaticBox(new Vector2(26f, 0.3f - 1f), new Vector2(4f, 2f), "StepLow_top0.3"); // top 0.3 <= 0.5
            AddStaticBox(new Vector2(31f, 0.9f - 1.5f), new Vector2(4f, 3f), "StepHigh_top0.9"); // top 0.9 > 0.5
            AddRamp(lipX: 34f, lipY: FloorTopY, lengthAlong: 12f, slopeDeg: 30f, "SlopeWithinLimit_30deg");
            AddRamp(lipX: 44f, lipY: FloorTopY, lengthAlong: 7f, slopeDeg: 75f, "SlopeOverLimit_75deg");

            // A landing ledge after the slope course, so the slope-within-limit climb leads somewhere walkable.
            AddStaticBox(new Vector2(49f, 6f - 0.5f), new Vector2(8f, 1f), "Ledge_AfterSlopes"); // top at Y = 6

            // === Station 3 — moving platforms over a gap (one lateral, one vertical) ===========================
            //
            // A gap to the right of the ledge bridged by a lateral platform, then a vertical platform lifting the
            // character to a higher ledge. The character rides each (the SideScroller's verified carry, generalized to
            // two axes).
            AddMovingPlatform(
                new Vector2(58f, 6f - 0.25f),
                size: new Vector2(4f, 0.5f),
                travelHalfExtent: new Vector2(4f, 0f),
                speed: 3f,
                "MovingPlatform_Lateral"
            );
            AddMovingPlatform(
                new Vector2(66f, 5f),
                size: new Vector2(4f, 0.5f),
                travelHalfExtent: new Vector2(0f, 3f),
                speed: 2.5f,
                "MovingPlatform_Vertical"
            );

            // The higher ledge the vertical platform tops out at, leading into the wind zone.
            AddStaticBox(new Vector2(73f, 9f - 0.5f), new Vector2(10f, 1f), "Ledge_HighWind"); // top at Y = 9

            // === Station 4 — wind / force zone ================================================================
            //
            // A sensor volume above the high ledge with a constant upward+forward force: the character walking through
            // is pushed up and along (the trigger-read WindZoneSystem2D writes RelativeVelocity). An updraft that helps
            // clear the next obstacle.
            AddWindZone(
                new Vector2(73f, 11f),
                size: new Vector2(8f, 4f),
                force: new Vector2(2f, 14f),
                "WindZone_Updraft"
            );

            // === Station 5 — rope swing across a gap too wide to jump =========================================
            //
            // After the high ledge, a wide gap (the launch ledge ends at X = 78, the far ledge begins at X = 92 — a
            // 14-unit gap, far past a single jump). A rope anchor floats above the gap on the dedicated rope-anchor
            // category; the character jumps off the launch ledge, grabs (E), swings, and releases (Q / jump) to land on
            // the far ledge. The anchor sits ABOVE the jump apex, so the GRAB REACH (RopeAnchorSearchRadius, 12) — not
            // the rope length — is what must reach it; the 6-unit rope is the swing-arc radius that clears the gap.
            AddRopeAnchor(new Vector2(85f, 16f), radius: 0.3f, "RopeAnchor");
            AddStaticBox(new Vector2(95f, 9f - 0.5f), new Vector2(8f, 1f), "Ledge_FarRope"); // top at Y = 9

            // === Station 6 — teleporter pad + destination =====================================================
            //
            // A sensor pad on the far rope ledge; stepping on it teleports the character to a destination marker back
            // near the spawn (so the course loops). The destination is an empty marker GameObject; the teleporter
            // baker resolves it to its entity and TeleporterSystem2D moves the character there on entry.
            var teleportDestination = AddDestinationMarker(
                new Vector3(0f, FloorTopY + CharacterHalfHeight + 0.1f, 0f),
                "TeleportDestination"
            );
            AddTeleporter(
                new Vector2(97f, FloorTopY + 9f + 0.5f),
                size: new Vector2(1.5f, 1f),
                destination: teleportDestination,
                "Teleporter_BackToSpawn"
            );

            // === The character ================================================================================
            //
            // A CAPSULE-proxy character (1 x 2, vertical caps) spawned standing on the normal floor segment. The
            // capsule mandate is satisfied here, on the sibling controller authoring's Capsule proxy choice.
            AddCharacter(new Vector3(0f, FloorTopY + CharacterHalfHeight + 0.05f, 0f));

            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, ChildScene);
        }

        // The parent scene: the SubScene reference + the 2D orthographic follow camera + DebugPhysicsWorld2D.
        static void BuildParent()
        {
            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var subSceneGo = new GameObject("PlatformerSample SubScene");
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(ChildScene);
            subScene.AutoLoadScene = true;

            // The follow camera (orthographic), looking down +Z at the 2D plane.
            var camGo = new GameObject("Main Camera");
            var cam = camGo.AddComponent<Camera>();
            cam.orthographic = true;
            cam.orthographicSize = 8f;
            cam.transform.position = new Vector3(0f, 2f, -10f);
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.12f, 0.13f, 0.18f, 1f);
            camGo.AddComponent<PlatformerFollowCamera>();
            camGo.tag = "MainCamera";

            // The debug draw, on a regular parent-scene GameObject (NOT the SubScene — it is a hybrid MonoBehaviour,
            // not an authoring marker). Toggle on so the otherwise-invisible ECS physics bodies outline in the Game
            // view while playing.
            var debugGo = new GameObject("DebugPhysicsWorld2D");
            var debug = debugGo.AddComponent<DebugPhysicsWorld2D>();
            debug.DrawWorld = true;

            // World-space TextMeshPro feature labels, one beside each course feature so the otherwise-unlabelled
            // course explains itself in the Game view (especially the rope's grab/release keys). These are plain
            // rendering GameObjects living in the PARENT scene — NOT authoring markers in the SubScene — because they
            // carry no ECS component and bake to nothing; they are MeshRenderer-backed 3D text the camera sees while
            // playing. Their world positions match the SubScene feature positions (the SubScene loads at origin in
            // PlayMode, so a parent-scene label at the feature's world coordinate sits beside it).
            BuildLabels();

            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, ParentScene);

            // Deliberately NO EditorBuildSettings registration here — see the class summary. In-editor PlayMode
            // SubScene loading works from the SubScene's SceneAsset GUID reference alone; polluting the consuming
            // project's build list with sample scenes is the SideScroller builder's bug, not repeated.
        }

        // ---- authoring helpers ---------------------------------------------------------------------------------

        // A collider-only static box (no Rigidbody2D) — the substrate's static-body fallback bakes it.
        static GameObject AddStaticBox(Vector2 center, Vector2 size, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(center.x, center.y, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = size;
            return go;
        }

        // A flat static floor segment carrying a FrictionModifier2D the character's GroundMove stance feels. Its top
        // surface sits at FloorTopY; the segment is 1 unit thick.
        static GameObject AddFrictionFloor(float centerX, float width, float friction, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(centerX, FloorTopY - 0.5f, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = new Vector2(width, 1f);

            var modifier = go.AddComponent<FrictionModifier2DAuthoring>();
            modifier.Friction = friction;
            return go;
        }

        // A long thin static box rotated to form a ramp, its high (left) end butted at the lip (lipX, lipY) so its top
        // surface ascends to the right at slopeDeg. The character climbs it from the lip.
        static GameObject AddRamp(float lipX, float lipY, float lengthAlong, float slopeDeg, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(lipX + lengthAlong * 0.5f, lipY - 0.5f, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = new Vector2(lengthAlong, 1f);
            go.transform.RotateAround(new Vector3(lipX, lipY, 0f), Vector3.forward, slopeDeg);
            return go;
        }

        static void AddCharacter(Vector3 position)
        {
            var go = new GameObject("Character");
            go.transform.position = position;

            var auth = go.AddComponent<CharacterController2DAuthoring>();
            var props = AuthoringKinematicCharacterProperties2D.GetDefault();
            props.MaxGroundedSlopeAngle = MaxSlopeDeg;
            auth.CharacterProperties = props;

            // The CAPSULE mandate: the cast proxy AND the matching substrate world shape are both a capsule, derived
            // from this one Capsule proxy choice + size/direction by CharacterController2DBaker. A 1 x 2 vertical
            // capsule (a standing character). Re-asserting this here is load-bearing — the companion
            // PlatformerCharacterAuthoring carries NO capsule field, so the capsule lives entirely on this sibling.
            auth.ProxyShape = CharacterProxyShape2D.Capsule;
            auth.ProxyCapsuleSize = new float2(CharacterWidth, CharacterHeight);
            auth.ProxyCapsuleDirection = CharacterCapsuleDirection2D.Vertical;
            auth.InterpolateRendering = true;

            // Step handling on so the character mounts the low step. Width-for-step-grounding = capsule width.
            var step = auth.StepAndSlopeHandling;
            step.StepHandling = true;
            step.MaxStepHeight = MaxStepHeight;
            step.CharacterWidthForStepGroundingCheck = CharacterWidth;
            auth.StepAndSlopeHandling = step;

            // The Platformer companion: per-character movement + rope tuning, the initial stance, the tag. Its baker
            // bakes PlatformerCharacterTuning2D from these fields; CharacterController2DBaker bakes the capsule body.
            var platformer = go.AddComponent<PlatformerCharacterAuthoring>();
            // The rope-grab query filters strictly to the rope-anchor category; the character's mask must select the
            // anchor's dedicated category bit (and ONLY it — bit 0 would also catch floors/walls).
            platformer.RopeAnchorLayerMask = (int)RopeAnchorCategoryMask;
            // The rope LENGTH is the swing-constraint radius; the SEARCH RADIUS is the grab reach (separate dials).
            // The anchor at (85, 16) sits ~7 units above the launch ledge top (Y = 9), well past the ~2-unit jump apex
            // rise, so a grab reach equal to RopeLength (6) never reached it — the original conflation was the grab bug.
            // A generous search radius (12) lets the character grab the anchor from anywhere on/above the launch ledge;
            // the 6-unit rope keeps the swing arc clearing the gap floor.
            platformer.RopeLength = 6f;
            platformer.RopeAnchorSearchRadius = 12f;
            // Below the lowest walkable surface (the spawn floor top is Y = 0); a fall past this respawns at the last
            // safe point. The course never legitimately reaches Y = -15, so only a genuine fall triggers it.
            platformer.FallRespawnThresholdY = -15f;
        }

        // A bouncy DYNAMIC crate: a high-bounciness shape (the substrate's real PhysicsShape2D material the SOLVER
        // resolves), also pushable so the character can shove it. Shows the prop-response material layer.
        static void AddBouncyCrate(Vector2 position, Vector2 size, float mass, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(position.x, position.y, 0f);

            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.UseAutoMass = false;
            body.Mass = mass;
            body.GravityScale = 1f;

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new float2(size.x, size.y);
            shape.Density = 1f;
            // The bouncy material: a high coefficient of restitution the Box2D solver resolves on contact.
            shape.OverrideBounciness = true;
            shape.Bounciness = 0.85f;

            go.AddComponent<Pushable2DAuthoring>();
        }

        static void AddPushableCrate(Vector2 position, Vector2 size, float mass, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(position.x, position.y, 0f);

            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.UseAutoMass = false;
            body.Mass = mass;
            body.GravityScale = 1f;

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new float2(size.x, size.y);
            shape.Density = 1f;

            go.AddComponent<Pushable2DAuthoring>();
        }

        static void AddMovingPlatform(Vector2 center, Vector2 size, Vector2 travelHalfExtent, float speed, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(center.x, center.y, 0f);

            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Kinematic; // driven by MovePosition; never falls.

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new float2(size.x, size.y);

            var mover = go.AddComponent<MovingPlatform2DAuthoring>();
            mover.TravelHalfExtent = travelHalfExtent;
            mover.Speed = speed;
        }

        // A trigger-sensor wind/force zone. The sensor shape reports trigger events but never produces a collision
        // response; WindZoneSystem2D reads those events and writes the force into the character's RelativeVelocity.
        static void AddWindZone(Vector2 center, Vector2 size, Vector2 force, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(center.x, center.y, 0f);

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new float2(size.x, size.y);
            shape.CollisionResponse = PhysicsCollisionResponse2D.Sensor; // a trigger volume, not a solid.

            var zone = go.AddComponent<WindZone2DAuthoring>();
            zone.Force = force;
        }

        // A small static rope anchor on the dedicated rope-anchor contact category. The grab query's mask selects this
        // category and ONLY it, so floors/walls (category bit 0) are never grabbed. A solid small collider placed well
        // above the swing arc so the character does not bonk into it while swinging.
        static void AddRopeAnchor(Vector2 position, float radius, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(position.x, position.y, 0f);

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Circle;
            shape.Radius = radius;
            // Override the contact filter so this shape is on the dedicated rope-anchor category bit; it collides with
            // everything (ContactBits = ~0) so it is a real obstacle, but the grab query filters by THIS category.
            shape.OverrideFilterBits = true;
            shape.CategoryBits = 1 << RopeAnchorCategoryBit;
            shape.ContactBits = ~0;

            go.AddComponent<RopeAnchor2DAuthoring>();
        }

        // A trigger-sensor teleporter pad referencing a destination marker. On a character's Enter, TeleporterSystem2D
        // performs a best-effort teleport to the destination's transform.
        static void AddTeleporter(Vector2 center, Vector2 size, GameObject destination, string name)
        {
            var go = new GameObject(name);
            go.transform.position = new Vector3(center.x, center.y, 0f);

            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = new float2(size.x, size.y);
            shape.CollisionResponse = PhysicsCollisionResponse2D.Sensor;

            var teleporter = go.AddComponent<Teleporter2DAuthoring>();
            teleporter.Destination = destination;
        }

        // An empty marker GameObject the teleporter moves the character to. No collider/body — its transform position
        // is all the teleporter baker reads (resolved to its entity, whose LocalToWorld TeleporterSystem2D reads).
        static GameObject AddDestinationMarker(Vector3 position, string name)
        {
            var go = new GameObject(name);
            go.transform.position = position;
            return go;
        }

        // ---- world-space TextMeshPro feature labels ------------------------------------------------------------

        // World-space label font size, in TMP world units (the camera is orthographic at size 8, so the visible
        // frame is ~16 units tall; a size-4 label is legible without dwarfing the 1 x 2 character).
        const float LabelFontSize = 4f;

        // The default light-cyan label colour, legible against the dark (0.12, 0.13, 0.18) camera clear colour.
        static readonly Color LabelColor = new Color(0.85f, 0.95f, 1f, 1f);

        // Places one world-space TextMeshPro label beside each course feature. The world coordinates mirror the
        // SubScene feature positions in BuildChild (the SubScene loads at origin in PlayMode), with each label lifted
        // above its feature so it reads against the background rather than over the geometry.
        static void BuildLabels()
        {
            var labelsRoot = new GameObject("Labels");

            // Near spawn (the character stands at X = 0, floor top Y = 0) — the controls legend.
            SpawnLabel("Move: A/D or ←/→    Jump: Space/W", new Vector3(0f, 4.5f, 0f), labelsRoot.transform);

            // The three friction floor segments (top Y = 0): Ice center X = 9, Sticky center X = 19.
            SpawnLabel("Ice (slippery)", new Vector3(9f, 1.6f, 0f), labelsRoot.transform);
            SpawnLabel("Sticky", new Vector3(19f, 1.6f, 0f), labelsRoot.transform);

            // The step + slope course: low/high steps near X = 26-31, ramps with lips at X = 34 and X = 44.
            SpawnLabel("Step", new Vector3(28.5f, 2.2f, 0f), labelsRoot.transform);
            SpawnLabel("Slope", new Vector3(39f, 3.5f, 0f), labelsRoot.transform);

            // The two moving platforms over the gap (lateral at X = 58, vertical at X = 66).
            SpawnLabel("Moving Platform", new Vector3(62f, 8f, 0f), labelsRoot.transform);

            // The wind/force zone — a sensor with a constant updraft force (WindZone_Updraft, center (73, 11)).
            SpawnLabel("Wind Zone", new Vector3(73f, 14f, 0f), labelsRoot.transform);

            // The rope anchor floating above the wide gap (X = 85, Y = 16) — the explicit grab/release keys.
            SpawnLabel(
                "Rope — press E to grab  (Q/Shift to release)",
                new Vector3(85f, 17.5f, 0f),
                labelsRoot.transform
            );

            // The teleporter sensor pad on the far rope ledge (center (97, 9.5)).
            SpawnLabel("Teleporter", new Vector3(97f, 11.5f, 0f), labelsRoot.transform);
        }

        // Creates one world-space TextMeshPro (3D text, NOT TextMeshProUGUI) GameObject at worldPos, parented under
        // the Labels root. World-space TMP renders through a MeshRenderer, so the orthographic camera sees it in the
        // Game view. The text faces +Z (identity rotation) to read straight-on in the 2D side view, and is centred so
        // worldPos is the label's midpoint above its feature.
        static void SpawnLabel(string text, Vector3 worldPos, Transform parent)
        {
            var go = new GameObject("Label_" + text);
            go.transform.SetParent(parent, worldPositionStays: true);
            go.transform.position = worldPos;
            go.transform.rotation = Quaternion.identity; // face +Z; a flat side-on 2D view reads the XY plane.

            var tmp = go.AddComponent<TextMeshPro>();
            tmp.text = text;
            tmp.fontSize = LabelFontSize;
            tmp.color = LabelColor;
            tmp.alignment = TextAlignmentOptions.Center;
            tmp.textWrappingMode = TextWrappingModes.NoWrap; // one-line labels; the rope label is widest, must not wrap.
        }
    }
}
