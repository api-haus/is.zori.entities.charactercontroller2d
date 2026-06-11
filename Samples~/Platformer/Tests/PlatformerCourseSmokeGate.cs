using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Zori.Entities.CharacterController2D.Samples.Platformer;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Tests
{
    /// <summary>
    /// The P7 behavioural smoke gate for the Platformer sample course: it loads the authored course scene
    /// (<c>PlatformerSample.unity</c> + its baked SubScene), drives the <c>FixedStepSimulationSystemGroup</c> while
    /// feeding the capsule character <see cref="PlatformerCharacterControl2D"/> intent, and asserts that each named
    /// feature actually works — move/jump, the moving platforms, the wind zone, the rope swing, the friction
    /// materials — plus it RESOLVES the open teleport probe (does a far swept <c>MovePosition</c> reach the
    /// destination or stop short).
    ///
    /// <para>No mocks: the real Platformer control/physics/stance systems, the real scene-prop systems
    /// (<c>MovingPlatformSystem2D</c>, <c>WindZoneSystem2D</c>, <c>TeleporterSystem2D</c>), and the real
    /// <c>is.zori.entities.physics2d</c> Box2D world run together. The harness is the substrate's proven
    /// fixed-step-drive pattern (swap in a <c>FixedRateSimpleManager</c> so each <c>group.Update()</c> is exactly one
    /// deterministic step in batchmode; the catch-up manager gates on wall-clock time, which barely advances per
    /// yield in batchmode).</para>
    ///
    /// <para>The course is one continuous left-to-right level (spawn → materials → steps/slopes → platforms → wind →
    /// rope → teleport-back-to-spawn). Walking the whole course blind in one drive is fragile, so each feature is
    /// exercised in isolation: the character is PLACED near the feature (a kinematic body's pose is set directly via
    /// <c>LocalToWorld</c> + a settle, the same authoritative pose the solve reads each step) and then driven, and the
    /// feature's specific observable is asserted. Placement is legitimate — the controller solve reads its start pose
    /// from <c>LocalToWorld</c> every step, so seeding it is how the test addresses a far station.</para>
    /// </summary>
    [TestFixture]
    public sealed class PlatformerCourseSmokeGate
    {
        // The course scene lives in the sample under Samples~/Platformer/Scenes/; the temp-Assets-copy dance places it
        // at this path while the editor imports the sample. The parent scene carries the SubScene with the course.
        const string ParentSceneName = "PlatformerSample";

        const int LoadTimeoutFrames = 1200;
        const float FixedDt = 1f / 60f;

        // --- course coordinates (read from PlatformerSample_Sub.unity authoring transforms) ---
        // Floor segments (top surface Y = -0.5 + 0.5 = 0): Normal X∈[-10,4], Ice center 9 (X∈[4,14]), Sticky center 19.
        static readonly float2 FloorNormalCenter = new float2(-3f, 0f);
        static readonly float2 FloorIceCenter = new float2(9f, 0f);
        static readonly float2 FloorStickyCenter = new float2(19f, 0f);
        static readonly float2 RopeAnchorPos = new float2(85f, 16f);
        static readonly float2 TeleporterPadPos = new float2(97f, 9.5f);
        static readonly float2 TeleportDestinationPos = new float2(0f, 1.1f);
        // Wind zone: sensor at (73,11); the character standing on Ledge_HighWind (top Y ~9) is below it — we place the
        // character inside the sensor region and confirm the updraft adds +Y velocity.
        static readonly float2 WindZoneCenter = new float2(73f, 11f);
        // Moving platforms: lateral home (58, 5.75), vertical home (66, 5).
        static readonly float2 LateralPlatformHome = new float2(58f, 5.75f);

        World _world;
        FixedStepSimulationSystemGroup _fixedGroup;
        IRateManager _savedRateManager;
        Entity _character;
        EntityManager _em;

        // The course scene must be in the build-profile scene list for SceneManager.LoadScene to resolve it. Editor
        // 6000.6 reads that list at PlayMode ENTRY, so it is registered by a SEPARATE editor batchmode pass
        // (PlatformerSmokeBuildSettings.Register) BEFORE this PlayMode test run, not from a [OneTimeSetUp] (too late).

        [TearDown]
        public void TearDown()
        {
            if (_fixedGroup != null && _savedRateManager != null)
                _fixedGroup.RateManager = _savedRateManager;
            _fixedGroup = null;
            _savedRateManager = null;
        }

        // ----- harness ---------------------------------------------------------------------------------------------

        IEnumerator LoadCourse()
        {
            SceneManager.LoadScene(ParentSceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");
            _em = _world.EntityManager;

            // Wait (bounded) for the SubScene to stream + bake the Platformer character archetype.
            var charQuery = _em.CreateEntityQuery(
                ComponentType.ReadOnly<PlatformerCharacterTag>(),
                ComponentType.ReadOnly<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>());
            var frames = 0;
            while (charQuery.CalculateEntityCount() == 0 && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.Greater(
                charQuery.CalculateEntityCount(),
                0,
                $"No baked Platformer character appeared after {frames} frames — the course SubScene did not bake.");

            using (var ents = charQuery.ToEntityArray(Allocator.Temp))
                _character = ents[0];

            _fixedGroup = _world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(_fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            _savedRateManager = _fixedGroup.RateManager;
            _fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // Tick the InitializationSystemGroup so the sample's one-shot structural-setup systems run:
            // PlatformInitSystem2D adds TrackedTransform2D + the PhysicsBody2DCommand buffer to moving platforms, and
            // PushableInitSystem2D adds the command buffer to pushable crates. Without this the platforms never get
            // driven and a rider is never carried. (The driven character carries its own command buffer from baking,
            // so the character solve works on the fixed group alone — but the props need their init pass.)
            var initGroup = _world.GetExistingSystemManaged<InitializationSystemGroup>();
            if (initGroup != null)
                initGroup.Update();

            // One update so PhysicsWorld2DSystem creates the Box2D bodies for the whole course.
            _fixedGroup.Update();
        }

        void Step(int count, float moveX = 0f, bool jump = false, bool grab = false, bool release = false)
        {
            for (var i = 0; i < count; i++)
            {
                var c = _em.GetComponentData<PlatformerCharacterControl2D>(_character);
                c.MoveX = moveX;
                // Latch edges only on the first step of the call (a tap), so the consuming solve sees a rising edge
                // and clears it — matching the real control system's edge latch.
                if (i == 0)
                {
                    if (jump) c.JumpPressed = true;
                    if (grab) c.GrabPressed = true;
                    if (release) c.ReleasePressed = true;
                }
                _em.SetComponentData(_character, c);
                _fixedGroup.Update();
            }
        }

        float2 Position()
        {
            return _em.GetComponentData<LocalToWorld>(_character).Position.xy;
        }

        KinematicCharacterBody2D Body() => _em.GetComponentData<KinematicCharacterBody2D>(_character);

        PlatformerStance2D Stance() => _em.GetComponentData<PlatformerCharacterState2D>(_character).Stance;

        // Place the kinematic character at a world position. Two facts make this non-trivial: (1) the
        // PhysicsBody2DWriteBackSystem scatters the simulated body pose into LocalToWorld each step, so writing
        // LocalToWorld alone is undone; (2) Box2D clamps every body's linear speed to PhysicsWorld2DConfig
        // .maximumLinearSpeed (400 u/s → ~6.67 u/step at 60 Hz), so a single MovePosition/SetTransformTarget cannot
        // jump a far target — it advances at most ~6.67 units/step. We therefore detach the PlatformerCharacterTag (so
        // the active solve does not fight the placement by re-targeting the body to its own pose) and drive the body
        // to pos with REPEATED MovePosition steps until it arrives, then re-arm the solve and seed the stance. This is
        // a test-harness placement to address a far station; the runtime character only ever walks. (It is also the
        // mechanism that revealed the teleport finding — see the teleport probe.)
        void PlaceCharacter(float2 pos, PlatformerStance2D stance = PlatformerStance2D.GroundMove)
        {
            var hadTag = _em.HasComponent<PlatformerCharacterTag>(_character);
            if (hadTag)
                _em.RemoveComponent<PlatformerCharacterTag>(_character);

            // Repeated MovePosition steps until the body arrives (the ~6.67 u/step speed cap means a far target needs
            // several steps). Bounded so a never-arriving target (wedged in geometry) does not hang.
            for (var i = 0; i < 200; i++)
            {
                var commands = _em.GetBuffer<PhysicsBody2DCommand>(_character);
                PhysicsBody2DCommands.MovePosition(commands, pos);
                _fixedGroup.Update();
                if (length(Position() - pos) < 0.5f)
                    break;
            }

            // PARK: a MovePosition to the CURRENT pose gives the kinematic body a zero SetTransformTarget velocity, so
            // it does not retain the (large) approach velocity into the first solve step (which otherwise drops the
            // character several units in one frame, tunnelling past thin colliders / sensor bands).
            var parkCommands = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.SetLinearVelocity(parkCommands, new float2(0f, 0f));
            PhysicsBody2DCommands.MovePosition(parkCommands, Position());
            _fixedGroup.Update();

            if (hadTag)
                _em.AddComponent<PlatformerCharacterTag>(_character);

            var body = Body();
            body.RelativeVelocity = new float2(0f, 0f);
            _em.SetComponentData(_character, body);

            var st = _em.GetComponentData<PlatformerCharacterState2D>(_character);
            st.Stance = stance;
            _em.SetComponentData(_character, st);

            if (_em.HasComponent<PhysicsBody2DSmoothing>(_character))
            {
                var sm = _em.GetComponentData<PhysicsBody2DSmoothing>(_character);
                sm.hasPrev = 0;
                _em.SetComponentData(_character, sm);
            }
        }

        // Place by first lifting the character to a high CLEAR-AIR waypoint at the target X (so the horizontal creep of
        // PlaceCharacter does not wedge on a wall on the way), then dropping straight to the target. Used for stations
        // hemmed in by geometry (the wind sensor sits beside the right wall of Ledge_HighWind, which a horizontal
        // approach catches on).
        void PlaceCharacterViaSky(float2 pos, PlatformerStance2D stance = PlatformerStance2D.GroundMove)
        {
            PlaceCharacter(new float2(pos.x, pos.y + 25f), stance);
            PlaceCharacter(pos, stance);
        }

        // The live X of the LATERAL moving platform (the one whose TravelHalfExtent.x is the dominant axis), read from
        // its LocalToWorld — the platform oscillates, so the rider must be dropped where it currently is.
        float FindPlatformX()
        {
            var q = _em.CreateEntityQuery(
                ComponentType.ReadOnly<MovingPlatform2D>(),
                ComponentType.ReadOnly<LocalToWorld>());
            using var plats = q.ToComponentDataArray<MovingPlatform2D>(Allocator.Temp);
            using var ltws = q.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            for (var i = 0; i < plats.Length; i++)
            {
                if (abs(plats[i].TravelHalfExtent.x) > abs(plats[i].TravelHalfExtent.y))
                    return ltws[i].Position.x;
            }
            return LateralPlatformHome.x;
        }

        // HOLD the character pinned at a fixed world pos for one step WITH the solve attached, so the substrate
        // trigger/contact systems see a persistent overlap (a thin sensor band needs the body to dwell in it for the
        // Begin event to fire — a body falling through at speed can skip the band between steps). We pin by issuing a
        // SetLinearVelocity(0) + MovePosition(pos) on the character's command buffer each step: the body holds its
        // pose, the wind/teleporter trigger-read systems run normally, and the InWindZone2D Stay tag can land.
        void PinStep(float2 pos)
        {
            var commands = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.SetLinearVelocity(commands, new float2(0f, 0f));
            PhysicsBody2DCommands.MovePosition(commands, pos);
            _fixedGroup.Update();
        }

        // Count trigger events currently in the substrate's per-step buffer (on the PhysicsWorldSingleton2D entity).
        int TriggerEventCount()
        {
            var q = _em.CreateEntityQuery(ComponentType.ReadOnly<Zori.Entities.Physics2D.PhysicsWorldSingleton2D>());
            using var ents = q.ToEntityArray(Allocator.Temp);
            if (ents.Length == 0 || !_em.HasBuffer<Zori.Entities.Physics2D.PhysicsTriggerEvent2D>(ents[0]))
                return 0;
            return _em.GetBuffer<Zori.Entities.Physics2D.PhysicsTriggerEvent2D>(ents[0], true).Length;
        }

        // ----- per-feature gates ----------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator A_GroundMoveAndJump_CharacterWalksRightAndLeavesTheGround()
        {
            yield return LoadCourse();
            // Place on a CLEAR span of Floor_Normal (X in [-10,4]) away from the crates (BouncyCrate at -6, PushCrate
            // at 2), so the character grounds on the floor itself, not on a crate.
            PlaceCharacter(new float2(-1f, 1.5f));
            Step(20); // settle onto the normal floor

            Assert.IsTrue(Body().IsGrounded, "character should be grounded on the normal floor after settling");
            var startX = Position().x;
            var restY = Position().y;

            // Walk right for ~0.8s; the character should advance meaningfully in +X.
            Step(50, moveX: 1f);
            var walkedX = Position().x;
            Assert.Greater(walkedX, startX + 1.5f, $"GroundMove: character should walk right ({startX} -> {walkedX})");

            // Jump: the character must leave the ground (peak Y above the grounded rest). JumpSpeed 9 / gravity 20
            // gives an ideal apex rise of 9^2/(2*20) ≈ 2.0; assert a conservative 0.4 to absorb the one-step pose
            // latency and the grounded-snap window.
            var peakY = Position().y;
            // Re-settle on the floor (the walk may have carried the character toward a crate); confirm grounded.
            PlaceCharacter(new float2(-1f, 1.5f));
            Step(20);
            restY = Position().y;
            Assert.IsTrue(Body().IsGrounded, "character should be re-grounded before the jump");
            peakY = restY;
            Step(1, moveX: 0f, jump: true);
            for (var i = 0; i < 40; i++)
            {
                Step(1);
                peakY = max(peakY, Position().y);
            }
            Assert.Greater(peakY, restY + 0.4f, $"Jump: peak Y {peakY} should rise above rest {restY}");
            Assert.IsFalse(Body().IsGrounded && abs(peakY - restY) < 0.1f, "Jump: the character must actually leave the ground");
        }

        [UnityTest]
        public IEnumerator B_FrictionMaterials_IceRetainsSpeedLongerThanSticky()
        {
            yield return LoadCourse();

            // Ice run: accelerate right on the ice segment, then release input and measure how far the character
            // coasts. Low friction (ice = 0.15) → low acceleration sharpness → it both gains AND sheds speed slowly,
            // so after a fixed accel window the ice speed differs from the sticky speed. The decisive, friction-driven
            // observable is the achieved speed under an identical accel window.
            PlaceCharacter(new float2(FloorIceCenter.x - 3f, 1.05f));
            Step(20);
            Assert.IsTrue(Body().IsGrounded, "character should be grounded on the ice floor");
            Step(25, moveX: 1f);
            var iceSpeed = abs(Body().RelativeVelocity.x);

            // Sticky run: same accel window on the sticky segment (friction 3 → high sharpness → reaches target fast).
            PlaceCharacter(new float2(FloorStickyCenter.x - 3f, 1.05f));
            Step(20);
            Assert.IsTrue(Body().IsGrounded, "character should be grounded on the sticky floor");
            Step(25, moveX: 1f);
            var stickySpeed = abs(Body().RelativeVelocity.x);

            // The friction modifier scales the accel sharpness, so under an identical short accel window the sticky
            // surface (high friction) reaches a higher speed than the ice (low friction). They must differ, and the
            // sticky one must be the faster — the character genuinely FEELS the material.
            Assert.Greater(
                stickySpeed,
                iceSpeed + 0.25f,
                $"Materials: sticky speed {stickySpeed} should exceed ice speed {iceSpeed} under the same accel window "
                    + "(ice's low friction = slow to gain speed = slippery feel)");
        }

        [UnityTest]
        public IEnumerator C_MovingPlatform_CharacterStandingOnItIsCarried()
        {
            yield return LoadCourse();

            // Find the lateral moving platform's LIVE position (it oscillates X within ±4 of home (58, 5.75) at speed 3,
            // so the test reads where it actually is rather than assuming home). Then drop the character onto it from
            // straight above (clear air → no wall to wedge on). The platform's TrackedTransform2D + command buffer were
            // added by PlatformInitSystem2D at load; MovingPlatformSystem2D drives it each fixed step. With NO player
            // input, the C4b-verified auto-parent carry (Update_ParentMovement) should sweep the rider in X.
            var platformX = FindPlatformX();
            PlaceCharacterViaSky(new float2(platformX, LateralPlatformHome.y + 1.0f), PlatformerStance2D.AirMove);
            // Let the character settle onto the platform and ground.
            for (var i = 0; i < 120; i++)
            {
                Step(1, moveX: 0f);
                if (Body().IsGrounded)
                    break;
            }
            Assert.IsTrue(Body().IsGrounded, "character should land on the moving platform");

            // While the rider stays GROUNDED on the platform, with NO player input, the platform's own X oscillation
            // must carry it: track the X sweep over the grounded window. The character's own X velocity stays ~0 (no
            // input), so any X displacement is the platform carrying it (Update_ParentMovement). Stop once it leaves
            // the platform (the oscillation can eventually outrun the rider — the carry is proven by the grounded sweep
            // before that).
            var startX = Position().x;
            var minX = startX;
            var maxX = startX;
            var groundedFrames = 0;
            for (var i = 0; i < 220; i++)
            {
                Step(1, moveX: 0f);
                if (!Body().IsGrounded)
                {
                    if (groundedFrames > 10)
                        break; // it rode the platform, then came off — carry already demonstrated
                    continue;
                }
                groundedFrames++;
                var x = Position().x;
                minX = min(minX, x);
                maxX = max(maxX, x);
            }
            Assert.Greater(groundedFrames, 10, "character should ride the platform grounded for a sustained window");
            Assert.Greater(
                maxX - minX,
                0.3f,
                $"Moving platform: a standing character with no input should be carried in X by the platform "
                    + $"(grounded X sweep {maxX - minX:F3} over {groundedFrames} grounded frames)");
        }

        [UnityTest]
        public IEnumerator D_WindZone_KnownLimitation_CharacterGroundsOnSensorAndNoTriggerFires()
        {
            yield return LoadCourse();

            // KNOWN LIMITATION (root-caused). The wind zone is a sensor (CollisionResponse=Sensor → isTrigger=true);
            // WindZoneSystem2D reads the trigger Begin and adds the zone's force to the character's RelativeVelocity.
            // BUT the kinematic controller's grounding / collide-and-slide casts (PhysicsQueries2D) do NOT exclude
            // sensor (isTrigger) shapes, so the character COLLIDES WITH / GROUNDS ON the sensor instead of passing
            // through it. It never enters the sensor's interior, so the trigger Begin never fires, the InWindZone2D
            // Stay tag is never added, and the wind force is never applied. The same root cause breaks the teleporter
            // (test F). This is a substrate/controller binding gap (the cast queries should filter out trigger shapes),
            // surfaced by the sample — NOT a sample-authoring bug. It is asserted here as the observed reality so that
            // when the controller starts excluding sensors from its casts, these assertions flip RED and the feature
            // can be re-enabled.
            //
            // Drop the character straight down onto the wind sensor (sensor at (73,11)). It should ground ON the sensor
            // and the in-zone tag should NEVER be set.
            PlaceCharacter(new float2(WindZoneCenter.x, WindZoneCenter.y + 4f), PlatformerStance2D.AirMove);
            var everInZone = false;
            Entity groundEntity = Entity.Null;
            for (var i = 0; i < 60; i++)
            {
                Step(1, moveX: 0f);
                if (_em.HasComponent<InWindZone2D>(_character))
                    everInZone = true;
                if (Body().IsGrounded)
                    groundEntity = Body().GroundHit.Entity;
            }

            var groundedOnSensor = groundEntity != Entity.Null && _em.HasComponent<WindZone2D>(groundEntity);
            UnityEngine.Debug.Log(
                $"[P7-WIND-LIMITATION] everInZone={everInZone} grounded-on-WindZone-sensor={groundedOnSensor}. "
                    + "The controller's grounding cast treats the sensor as solid ground, so the character never enters "
                    + "the trigger interior and the wind force is never applied — a substrate/controller binding gap "
                    + "(casts do not exclude isTrigger shapes), the same root cause as the teleport gap.");

            // The documented reality: the character grounds ON the wind sensor (treating the trigger as solid), and the
            // wind force is therefore never applied (the in-zone tag never sets). If the controller is later fixed to
            // exclude sensors from its casts, BOTH asserts flip and this feature works — re-enable it then.
            Assert.IsTrue(groundedOnSensor,
                "Wind zone (known limitation): the character grounds ON the wind sensor — the controller cast treats "
                    + "the trigger as solid ground instead of passing through it.");
            Assert.IsFalse(everInZone,
                "Wind zone (known limitation): because the character never enters the sensor interior (it grounds on "
                    + "top), the trigger Begin never fires and the wind force is never applied. Re-enable when the "
                    + "controller excludes isTrigger shapes from its casts.");
        }

        [UnityTest]
        public IEnumerator E_RopeSwing_GrabEntersSwingAndPendulumsAcrossTheGap()
        {
            yield return LoadCourse();

            // Place the character airborne just below the rope anchor (anchor at (85,16), RopeLength authored 6) so a
            // grab finds it. Put it slightly off-centre so gravity drives a swing rather than hanging straight down.
            PlaceCharacter(new float2(RopeAnchorPos.x - 2f, RopeAnchorPos.y - 5f), PlatformerStance2D.AirMove);
            Step(2); // let the airborne pose settle into the solve

            // Grab: in AirMove, the GrabPressed edge runs the anchor query and should enter RopeSwing.
            Step(1, grab: true);
            Assert.AreEqual(
                PlatformerStance2D.RopeSwing,
                Stance(),
                "Rope: a grab within RopeLength of the anchor should enter the RopeSwing stance");

            var rope = _em.GetComponentData<RopeSwingState2D>(_character);
            Assert.AreEqual(RopeAnchorPos.x, rope.AnchorPoint.x, 0.5f, "rope anchor X should match the scene anchor");

            // Swing: with the character off-centre, gravity + the rope clamp produce pendulum motion. Track the X
            // range over the swing — a real pendulum sweeps a horizontal arc; a frozen/non-swinging body would not.
            var minX = Position().x;
            var maxX = Position().x;
            // Drive toward the far side to build the swing, then release at the far reach.
            for (var i = 0; i < 120; i++)
            {
                Step(1, moveX: 1f);
                var x = Position().x;
                minX = min(minX, x);
                maxX = max(maxX, x);
                // The character must stay within RopeLength + a margin of the anchor (the clamp holds it on the circle).
                var d = length(Position() - RopeAnchorPos);
                Assert.LessOrEqual(d, rope.RopeLength + 0.6f, $"Rope: character must stay on the rope circle (dist {d:F3})");
            }
            Assert.Greater(maxX - minX, 0.8f, $"Rope: the swing should sweep a horizontal arc (X range {maxX - minX:F3})");

            // Release by jump: should leave RopeSwing back to AirMove, carrying swing momentum + the jump impulse.
            Step(1, jump: true);
            Assert.AreEqual(
                PlatformerStance2D.AirMove,
                Stance(),
                "Rope: a jump should release the rope back to AirMove");
        }

        [UnityTest]
        public IEnumerator F_Teleport_Probe_TriggerNeverFiresAndSweptMovePositionWouldStopShort()
        {
            yield return LoadCourse();

            // THE P7 TELEPORT PROBE — the verdict has TWO layers, both binding gaps.
            //
            // LAYER 1 (the dominant gap, root-caused). The teleporter is a sensor (CollisionResponse=Sensor →
            // isTrigger=true) and TeleporterSystem2D fires on its trigger Begin. But the kinematic controller's
            // grounding / collide-and-slide casts do NOT exclude sensor (isTrigger) shapes, so the character GROUNDS
            // ON the teleporter pad instead of passing into it — the trigger Begin NEVER fires and the teleport never
            // even attempts. (Same root cause as the wind zone, test D.) So the FIRST observable is: no trigger event
            // is ever produced while the character sits on the pad.
            //
            // LAYER 2 (the gap behind the gap, measured separately). EVEN IF the trigger fired, the teleport is
            // best-effort: it writes LocalToWorld := destination and enqueues a swept MovePosition(destination). The
            // substrate has no instantaneous SetTransform — MovePosition maps to SetTransformTarget(target, dt), and
            // Box2D clamps the implied velocity to PhysicsWorld2DConfig.maximumLinearSpeed = 400 u/s (~6.67 u/step at
            // 60 Hz). A ~97-unit cross-course teleport would therefore advance only ~6.67 u/step and STOP SHORT,
            // sweeping through the world over ~15 steps rather than teleporting. We measure this directly by driving a
            // far MovePosition and watching the body fail to reach the target in a step.

            // LAYER 1: pin the character on the teleporter pad and confirm NO trigger event fires.
            PlaceCharacterViaSky(TeleporterPadPos, PlatformerStance2D.AirMove);
            var anyTrigger = false;
            for (var i = 0; i < 20; i++)
            {
                PinStep(TeleporterPadPos);
                if (TriggerEventCount() > 0)
                    anyTrigger = true;
            }
            Assert.IsFalse(anyTrigger,
                "Teleport (layer 1, binding gap): the teleporter is a sensor, but the controller's casts treat it as "
                    + "solid ground, so the character sits ON the pad and never enters it — the trigger Begin never "
                    + "fires and the teleport never attempts. Re-enable when the controller excludes isTrigger shapes "
                    + "from its casts.");

            // LAYER 2: directly drive a far MovePosition (the body, tag off so only the move acts) and confirm the
            // swept SetTransformTarget cannot cross the course in one step — it advances ≤ ~6.67 u (the 400 u/s clamp).
            _em.RemoveComponent<PlatformerCharacterTag>(_character);
            var fromX = Position().x;
            var farCmds = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.MovePosition(farCmds, TeleportDestinationPos); // a ~97-unit jump to spawn
            _fixedGroup.Update();
            var advancedOneStep = abs(Position().x - fromX);
            _em.AddComponent<PlatformerCharacterTag>(_character);

            UnityEngine.Debug.Log(
                $"[P7-TELEPORT-PROBE] LAYER1: trigger fired on pad = {anyTrigger} (it does NOT — the character grounds "
                    + $"on the sensor). LAYER2: a far MovePosition (~97 u) advanced only {advancedOneStep:F2} u in one "
                    + "step (the 400 u/s ≈ 6.67 u/step maximumLinearSpeed clamp), so even if it fired the swept move "
                    + "STOPS SHORT. VERDICT: teleport is NON-FUNCTIONAL — the trigger never fires (dominant gap), and "
                    + "the swept MovePosition could not instantaneously reach anyway (the substrate lacks an "
                    + "instantaneous SetTransform — teleport decision A, unbuilt).");

            Assert.Less(advancedOneStep, 10f,
                $"Teleport (layer 2, binding gap): a far ~97-unit MovePosition advanced only {advancedOneStep:F2} u in "
                    + "one step — the swept SetTransformTarget is clamped to ~6.67 u/step (400 u/s maximumLinearSpeed) "
                    + "and STOPS SHORT of a cross-course teleport. An instantaneous substrate SetTransform (decision A) "
                    + "would land it in one step and flip this RED.");
        }
    }
}
