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
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var frames = 0;
            while (charQuery.CalculateEntityCount() == 0 && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.Greater(
                charQuery.CalculateEntityCount(),
                0,
                $"No baked Platformer character appeared after {frames} frames — the course SubScene did not bake."
            );

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
                    if (jump)
                        c.JumpPressed = true;
                    if (grab)
                        c.GrabPressed = true;
                    if (release)
                        c.ReleasePressed = true;
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
                ComponentType.ReadOnly<LocalToWorld>()
            );
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
            Assert.IsFalse(
                Body().IsGrounded && abs(peakY - restY) < 0.1f,
                "Jump: the character must actually leave the ground"
            );
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
                    + "(ice's low friction = slow to gain speed = slippery feel)"
            );
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
                    + $"(grounded X sweep {maxX - minX:F3} over {groundedFrames} grounded frames)"
            );
        }

        [UnityTest]
        public IEnumerator D_WindZone_CharacterPassesIntoSensorAndUpdraftAppliesForce()
        {
            yield return LoadCourse();

            // The wind zone is a sensor (CollisionResponse=Sensor → isTrigger=true); WindZoneSystem2D reads the
            // trigger Begin and adds the zone's force to the character's RelativeVelocity while inside. Now that the
            // controller's grounding / collide-and-slide casts EXCLUDE sensor (isTrigger) shapes (the sensor fix), the
            // character passes INTO the sensor as a visitor instead of grounding on it: the trigger Begin fires, the
            // InWindZone2D Stay tag lands, and the updraft applies. This test was the P7 known-limitation case; it now
            // asserts the working behaviour — the sensor fix flipped it from RED to a real feature gate.
            //
            // Place the character ABOVE the wind sensor (sensor at (73,11)) and let it FALL through under gravity. A
            // clean not-touching → touching TRANSITION as it enters the sensor is what fires the substrate trigger
            // Begin (warping the body already-overlapping the sensor produces no transition, hence no Begin — the
            // substrate's begin-on-entry semantics, mirrored from the substrate's own Trigger_BodyPassesThroughSensor
            // test). Now that the controller's casts exclude the isTrigger sensor, the character does not ground on it
            // — it falls through it as a visitor, the Begin fires, and the updraft applies.
            // The decisive, force-driven observable is the updraft ARRESTING the fall (not an absolute upward
            // velocity — during a brief fast pass-through gravity, −20·dt/step, can outweigh the +14·dt/step
            // updraft in absolute terms). We measure it as a controlled A/B over the same fall window, varying ONLY
            // the wind (negative-space measurement rule): the character falls through the sensor WITH the wind zone
            // applied, then the wind zone's force is zeroed and the SAME fall is repeated; the with-wind descent must
            // be measurably slower (a less-negative vertical velocity) than the no-wind free-fall.
            var startWindFall = new float2(WindZoneCenter.x, WindZoneCenter.y + 6f);

            // --- WITH wind: fall through the sensor; record the most-positive (least-negative) vertical velocity
            //     observed while inside the zone, and confirm the character enters the sensor without grounding on it.
            PlaceCharacterViaSky(startWindFall, PlatformerStance2D.AirMove);
            var everInZone = false;
            var groundedOnSensor = false;
            var bestVyWithWind = float.MinValue;
            for (var i = 0; i < 80; i++)
            {
                Step(1, moveX: 0f);
                if (_em.HasComponent<InWindZone2D>(_character))
                {
                    everInZone = true;
                    bestVyWithWind = max(bestVyWithWind, Body().RelativeVelocity.y);
                }
                if (Body().IsGrounded && _em.HasComponent<WindZone2D>(Body().GroundHit.Entity))
                    groundedOnSensor = true;
            }

            // --- NO wind (control): zero every wind zone's force, repeat the identical fall, and record the
            //     least-negative vertical velocity over the matching in-volume window (the character is now never
            //     tagged in-zone — the force is zero — so sample over the same Y band the sensor occupies).
            using (var q = _em.CreateEntityQuery(ComponentType.ReadWrite<WindZone2D>()))
            using (var zoneEnts = q.ToEntityArray(Allocator.Temp))
            {
                foreach (var z in zoneEnts)
                    _em.SetComponentData(z, new WindZone2D { Force = new float2(0f, 0f) });
            }
            PlaceCharacterViaSky(startWindFall, PlatformerStance2D.AirMove);
            var bestVyNoWind = float.MinValue;
            var sensorTop = WindZoneCenter.y + 1f;
            var sensorBottom = WindZoneCenter.y - 1f;
            for (var i = 0; i < 80; i++)
            {
                Step(1, moveX: 0f);
                var y = Position().y;
                if (y <= sensorTop && y >= sensorBottom)
                    bestVyNoWind = max(bestVyNoWind, Body().RelativeVelocity.y);
            }

            UnityEngine.Debug.Log(
                $"[P7-WIND] everInZone={everInZone} groundedOnSensor={groundedOnSensor} "
                    + $"bestVyWithWind={bestVyWithWind:F3} bestVyNoWind={bestVyNoWind:F3}. The controller now excludes "
                    + "the isTrigger sensor from its casts, so the character enters the trigger interior — the wind "
                    + "Begin fires, InWindZone2D is set, and the updraft arrests the fall relative to free-fall."
            );

            // The sensor fix: the character must NOT ground on the wind sensor (it passes through as a visitor)…
            Assert.IsFalse(
                groundedOnSensor,
                "Wind zone: the character must NOT ground on the wind sensor — the controller casts now exclude "
                    + "isTrigger shapes, so it passes into the sensor as a visitor rather than standing on it."
            );
            // …the trigger Begin therefore fires and the zone tags the character…
            Assert.IsTrue(
                everInZone,
                "Wind zone: the character must enter the sensor interior (InWindZone2D set) — the trigger Begin fires "
                    + "now that the controller passes through the sensor instead of grounding on it."
            );
            // …and the updraft applies its +Y force: the with-wind descent through the sensor is measurably slower
            // (a less-negative vertical velocity) than the no-wind free-fall over the same band.
            Assert.Greater(
                bestVyWithWind,
                bestVyNoWind + 0.1f,
                $"Wind zone: the updraft must arrest the fall — the in-zone vertical velocity with wind "
                    + $"({bestVyWithWind:F3}) must exceed the no-wind free-fall ({bestVyNoWind:F3}) over the same band."
            );
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
                "Rope: a grab within RopeLength of the anchor should enter the RopeSwing stance"
            );

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
                Assert.LessOrEqual(
                    d,
                    rope.RopeLength + 0.6f,
                    $"Rope: character must stay on the rope circle (dist {d:F3})"
                );
            }
            Assert.Greater(
                maxX - minX,
                0.8f,
                $"Rope: the swing should sweep a horizontal arc (X range {maxX - minX:F3})"
            );

            // Release by jump: should leave RopeSwing back to AirMove, carrying swing momentum + the jump impulse.
            Step(1, jump: true);
            Assert.AreEqual(
                PlatformerStance2D.AirMove,
                Stance(),
                "Rope: a jump should release the rope back to AirMove"
            );
        }

        [UnityTest]
        public IEnumerator F_Teleport_CharacterEntersPadAndArrivesAtDestinationAnyDistanceNoStreak()
        {
            yield return LoadCourse();

            // THE TELEPORT GATE — now a PROPER instantaneous teleport (SetTransform + SkipInterpolation), the 2D
            // mirror of the 3D Platformer sample's TeleporterSystem. The prior best-effort form (a swept
            // MovePosition clamped to ~6.67 u/step by maximumLinearSpeed) STOPPED SHORT of a far cross-course
            // teleport; TeleporterSystem2D now uses the substrate's instantaneous SetTransform, so the character
            // ARRIVES at the destination in one step at any distance, with no interpolation streak.
            //
            // The gate proves two things: (1) the trigger fires on the pad and the character ARRIVES at the
            // destination (the end-to-end teleporter path), and (2) the substrate SetTransform lands a far
            // destination in ONE step (the underlying mechanism, isolated).

            // (1) End-to-end: place the character ABOVE the teleporter pad and let it FALL through. A clean
            // not-touching → touching transition as it enters the pad sensor fires the substrate trigger Begin (the
            // controller's casts exclude the isTrigger pad, so the character passes INTO it as a visitor rather than
            // grounding on it). TeleporterSystem2D reads that Begin and teleports the character to TeleportDestination.
            var padX = TeleporterPadPos.x;
            PlaceCharacterViaSky(new float2(TeleporterPadPos.x, TeleporterPadPos.y + 6f), PlatformerStance2D.AirMove);
            var anyTrigger = false;
            var groundedOnPad = false;
            var arrived = false;
            for (var i = 0; i < 120; i++)
            {
                Step(1, moveX: 0f);
                if (TriggerEventCount() > 0)
                    anyTrigger = true;
                if (Body().IsGrounded && _em.HasComponent<Teleporter2D>(Body().GroundHit.Entity))
                    groundedOnPad = true;
                // The character ARRIVES at the destination after the teleporter fires (the SetTransform lands it
                // there in one step; the next solve reads the LocalToWorld := destination and stays). Distance from
                // the pad (~97 u away) to the destination is far past any swept per-step ceiling.
                if (length(Position() - TeleportDestinationPos) < 2.0f)
                {
                    arrived = true;
                    break;
                }
            }
            Assert.IsFalse(
                groundedOnPad,
                "Teleport: the character must NOT ground on the teleporter pad sensor — the controller casts exclude "
                    + "isTrigger shapes, so it passes into the pad as a visitor and the trigger Begin fires."
            );
            Assert.IsTrue(
                anyTrigger,
                "Teleport: a trigger Begin must fire while the character is in the teleporter pad sensor."
            );
            Assert.IsTrue(
                arrived,
                $"Teleport: the character must ARRIVE at the destination {TeleportDestinationPos} after entering the "
                    + $"pad (it was at {Position()}). The far cross-course teleport must complete — SetTransform is "
                    + "instantaneous and unclamped, so the destination is reached in one step at any distance."
            );

            // (2) Mechanism, isolated: directly drive a far SetTransform (tag off so only the command acts, no solve
            // re-target) and confirm the instantaneous set crosses the whole course in ONE step — past the ~6.67
            // u/step swept ceiling the old MovePosition was clamped to. This is the substrate fact the teleporter
            // now stands on.
            _em.RemoveComponent<PlatformerCharacterTag>(_character);
            var farTarget = new float2(padX, TeleporterPadPos.y); // re-cross the course back to the pad in one step
            var fromX = Position().x;
            var crossDistance = abs(farTarget.x - fromX);
            var farCmds = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.SetTransform(farCmds, farTarget, 0f);
            _fixedGroup.Update();
            var landed = Position();
            _em.AddComponent<PlatformerCharacterTag>(_character);

            UnityEngine.Debug.Log(
                $"[TELEPORT-GATE] End-to-end: trigger fired on pad = {anyTrigger}, grounded-on-pad = {groundedOnPad}, "
                    + $"ARRIVED at destination = {arrived}. Mechanism: a far SetTransform crossing {crossDistance:F1} u "
                    + $"landed at {landed} (target {farTarget}) in ONE step — instantaneous and unclamped (no ~6.67 "
                    + "u/step maximumLinearSpeed limit). VERDICT: proper teleport, mirrors the 3D sample."
            );

            Assert.Less(
                length(landed - farTarget),
                0.5f,
                $"Teleport mechanism: a far SetTransform crossing ~{crossDistance:F1} u must land at the target "
                    + $"{farTarget} in ONE step (landed {landed}). The instantaneous set bypasses the swept "
                    + "maximumLinearSpeed clamp, so distance does not matter."
            );
        }

        [UnityTest]
        public IEnumerator G_Respawn_FallBelowThreshold_TeleportsBackToLastSafePoint_VelocityZeroed()
        {
            yield return LoadCourse();

            // Stand on a clear span of the normal floor (top Y = 0) and settle so PlatformerRespawnSystem records a
            // safe point: it records only while grounded AND grounded the prior step (a sustained stand, not a graze).
            PlaceCharacter(new float2(-3f, 1.1f));
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "Respawn: the character must be grounded so a safe point is recorded.");
            var safe = _em.GetComponentData<LastSafePoint2D>(_character);
            Assert.IsTrue(
                safe.HasPoint,
                "Respawn: a safe point must be recorded after standing grounded on the floor."
            );
            var safeX = safe.Position.x;

            // Fall off the course: drive the body far below the fall threshold (tuned at -15). Tag off so only the
            // placement acts (no solve re-target), then re-arm and step — the respawn system reads Y < threshold and
            // teleports the character back to the recorded safe point.
            _em.RemoveComponent<PlatformerCharacterTag>(_character);
            var sinkCmds = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.SetLinearVelocity(sinkCmds, new float2(3f, -20f)); // falling with momentum
            PhysicsBody2DCommands.SetTransform(sinkCmds, new float2(-3f, -40f), 0f);
            _fixedGroup.Update();
            _em.AddComponent<PlatformerCharacterTag>(_character);
            Assert.Less(
                Position().y,
                -15f,
                $"Respawn: the character must be below the fall threshold before the respawn fires (was {Position()})."
            );

            // Step until the respawn lands the character back on the safe surface (above the threshold), then settle.
            var respawned = false;
            for (var i = 0; i < 60; i++)
            {
                Step(1);
                if (Position().y > -1f)
                {
                    respawned = true;
                    break;
                }
            }
            Assert.IsTrue(
                respawned,
                $"Respawn: the character must teleport back above the course floor after falling (was at {Position()})."
            );
            Step(20); // settle onto the safe surface
            Assert.AreEqual(
                safeX,
                Position().x,
                1.0f,
                $"Respawn: the character must land back at the last safe point's X ({safeX}); was at {Position()}."
            );
            Assert.IsTrue(
                Body().IsGrounded,
                "Respawn: after teleporting back the character should re-ground on the safe surface."
            );
            Assert.Less(
                length(Body().RelativeVelocity),
                1.0f,
                $"Respawn: velocity must be zeroed by the teleport (was {Body().RelativeVelocity})."
            );
        }

        // ---- jump-buffer window (Task 1) -------------------------------------------------------------------------
        //
        // The jump buffer is a TIME WINDOW (PlatformerCharacterTuning2D.JumpBufferTime, default 0.15s of fixed-step
        // time), not the old unbounded latch. A press is buffered when fresh; on a grounded step the buffered jump
        // fires only while it is still within the window of the press, otherwise it expires unfired. These gates drive
        // the buffer's decision points directly: a press too early does NOT re-jump on landing; a press within the
        // window DOES; and a held button (no fresh edge) never perpetually re-buffers.

        // Apply ONE fresh jump-press edge (rising edge), exactly as the control system does on wasPressedThisFrame:
        // set JumpBufferElapsedTime via the same path the solve would — by latching JumpPressed for one fixed step so
        // the solve stamps it from the simulation clock. Returns after the single stamping step.
        void PressJumpOnce()
        {
            var c = _em.GetComponentData<PlatformerCharacterControl2D>(_character);
            c.JumpPressed = true;
            _em.SetComponentData(_character, c);
            _fixedGroup.Update();
        }

        [UnityTest]
        public IEnumerator H_JumpBuffer_PressWellBeforeLanding_DoesNotRejump()
        {
            yield return LoadCourse();

            // Settle grounded on a clear span of the normal floor (top Y = 0).
            PlaceCharacter(new float2(-1f, 1.1f));
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "must be grounded before the test jump");
            var restY = Position().y;

            // Jump once. The character leaves the ground.
            PressJumpOnce();
            Assert.IsFalse(Body().IsGrounded, "the initial jump must leave the ground");

            // While airborne and well past the 0.15s buffer window (default JumpBufferTime), press jump ONCE more.
            // Step ~12 frames first (0.2s > 0.15s) so the press is OLD by the time the character lands.
            Step(12);
            PressJumpOnce(); // this press is stamped, but landing is still many frames away
            // Let the character finish its fall and land. The landing happens well after JumpBufferTime, so the
            // buffered press has EXPIRED and must NOT trigger a second jump.
            var landed = false;
            var maxYAfterLand = float.MinValue;
            for (var i = 0; i < 120; i++)
            {
                Step(1);
                if (Body().IsGrounded)
                {
                    landed = true;
                    // Once grounded, track the peak Y over the next steps — a re-jump would lift it well above rest.
                    for (var j = 0; j < 30; j++)
                    {
                        Step(1);
                        maxYAfterLand = max(maxYAfterLand, Position().y);
                    }
                    break;
                }
            }
            Assert.IsTrue(landed, "the character must land back on the floor");
            Assert.Less(
                maxYAfterLand,
                restY + 0.35f,
                $"Jump buffer: a press {0.2f}s before landing has EXPIRED (> JumpBufferTime 0.15s) and must NOT "
                    + $"re-jump on landing — peak Y after landing {maxYAfterLand:F3} should stay near rest {restY:F3}."
            );
            Assert.IsTrue(
                Body().IsGrounded,
                "after landing with no fresh in-window press the character stays grounded"
            );
        }

        [UnityTest]
        public IEnumerator H_JumpBuffer_PressWithinWindowBeforeLanding_DoesRejump()
        {
            yield return LoadCourse();

            PlaceCharacter(new float2(-1f, 1.1f));
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "must be grounded before the test jump");
            var restY = Position().y;

            // Jump once; the character leaves the ground.
            PressJumpOnce();
            Assert.IsFalse(Body().IsGrounded, "the initial jump must leave the ground");

            // Fall back down WITHOUT pressing, until the character is about to land (one step from grounded). The
            // descent from a JumpSpeed-9 / gravity-20 hop takes well over 0.15s, so we wait until close to the ground,
            // then press within the window so the buffered jump fires on the imminent grounded step.
            var armed = false;
            for (var i = 0; i < 120; i++)
            {
                // Approaching the floor (descending and low): press jump so it is fresh (within JumpBufferTime) when
                // the next step grounds the character.
                if (Body().RelativeVelocity.y < 0f && Position().y < restY + 0.6f)
                {
                    PressJumpOnce();
                    armed = true;
                    break;
                }
                Step(1);
            }
            Assert.IsTrue(armed, "the descent must reach the pre-landing band so the in-window press can be issued");

            // Now let it ground and re-launch: the in-window buffered press must fire a fresh jump on landing, lifting
            // the character above rest again.
            var peakY = restY;
            for (var i = 0; i < 60; i++)
            {
                Step(1);
                peakY = max(peakY, Position().y);
            }
            Assert.Greater(
                peakY,
                restY + 0.4f,
                $"Jump buffer: a press within JumpBufferTime (0.15s) of landing MUST fire the buffered jump on landing "
                    + $"— peak Y {peakY:F3} should rise above rest {restY:F3}."
            );
        }

        [UnityTest]
        public IEnumerator H_JumpBuffer_HeldButton_DoesNotPerpetuallyRejump()
        {
            yield return LoadCourse();

            PlaceCharacter(new float2(-1f, 1.1f));
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "must be grounded before the held-jump test");
            var restY = Position().y;

            // Simulate HOLDING the jump button: the real control system sets JumpPressed only on the rising edge
            // (wasPressedThisFrame), so a held key produces exactly ONE fresh edge and then nothing. Reproduce that:
            // one stamping step, then NO further presses — never re-latch JumpPressed. The character should jump once,
            // then settle back on the floor and STAY there (no perpetual re-jump from the held button).
            PressJumpOnce();
            Assert.IsFalse(Body().IsGrounded, "the single edge from the (held) press must jump once");

            // Drive many steps with NO further press edge (the held key re-stamps nothing). After the hop, the
            // character must settle grounded and stay grounded — count the airborne→grounded transitions; a held-button
            // auto-rejump bug would show repeated launches (many transitions / a never-settling body).
            var launches = 0;
            var wasGrounded = false;
            var settledGroundedRun = 0;
            for (var i = 0; i < 200; i++)
            {
                Step(1);
                var g = Body().IsGrounded;
                if (!g && wasGrounded)
                    launches++; // a fresh leave-the-ground = a launch
                wasGrounded = g;
                settledGroundedRun = g ? settledGroundedRun + 1 : 0;
            }
            Assert.LessOrEqual(
                launches,
                0,
                $"Jump buffer: a HELD button (one rising edge only) must NOT perpetually re-jump — after the first hop "
                    + $"there should be no further launches, but counted {launches} additional leave-the-ground events."
            );
            Assert.Greater(
                settledGroundedRun,
                30,
                "Jump buffer: after the single (held) jump the character must settle grounded and stay grounded, not "
                    + "bounce repeatedly."
            );
            Assert.Less(
                abs(Position().y - restY),
                0.3f,
                $"Jump buffer: a held button should leave the character resting on the floor (Y {Position().y:F3} ≈ "
                    + $"rest {restY:F3}), not airborne from a perpetual re-jump."
            );
        }

        // ---- moving platform is NOT a safe respawn point (Task 2) ------------------------------------------------
        //
        // PlatformerRespawnSystem records the last safe (grounded, stable) position, but a MOVING PLATFORM is not a
        // stable anchor — its pose travels. This gate stands the character on the lateral moving platform, confirms no
        // safe point is recorded while it rides (the GroundHit is a MovingPlatform2D body, excluded from the predicate),
        // then drops it onto the STATIC floor, confirms a safe point IS recorded there, and finally confirms a fall
        // respawns to that STATIC point — never to the platform.

        [UnityTest]
        public IEnumerator I_Respawn_MovingPlatformIsNotASafePoint()
        {
            yield return LoadCourse();

            // (a) Ride the lateral moving platform. Drop onto its live position from clear air, let it ground, and ride
            // for a sustained grounded window. While grounded on the platform, NO safe point may be recorded (the
            // platform's GroundHit is a MovingPlatform2D body, excluded). Crucially the character has NEVER stood on a
            // static surface yet in this run, so HasPoint must stay false the whole ride.
            var platformX = FindPlatformX();
            PlaceCharacterViaSky(new float2(platformX, LateralPlatformHome.y + 1.0f), PlatformerStance2D.AirMove);
            var groundedOnPlatformFrames = 0;
            var recordedWhileOnPlatform = false;
            for (var i = 0; i < 220; i++)
            {
                Step(1, moveX: 0f);
                var b = Body();
                if (b.IsGrounded && _em.HasComponent<MovingPlatform2D>(b.GroundHit.Entity))
                {
                    groundedOnPlatformFrames++;
                    if (_em.GetComponentData<LastSafePoint2D>(_character).HasPoint)
                        recordedWhileOnPlatform = true;
                }
                if (groundedOnPlatformFrames > 25)
                    break;
            }
            Assert.Greater(
                groundedOnPlatformFrames,
                10,
                "Respawn/platform: the character must ride the moving platform grounded for a sustained window."
            );
            Assert.IsFalse(
                recordedWhileOnPlatform,
                "Respawn/platform: NO safe point may be recorded while grounded on the moving platform — a travelling "
                    + "surface is not a stable respawn anchor (the predicate excludes a MovingPlatform2D ground hit)."
            );

            // (b) Now stand on the STATIC normal floor and settle; a safe point MUST be recorded there.
            PlaceCharacter(new float2(-3f, 1.1f));
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "Respawn/platform: must ground on the static floor.");
            var staticGround = _em.HasComponent<MovingPlatform2D>(Body().GroundHit.Entity);
            Assert.IsFalse(staticGround, "the normal floor must NOT be a moving platform (sanity).");
            var safe = _em.GetComponentData<LastSafePoint2D>(_character);
            Assert.IsTrue(
                safe.HasPoint,
                "Respawn/platform: a safe point MUST be recorded after standing on the static floor."
            );
            var staticSafeX = safe.Position.x;

            // (c) Fall off the course; the respawn must send the character back to the STATIC safe point, not anywhere
            // near the moving platform's X (~58).
            _em.RemoveComponent<PlatformerCharacterTag>(_character);
            var sinkCmds = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            PhysicsBody2DCommands.SetLinearVelocity(sinkCmds, new float2(0f, -20f));
            PhysicsBody2DCommands.SetTransform(sinkCmds, new float2(-3f, -40f), 0f);
            _fixedGroup.Update();
            _em.AddComponent<PlatformerCharacterTag>(_character);
            Assert.Less(Position().y, -15f, "Respawn/platform: must be below the fall threshold before respawn.");

            var respawned = false;
            for (var i = 0; i < 60; i++)
            {
                Step(1);
                if (Position().y > -1f)
                {
                    respawned = true;
                    break;
                }
            }
            Assert.IsTrue(respawned, $"Respawn/platform: must respawn above the floor (was at {Position()}).");
            Step(20);
            Assert.AreEqual(
                staticSafeX,
                Position().x,
                1.0f,
                $"Respawn/platform: must respawn to the STATIC safe point X ({staticSafeX}), not the moving platform "
                    + $"(~{platformX:F1}); was at {Position()}."
            );
        }

        // ---- step + adjacent-slope DIRECTIONAL regression (the lateral-jump fix) ----------------------------
        //
        // The real course Station-2 step+slope cluster — StepLow (26,-0.7) top 0.3 X[24,28], StepHigh (31,-0.6) top
        // 0.9 X[29,33], the slope-within-limit lip (34,0) rising right at 30° — is exactly the user-reported spot
        // where the capsule was "teleported" laterally instead of traversing it. The bug was directional (one
        // approach direction wedged the capsule at the step/slope corner with a body→character axis badly skewed
        // from the contact normal, so the D2 depenetration's cast-back over-reported a grazing contact as a multi-
        // unit overlap and the grounded vertical-decollide flung the character up-and-back). Root cause + fix:
        // KinematicCharacterUtilities2D.ReconstructOverlap (project the recovered depth onto the true normal).
        //
        // This gate drives the REAL course capsule across the corner in BOTH directions and asserts it never jumps
        // backward past its start (the lateral teleport) and is never flung vertically off the cluster. Pre-fix the
        // −X (down-the-slope) run jumped ~2.3 u backward + ~4.5 u up in a single fixed step AT THE CORNER; this is
        // RED on that.
        //
        // Both runs are BOUNDED to the traversable surface, because the course geometry deliberately has a floorless
        // gap below the slope lip (StepHigh ends at X=33 top 0.9, the lip is at X=34, and nothing solid sits in X[33,34]
        // at floor level). The corner-traversal the lateral-jump fix governs happens entirely on the slope↔lip↔step
        // surface; driving PAST the lip walks the capsule off into the void, where it free-falls and the unrelated
        // PlatformerRespawnSystem teleports it back (a multi-unit SetTransform that is NOT a controller fling). The
        // run therefore stops once the capsule reaches the corner region, so the assertions read the corner traversal
        // the fix is about, not the off-the-edge fall+respawn that the course's intended gap produces.

        [UnityTest]
        public IEnumerator StepSlope_CapsuleWalksAcrossCluster_LeftToRight_NoLateralJump()
        {
            yield return LoadCourse();
            // Place on the sticky floor (top Y=0) a few units left of StepLow's left face (X=24), drive +X up onto
            // StepLow (top 0.3, mountable). Stop at X=27.5 — once mounted on StepLow, before its right edge (X=28).
            // Walking further +X walks off StepLow's right edge into the course's INTENDED floorless gap at X[28,29]
            // (before the StepHigh wall) and free-falls; the RespawnSystem then teleports the capsule back (a multi-
            // unit SetTransform, not a controller fling). The lateral-jump-free corner traversal this gate governs is
            // the climb onto StepLow, which the bounded run reads. (Before the ReconstructOverlap depth fix the
            // capsule was held against the wall by the fabricated depenetration depth and never fell — the bug this
            // session fixed; the bound matches the now-correct fall behaviour.)
            yield return StepSlopeNoLateralJump(new float2(21f, 1.1f), +1f, stopAtX: 27.5f);
        }

        [UnityTest]
        public IEnumerator StepSlope_CapsuleWalksAcrossCluster_RightToLeft_NoLateralJump()
        {
            yield return LoadCourse();
            // Place up the slope-within-limit (lip 34, rising right; at X=40 the slope top Y ≈ (40−34)·tan30 ≈ 3.46)
            // and drive −X DOWN the slope to the lip corner — the direction that exhibited the jump. Stop at X=34.5
            // (the lip), before the floorless X[33,34] gap the course has below the lip — past it the capsule walks
            // off into the void (an intended course gap), free-falls, and the unrelated PlatformerRespawnSystem
            // teleports it back (a multi-unit SetTransform that is NOT a controller fling). The corner traversal this
            // gate governs is the on-surface descent to the lip, which is what the bounded run reads.
            yield return StepSlopeNoLateralJump(new float2(40f, 3.46f + 1.1f), -1f, stopAtX: 34.5f);
        }

        // Drives the capsule across the step+slope corner and asserts no lateral teleport / vertical fling. stopAtX
        // bounds the −X (down-the-slope) run to the lip, before the course's intended floorless gap below it; pass
        // float.NegativeInfinity (−X) / float.PositiveInfinity (+X) to run unbounded when no edge is reachable.
        IEnumerator StepSlopeNoLateralJump(float2 placeAt, float moveX, float stopAtX)
        {
            PlaceCharacter(placeAt);
            Step(25); // settle onto the surface
            Assert.IsTrue(Body().IsGrounded, "character must settle grounded on the step+slope cluster before walking");

            var startX = Position().x;
            // The backward bound: walking +X the character must never cross more than a small tolerance below its
            // start; walking −X never above start+tol. A backward overshoot is the lateral teleport (it jumped ~2.3 u).
            const float backTol = 0.25f;
            // The cluster + slope top stays well under this; the bug flung the character to Y > 5 in one step.
            const float maxReachableY = 9f;
            var extremeBack = startX;
            var prevX = startX;
            for (var i = 0; i < 200; i++)
            {
                Step(1, moveX: moveX);
                var p = Position();
                Assert.IsFalse(float.IsNaN(p.x) || float.IsNaN(p.y), $"no NaN at step {i}");
                var stepDx = p.x - prevX;
                // No multi-unit single-step jump in EITHER axis (the teleport was a single-frame lurch).
                Assert.Less(
                    abs(stepDx),
                    1.0f,
                    $"single-step X lurch of {stepDx} at step {i} (pos {p}) — the lateral teleport"
                );
                Assert.Less(
                    p.y,
                    maxReachableY,
                    $"character flung vertically off the cluster at step {i} (y={p.y}) — the up-fling of the lateral jump"
                );
                if (moveX > 0f)
                    extremeBack = min(extremeBack, p.x);
                else
                    extremeBack = max(extremeBack, p.x);
                prevX = p.x;
                // Stop once the capsule has traversed the corner to the bound (driving past it walks off the course's
                // intended floorless gap; the lateral-jump fix governs only the on-surface corner).
                if ((moveX > 0f && p.x >= stopAtX) || (moveX < 0f && p.x <= stopAtX))
                    break;
            }

            if (moveX > 0f)
                Assert.GreaterOrEqual(
                    extremeBack,
                    startX - backTol,
                    $"character jumped BACKWARD (−X) past its start while walking +X: start {startX}, furthest back {extremeBack}"
                );
            else
                Assert.LessOrEqual(
                    extremeBack,
                    startX + backTol,
                    $"character jumped BACKWARD (+X) past its start while walking −X: start {startX}, furthest back {extremeBack}"
                );

            // It actually traversed the corner (net progress in the drive direction).
            var endX = Position().x;
            if (moveX > 0f)
                Assert.Greater(
                    endX,
                    startX + 1f,
                    $"character must make forward (+X) progress across the corner: {startX} -> {endX}"
                );
            else
                Assert.Less(
                    endX,
                    startX - 1f,
                    $"character must make forward (−X) progress across the corner: {startX} -> {endX}"
                );
            yield return null;
        }

        // ===== the continuous-sim per-tick step-up TRACE on the REAL course (the user's exact spot) ===============
        //
        // The user reports that walking the REAL Station-2 step+slope corner the capsule (a) is blocked stepping
        // R-to-L over the StepLow lip (cannot step at all), and (b) L-to-R is "pushed away" laterally / ends at the
        // correct step-top Y but the climb-start X (an X that does not advance through the step). The user asked for
        // a continuous-sim trace observing EVERY event each tick. This drives the REAL course capsule with the REAL
        // PlatformerCharacterPhysicsSystem into the StepLow corner and logs, per fixed step: the pre-tick pose, the
        // velocity, the grounding state + ground-hit normal, the per-tick MovePosition TARGET the solve enqueues
        // (read off the character's PhysicsBody2DCommand buffer after the solve, before the substrate drains it next
        // tick), the actual landed pose, and the gap between the target and the landed pose (the swept-move clamp —
        // MovePosition is SetTransformTarget, a velocity-based collision-aware kinematic move, NOT a teleport).
        //
        // The decision points (negative-space point 6 — built from the solve's observable behaviour, not imagined
        // inputs): the capsule must ADVANCE through the StepLow lip (monotonic forward X progress past the step
        // face) and SETTLE on the step top, with no snap-back to the climb-start X and no backward overshoot.

        // The real Station-2 StepLow box: centre (26, -0.7), size (4,2) → top 0.3, left face X=24, right face X=28.
        const float StepLowLeftFaceX = 24f;
        const float StepLowRightFaceX = 28f;
        const float StepLowTopY = 0.3f;

        [UnityTest]
        public IEnumerator StepTrace_RealCourse_WalkRightOntoStepLow_AdvancesAndStands()
        {
            yield return LoadCourse();
            // Place on the sticky floor a few units left of StepLow's left face (X=24), drive +X up onto the step.
            yield return StepTrace(new float2(StepLowLeftFaceX - 3f, 1.1f), +1f);
        }

        [UnityTest]
        public IEnumerator StepTrace_RealCourse_WalkLeftOntoStepLow_AdvancesAndStands()
        {
            yield return LoadCourse();
            // Place ON the StepLow top, right of centre, and drive −X toward + off the step's LEFT lip (X=24) down to
            // the lower floor (the R-to-L approach the user reports is now blocked). The capsule should descend the
            // step cleanly and keep advancing −X, not be blocked at the lip.
            yield return StepTrace(new float2(StepLowRightFaceX - 1f, StepLowTopY + 1.05f), -1f);
        }

        // Drives the REAL course capsule into the StepLow corner, recording the full per-tick event trace, and
        // asserts it advances through the step and never snaps backward past its start.
        IEnumerator StepTrace(float2 placeAt, float moveX)
        {
            PlaceCharacter(placeAt);
            Step(25);
            Assert.IsTrue(Body().IsGrounded, "capsule must settle grounded before walking the step corner");

            var startPos = Position();
            var sb = new System.Text.StringBuilder();
            sb.AppendLine(
                $"[REAL-STEP-TRACE moveX={moveX:+0;-0}] start=({startPos.x:F3},{startPos.y:F3}) "
                    + $"StepLow face X∈[{StepLowLeftFaceX},{StepLowRightFaceX}] top={StepLowTopY}"
            );

            var extremeBack = startPos.x;
            var worstAgainstDx = 0f;
            var worstStep = -1;
            var biggestGap = 0f;
            var biggestGapStep = -1;
            var prevPos = startPos;

            for (var i = 0; i < 180; i++)
            {
                var prePos = Position();
                Step(1, moveX: moveX);
                var landed = Position();
                var b = Body();
                var target = EnqueuedMoveTarget();
                var dx = landed.x - prePos.x;
                var dy = landed.y - prePos.y;
                var gap = any(isnan(target)) ? 0f : length(target - landed);

                Assert.IsFalse(float.IsNaN(landed.x) || float.IsNaN(landed.y), $"no NaN at tick {i}");

                if (moveX > 0f)
                    extremeBack = min(extremeBack, landed.x);
                else
                    extremeBack = max(extremeBack, landed.x);
                var against = moveX > 0f ? -dx : dx;
                if (against > worstAgainstDx)
                {
                    worstAgainstDx = against;
                    worstStep = i;
                }
                if (gap > biggestGap)
                {
                    biggestGap = gap;
                    biggestGapStep = i;
                }

                bool nearFace = abs(prePos.x - StepLowLeftFaceX) < 3f;
                bool anomalous = against > 0.05f || gap > 0.1f || abs(dx) > 0.4f;
                if (nearFace || anomalous || (i % 15) == 0)
                {
                    sb.AppendLine(
                        $"  t{i, 3}: pre=({prePos.x:F3},{prePos.y:F3}) land=({landed.x:F3},{landed.y:F3}) "
                            + $"dx={dx:+0.000;-0.000} dy={dy:+0.000;-0.000} "
                            + $"target=({(any(isnan(target)) ? float.NaN : target.x):F3},{(any(isnan(target)) ? float.NaN : target.y):F3}) "
                            + $"gap={gap:F3} |v|={length(b.RelativeVelocity):F2} "
                            + $"v=({b.RelativeVelocity.x:+0.0;-0.0},{b.RelativeVelocity.y:+0.0;-0.0}) "
                            + $"grnd={b.IsGrounded} gN=({b.GroundHit.Normal.x:+0.0;-0.0},{b.GroundHit.Normal.y:+0.0;-0.0}) "
                            + $"sup={b.SuppressGroundSnappingUntilSteppedClear}"
                    );
                }
                prevPos = landed;

                // Bound the run to the StepLow span [24,28]: stop once the capsule has climbed onto StepLow (its
                // centre past X=27.5, before the right edge) for the +X climb, or descended onto the sticky floor
                // (centre below X=22) for the −X descend. Driving further +X walks off StepLow's right edge into the
                // course's INTENDED floorless gap at X[28,29] (before StepHigh) — a correct fall, not a defect; the
                // RespawnSystem then teleports the capsule back (a multi-unit SetTransform that is not a controller
                // fling). The bounded run reads the step traversal the trace is about, not that downstream fall.
                if (moveX > 0f && landed.x >= 27.5f)
                    break;
                if (moveX < 0f && landed.x <= 22f)
                    break;
            }

            var endPos = Position();
            sb.AppendLine(
                $"  END pos=({endPos.x:F3},{endPos.y:F3}) extremeBack={extremeBack:F3} "
                    + $"worstAgainstDx={worstAgainstDx:F3}@{worstStep} biggestTargetVsLandGap={biggestGap:F3}@{biggestGapStep}"
            );
            Debug.Log(sb.ToString());

            Assert.IsTrue(Body().IsGrounded, "capsule must end grounded on the step/floor");
            if (moveX > 0f)
            {
                // L2R: climb onto StepLow — cross the left face X=24, end on the step top (Y ~ StepLowTopY + 1.0).
                Assert.Greater(
                    endPos.x,
                    StepLowLeftFaceX + 1f,
                    $"capsule must advance THROUGH the StepLow face X={StepLowLeftFaceX} (the 'incorrect X — snapped "
                        + $"back to climb-start' symptom); ended at X={endPos.x:F3}"
                );
                Assert.Greater(
                    endPos.y,
                    StepLowTopY + 0.6f,
                    $"capsule must climb onto StepLow (top {StepLowTopY}); ended Y={endPos.y:F3}"
                );
                Assert.GreaterOrEqual(
                    extremeBack,
                    startPos.x - 0.25f,
                    $"capsule jumped BACKWARD (−X) past start while walking +X ('pushed away'); furthest back {extremeBack:F3}"
                );
            }
            else
            {
                // R2L: descend off StepLow's left lip — cross X=24 going −X, keep advancing −X (the 'now blocked' case).
                Assert.Less(
                    endPos.x,
                    StepLowLeftFaceX - 0.5f,
                    $"capsule must step DOWN over the StepLow left lip (X={StepLowLeftFaceX}) and keep going −X (the "
                        + $"'R-to-L step now blocked' symptom); ended at X={endPos.x:F3}"
                );
                Assert.LessOrEqual(
                    extremeBack,
                    startPos.x + 0.25f,
                    $"capsule jumped BACKWARD (+X) past start while walking −X; furthest back {extremeBack:F3}"
                );
            }
            yield return null;
        }

        // ===== the user's EXACT inspector values, on the REAL baked character, both snap modes, the snap-back probe =
        //
        // The headline open symptom: "correct Y, incorrect X" — after climbing the step the capsule's final X snaps
        // BACK to the X where the climb began (Y is the step top, correct). The user reports Snap-To-Ground ON ⇒
        // happens all the time; OFF ⇒ sometimes. These tests drive the REAL Station-2 geometries (the committed
        // PlatformerSample) with the user's exact inspector values set on the real baked character entity, both snap
        // modes, both directions, over BOTH the plain StepLow (X[24,28], top 0.3) and the StepHigh+ramp corner
        // (StepHigh X[29,33] top 0.9 abutting the 30° SlopeWithinLimit lip at X=34). They trace every tick and
        // assert monotonic forward progress through the step (no snap-back to the climb-start X). RED on the symptom;
        // the per-tick log shows the exact tick the written X regresses.

        // The user's exact inspector values, set on the real baked character. Step Handling ON, Max Step Height 0.5,
        // Extra Step Checks 0.1, Char Width For Step Grounding 1; Evaluate Grounding ON, Ground Snapping Distance
        // 0.5, Enhanced Ground Precision OFF; Snap To Ground per the argument.
        void ApplyUserStepParams(bool snapToGround)
        {
            var props = _em.GetComponentData<KinematicCharacterProperties2D>(_character);
            props.EvaluateGrounding = true;
            props.SnapToGround = snapToGround;
            props.GroundSnappingDistance = 0.5f;
            props.EnhancedGroundPrecision = false;
            _em.SetComponentData(_character, props);

            var step = _em.GetComponentData<BasicStepAndSlopeHandlingParameters2D>(_character);
            step.StepHandling = true;
            step.MaxStepHeight = 0.5f;
            step.ExtraStepChecksDistance = 0.1f;
            step.CharacterWidthForStepGroundingCheck = 1f;
            _em.SetComponentData(_character, step);
        }

        // The StepLow plain step (geometry 2 — a single mountable step, top 0.3): walk RIGHT onto it (climb +X) and
        // LEFT off its left lip (descend −X). Bounded to the StepLow span [24,28] so the run reads the climb/stand,
        // not the floorless gap the course has at X[28,29] before StepHigh (walking into that gap is an intended
        // course fall, not a controller defect).
        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkRight_SnapOn()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(StepLowLeftFaceX - 3f, 1.1f), +1f, true, stopAtX: 27.5f);
        }

        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkRight_SnapOff()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(StepLowLeftFaceX - 3f, 1.1f), +1f, false, stopAtX: 27.5f);
        }

        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkLeftDescend_SnapOn()
        {
            yield return LoadCourse();
            // Spawn ON StepLow's top (X=26 centre) and walk −X down off its LEFT lip (X=24) onto the sticky floor
            // (top 0). The R-to-L step traversal; stop at X=22 (well onto the floor, before any other feature).
            yield return StepTraceUserParams(new float2(26f, StepLowTopY + 1.05f), -1f, true, stopAtX: 22f);
        }

        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkLeftDescend_SnapOff()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(26f, StepLowTopY + 1.05f), -1f, false, stopAtX: 22f);
        }

        // The StepLow→StepHigh TRANSITION (the exact reproduction): walk +X across StepLow's right edge (X=28) into
        // the narrow gap before StepHigh's wall (X=29). This is where the snap-back fired — the capsule, grazing
        // StepHigh's left face while still over StepLow's edge, was pushed ~0.86 u BACKWARD (−X) out of the wall it
        // was only touching (ReconstructOverlap's fabricated depth). It must instead either stay or fall straight
        // into the course's intended X[28,29] gap — NEVER lurch backward. The run stops when the capsule falls off
        // (fellOffY), BEFORE the RespawnSystem fires, so the trace reads the approach-and-graze, not the respawn.
        // RED pre-fix (a 0.86 u backward lurch at the wall); GREEN post-fix (falls straight into the gap).
        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkRightIntoStepHighTransition_NoBackwardSnap_SnapOn()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(StepLowLeftFaceX - 3f, 1.1f), +1f, true, stopAtX: 33f);
        }

        [UnityTest]
        public IEnumerator StepUser_PlainStep_WalkRightIntoStepHighTransition_NoBackwardSnap_SnapOff()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(StepLowLeftFaceX - 3f, 1.1f), +1f, false, stopAtX: 33f);
        }

        // The StepHigh+ramp corner (geometry 1): StepHigh X[29,33] top 0.9 (over the 0.5 max — a WALL) abutting the
        // 30° ramp lip at X=34. The traversable corner behaviour: descend the ramp R-to-L to the lip and the StepHigh
        // wall. Bounded to stop at X=33.5 (at the wall), before the floorless X[33,34] gap below the lip the course
        // intends (walking past it is the same intended fall as the StepLow gap). Asserts the corner descent does
        // not fling the capsule backward (+X) or vertically — the "pushed away" / teleport symptom.
        [UnityTest]
        public IEnumerator StepUser_SlopeCorner_WalkLeftDownToWall_SnapOn()
        {
            yield return LoadCourse();
            // Spawn up the ramp (X=38, slope Y≈(38−34)·tan30≈2.31), drive −X DOWN the ramp toward the lip + wall.
            yield return StepTraceUserParams(new float2(38f, 2.31f + 1.1f), -1f, true, stopAtX: 34.2f);
        }

        [UnityTest]
        public IEnumerator StepUser_SlopeCorner_WalkLeftDownToWall_SnapOff()
        {
            yield return LoadCourse();
            yield return StepTraceUserParams(new float2(38f, 2.31f + 1.1f), -1f, false, stopAtX: 34.2f);
        }

        // Drives the REAL baked character with the user's exact params over a real Station-2 geometry, tracing every
        // tick, and asserts no backward snap-back to the climb-start X. Bounded by stopAtX (reached in the drive
        // direction) and by a fell-off-the-course Y floor, so the trace reads the step region, not the downstream
        // intended course gaps + the RespawnSystem SetTransform they trigger (which is not a controller fling).
        IEnumerator StepTraceUserParams(float2 placeAt, float moveX, bool snapToGround, float stopAtX)
        {
            PlaceCharacter(placeAt);
            ApplyUserStepParams(snapToGround); // set AFTER placement (placement detaches/re-attaches the tag)
            Step(25);
            Assert.IsTrue(Body().IsGrounded, "capsule must settle grounded before walking");

            var startPos = Position();
            const float fellOffY = -2f; // below this the capsule has left the course into an intended gap
            var sb = new System.Text.StringBuilder();
            sb.AppendLine(
                $"[USER-STEP-TRACE moveX={moveX:+0;-0} snap={snapToGround}] start=({startPos.x:F3},{startPos.y:F3}) "
                    + $"stopAtX={stopAtX} params: MaxStep=0.5 ExtraChecks=0.1 Width=1 SnapDist=0.5"
            );

            var extremeBack = startPos.x; // furthest the capsule went AGAINST the drive direction
            var maxForward = startPos.x; // furthest WITH the drive direction
            var maxY = startPos.y;
            var worstAgainstDx = 0f;
            var worstStep = -1;
            var biggestGap = 0f;
            var biggestGapStep = -1;
            var ticksRun = 0;

            for (var i = 0; i < 220; i++)
            {
                var prePos = Position();
                Step(1, moveX: moveX);
                ticksRun++;
                var landed = Position();
                var b = Body();
                var target = EnqueuedMoveTarget();
                var dx = landed.x - prePos.x;
                var dy = landed.y - prePos.y;
                var gap = any(isnan(target)) ? 0f : length(target - landed);

                Assert.IsFalse(float.IsNaN(landed.x) || float.IsNaN(landed.y), $"no NaN at tick {i}");

                if (moveX > 0f)
                {
                    extremeBack = min(extremeBack, landed.x);
                    maxForward = max(maxForward, landed.x);
                }
                else
                {
                    extremeBack = max(extremeBack, landed.x);
                    maxForward = min(maxForward, landed.x);
                }
                maxY = max(maxY, landed.y);
                var against = moveX > 0f ? -dx : dx;
                if (against > worstAgainstDx)
                {
                    worstAgainstDx = against;
                    worstStep = i;
                }
                if (gap > biggestGap)
                {
                    biggestGap = gap;
                    biggestGapStep = i;
                }

                bool anomalous = against > 0.05f || gap > 0.1f || abs(dx) > 0.4f;
                if (anomalous || (i % 15) == 0)
                {
                    sb.AppendLine(
                        $"  t{i, 3}: pre=({prePos.x:F3},{prePos.y:F3}) land=({landed.x:F3},{landed.y:F3}) "
                            + $"dx={dx:+0.000;-0.000} dy={dy:+0.000;-0.000} "
                            + $"target=({(any(isnan(target)) ? float.NaN : target.x):F3},{(any(isnan(target)) ? float.NaN : target.y):F3}) "
                            + $"gap={gap:F3} |v|={length(b.RelativeVelocity):F2} "
                            + $"v=({b.RelativeVelocity.x:+0.0;-0.0},{b.RelativeVelocity.y:+0.0;-0.0}) "
                            + $"grnd={b.IsGrounded} gN=({b.GroundHit.Normal.x:+0.0;-0.0},{b.GroundHit.Normal.y:+0.0;-0.0}) "
                            + $"sup={b.SuppressGroundSnappingUntilSteppedClear}"
                    );
                }

                // Stop once the capsule has traversed the step region to the bound, or fell off the course into an
                // intended gap (past the step the trace is about).
                if ((moveX > 0f && landed.x >= stopAtX) || (moveX < 0f && landed.x <= stopAtX))
                    break;
                if (landed.y < fellOffY)
                {
                    sb.AppendLine($"  (fell off the course at tick {i}, y={landed.y:F3} — past the step region)");
                    break;
                }
            }

            var endPos = Position();
            sb.AppendLine(
                $"  END pos=({endPos.x:F3},{endPos.y:F3}) ticks={ticksRun} start.x={startPos.x:F3} "
                    + $"maxForward={maxForward:F3} extremeBack={extremeBack:F3} maxY={maxY:F3} "
                    + $"worstAgainstDx={worstAgainstDx:F3}@{worstStep} biggestTargetVsLandGap={biggestGap:F3}@{biggestGapStep}"
            );
            Debug.Log(sb.ToString());

            // The load-bearing decision points (the user's symptom), read over the bounded step region:
            // 1. NO single-tick backward lurch against the drive (the "incorrect X" snap-back was a 0.86 u backward
            //    push out of a wall the capsule was only grazing — the ReconstructOverlap depth fabrication).
            Assert.LessOrEqual(
                worstAgainstDx,
                0.3f,
                $"single-tick backward lurch {worstAgainstDx:F3} at tick {worstStep} against the drive (snap {snapToGround}) "
                    + "— the 'pushed away / correct Y, incorrect X' snap-back"
            );
            // 2. NO backward overshoot past the start (the lateral teleport).
            if (moveX > 0f)
                Assert.GreaterOrEqual(
                    extremeBack,
                    startPos.x - 0.3f,
                    $"jumped BACKWARD (−X) past start while walking +X; furthest back {extremeBack:F3}"
                );
            else
                Assert.LessOrEqual(
                    extremeBack,
                    startPos.x + 0.3f,
                    $"jumped BACKWARD (+X) past start while walking −X; furthest back {extremeBack:F3}"
                );
            // 3. NO vertical fling.
            Assert.Less(maxY, startPos.y + 4f, $"flung vertically (maxY {maxY:F3} vs start {startPos.y:F3})");
            // 4. Net forward progress in the drive direction over the bounded region (it traversed, not stuck).
            if (moveX > 0f)
                Assert.Greater(
                    maxForward,
                    startPos.x + 0.8f,
                    $"must make +X progress: {startPos.x:F3} -> max {maxForward:F3}"
                );
            else
                Assert.Less(
                    maxForward,
                    startPos.x - 0.8f,
                    $"must make −X progress: {startPos.x:F3} -> max {maxForward:F3}"
                );

            yield return null;
        }

        // The MovePosition target the solve enqueued this tick — the last MovePosition command still in the
        // character's command buffer (the substrate drains + clears it at the start of the NEXT tick).
        float2 EnqueuedMoveTarget()
        {
            if (!_em.HasBuffer<PhysicsBody2DCommand>(_character))
                return new float2(float.NaN, float.NaN);
            var buf = _em.GetBuffer<PhysicsBody2DCommand>(_character);
            var target = new float2(float.NaN, float.NaN);
            for (var i = 0; i < buf.Length; i++)
                if (
                    buf[i].kind == PhysicsBody2DCommandKind.MovePosition
                    || buf[i].kind == PhysicsBody2DCommandKind.MovePositionAndRotation
                )
                    target = buf[i].linear;
            return target;
        }
    }
}
