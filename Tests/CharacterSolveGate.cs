using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Tests
{
    /// <summary>
    /// The C4a behavioural gate: integration tests that author a kinematic character + static world geometry (via
    /// the editor-side <c>CharacterFixtureBuilder</c>), drive the FixedStepSimulationSystemGroup, and assert
    /// <see cref="LocalToWorld"/> invariants — built adversarially from the solve's own decision points, not from the
    /// inputs the implementer imagined. The gates exercise the C4a core: grounding, collide-and-slide, slope climb,
    /// wall-while-grounded slide, and the D2 overlap depenetration at shallow / DEEP / grounded depths (the most
    /// uncertain port). No mocks — the real default solve system (<c>KinematicCharacterPhysicsSolveSystem2D</c>) runs
    /// over a real Box2D world.
    /// </summary>
    /// <remarks>
    /// The fixed-step group is driven explicitly with a swapped <c>FixedRateSimpleManager</c> (the substrate's
    /// FallingBodyValidation pattern), because the default catch-up manager gates each step on wall-clock time, which
    /// barely advances per <c>yield null</c> in batchmode. The controller solve runs <c>[UpdateAfter(PhysicsWorld2DSystem)]</c>
    /// and the resulting <c>MovePosition</c> applies on the NEXT step (one-step pipeline latency, design D3), so the
    /// tests drive enough steps for the pipeline to settle. The baker does not author the
    /// <see cref="DefaultCharacterController2DTag"/> (the tag is the opt-in to the default solve), so each test adds
    /// it to the baked character at runtime — the real API, no mock.
    /// </remarks>
    public sealed class CharacterSolveGate
    {
        const int LoadTimeoutFrames = 600;
        const float FixedDt = 1f / 60f;
        const float CharacterRadius = 0.5f;
        const float Offset = KinematicCharacterUtilities2D.Constants.CollisionOffset; // 0.01

        World _world;
        FixedStepSimulationSystemGroup _fixedGroup;
        Unity.Entities.IRateManager _savedRateManager;
        EntityQuery _characterQuery;

        [TearDown]
        public void TearDown()
        {
            if (_fixedGroup != null && _savedRateManager != null)
                _fixedGroup.RateManager = _savedRateManager;
            _fixedGroup = null;
            _savedRateManager = null;
        }

        // ---- shared fixture driving ------------------------------------------------------------------------

        IEnumerator LoadAndPrepare(string sceneName)
        {
            SceneManager.LoadScene(sceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");

            var em = _world.EntityManager;

            // Wait (bounded) for the SubScene to stream + bake the character archetype.
            var bakedQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>());
            var frames = 0;
            while (bakedQuery.CalculateEntityCount() == 0 && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.Greater(
                bakedQuery.CalculateEntityCount(),
                0,
                $"No baked character appeared after {frames} frames — build the fixtures via "
                    + "CharacterFixtureBuilder.BuildAll first.");

            // Opt the character into the default solve system by adding the gating tag at runtime.
            using (var ents = bakedQuery.ToEntityArray(Allocator.Temp))
            {
                foreach (var e in ents)
                {
                    if (!em.HasComponent<DefaultCharacterController2DTag>(e))
                        em.AddComponent<DefaultCharacterController2DTag>(e);
                }
            }

            _characterQuery = em.CreateEntityQuery(
                ComponentType.ReadWrite<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>());

            _fixedGroup = _world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(_fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            _savedRateManager = _fixedGroup.RateManager;
            _fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // First update: PhysicsWorld2DSystem creates the Box2D bodies (does not step on the creation frame), so
            // LocalToWorld reads the authored pose after this.
            _fixedGroup.Update();
        }

        Entity TheCharacter()
        {
            using var ents = _characterQuery.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(1, ents.Length, "the gate fixtures carry exactly one character");
            return ents[0];
        }

        float2 Position()
        {
            using var ltw = _characterQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            return ltw[0].Position.xy;
        }

        KinematicCharacterBody2D Body() => _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(TheCharacter());

        void SetRelativeVelocity(float2 v)
        {
            var e = TheCharacter();
            var body = _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);
            body.RelativeVelocity = v;
            _world.EntityManager.SetComponentData(e, body);
        }

        // Step N fixed steps, optionally re-applying a drive velocity before each step (so a steady horizontal walk
        // is not eroded by the per-step projection). Pass driveVelocity = null to let the solve evolve velocity
        // itself (grounding / depenetration / gravity).
        void Step(int count, float2? driveVelocity = null)
        {
            for (var i = 0; i < count; i++)
            {
                if (driveVelocity.HasValue)
                    SetRelativeVelocity(driveVelocity.Value);
                _fixedGroup.Update();
            }
        }

        PhysicsWorld GetPhysicsWorld()
        {
            return _world.EntityManager
                .CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>())
                .GetSingleton<PhysicsWorldSingleton2D>()
                .world;
        }

        // Whether the character's proxy PENETRATES any WORLD body (not just touches). OverlapPoint/OverlapCircle at
        // the character centre always self-hits the character's own collider (the substrate overlap has no
        // entity-exclusion), so the no-residual-overlap check must (a) exclude the character entity and (b) shrink
        // the test radius below the proxy radius so a body resting at the surface (zero separation, the decollide's
        // target) is not counted as penetration — only a body still INSIDE the shrunk disc is a real overlap.
        bool PenetratesWorld(float2 position)
        {
            var pw = GetPhysicsWorld();
            var character = TheCharacter();
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.OverlapCircle(pw, position, CharacterRadius - (3f * Offset), 0ul, hits);
            for (var i = 0; i < hits.Length; i++)
            {
                if (hits[i].entity != character && hits[i].entity != Entity.Null)
                    return true;
            }
            return false;
        }

        // ---- Grounding -------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Grounding_DropOntoFlatFloor_SettlesAtRadiusAboveFloor_NoXDrift()
        {
            yield return LoadAndPrepare("CC2D_Grounding");

            var startX = Position().x;

            // Let it fall and settle: gravity (applied by the default solve while airborne) carries it down onto the
            // floor; the grounding probe + snap stop it ~radius+offset above the floor top.
            Step(180);

            var pos = Position();
            var body = Body();

            Assert.IsTrue(body.IsGrounded, "character must report grounded after landing on flat floor");
            // Rest height: the snap lands the proxy surface on the floor top then lifts by CollisionOffset, so the
            // centre sits radius + offset above the floor top (Y=0).
            float expectedY = CharacterFixtureBuilderConstants.FloorTopY + CharacterRadius + Offset;
            Assert.AreEqual(expectedY, pos.y, 0.08f, $"settled centre-Y should be ~radius above the floor; got {pos.y}");
            Assert.AreEqual(startX, pos.x, 1e-2f, "a straight drop must not drift in X");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN in the settled pose");
        }

        // ---- Collide-and-slide into a wall -----------------------------------------------------------------

        [UnityTest]
        public IEnumerator CollideAndSlide_IntoWall_XClampsAtWall_NoPenetration()
        {
            yield return LoadAndPrepare("CC2D_Wall");

            // Settle on the floor first.
            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded before walking");

            // Drive right into the wall (left face at X=3) for plenty of steps.
            Step(180, new float2(4f, 0f));

            var pos = Position();
            // The proxy radius is 0.5, so the centre cannot pass X = 3 - radius = 2.5 (plus it stops a CollisionOffset
            // short). Allow a small tolerance above 2.5 only for the offset margin, and assert it got close.
            Assert.LessOrEqual(pos.x, 2.5f + 1e-2f, $"centre-X must clamp at wall - radius (2.5); got {pos.x}");
            Assert.Greater(pos.x, 2.0f, $"character should have travelled up to the wall, not stopped early; got {pos.x}");

            // No penetration: the proxy does not penetrate the wall (excluding the character's own collider), and its
            // rightmost point does not cross the wall face.
            Assert.IsFalse(PenetratesWorld(pos), "character must not penetrate the wall");
            Assert.LessOrEqual(pos.x + CharacterRadius, 3.0f + 1e-2f, "the proxy's right edge must not cross the wall face");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN after the wall collide-and-slide");
        }

        // ---- Slope climb within limit ----------------------------------------------------------------------

        [UnityTest]
        public IEnumerator CollideAndSlide_UpSlopeWithinLimit_Climbs_StaysGrounded()
        {
            yield return LoadAndPrepare("CC2D_Slope");

            // Settle onto the ramp.
            Step(90);
            var groundedY = Position().y;
            Assert.IsTrue(Body().IsGrounded, "character must be grounded on the 30-degree ramp (within the 60-degree limit)");

            // Walk up the ramp (to the right).
            Step(150, new float2(3f, 0f));

            var pos = Position();
            Assert.Greater(pos.y, groundedY + 0.3f, $"walking up a 30-degree slope must raise Y; start {groundedY}, end {pos.y}");
            Assert.IsTrue(Body().IsGrounded, "character must stay grounded while climbing a within-limit slope");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN while climbing the slope");
        }

        // ---- Wall while grounded ---------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator CollideAndSlide_WallWhileGrounded_SlidesAlong_StaysGrounded()
        {
            yield return LoadAndPrepare("CC2D_WallGrounded");

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded");
            var floorY = Position().y;

            // Walk into the wall along the floor.
            Step(150, new float2(4f, 0f));

            var pos = Position();
            // Stays grounded and at floor height — sliding along the wall, not climbing it or popping up.
            Assert.IsTrue(Body().IsGrounded, "character must stay grounded while pressing into the wall");
            Assert.AreEqual(floorY, pos.y, 0.06f, $"pressing a wall while grounded must not change floor height; {floorY} -> {pos.y}");
            Assert.LessOrEqual(pos.x + CharacterRadius, 3.0f + 1e-2f, "must not penetrate the wall");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN");
        }

        // ---- Depenetration (D2 — probed hard) --------------------------------------------------------------

        [UnityTest]
        public IEnumerator Depenetration_ShallowOverlapWithWall_PushedOut_NoResidualOverlap()
        {
            yield return LoadAndPrepare("CC2D_OverlapShallow");

            var startPos = Position();
            Assert.IsTrue(PenetratesWorld(startPos), "sanity: character starts penetrating the wall");

            // Grounding is off in this fixture (pure horizontal-overlap probe). Drive zero velocity each step so the
            // default solve's airborne gravity does not accumulate into a free-fall confound — only the D2 cast-back
            // depenetration moves the character. It must push out (to the LEFT, away from the wall to its right)
            // within a bounded number of fixed steps.
            Step(80, new float2(0f, 0f));

            var pos = Position();
            Assert.IsFalse(PenetratesWorld(pos), $"character still penetrating the wall after depenetration; pos {pos}");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN from the depenetration");
        }

        [UnityTest]
        public IEnumerator Depenetration_DeepOverlap_StillResolves_NoNaN_NoTeleport()
        {
            yield return LoadAndPrepare("CC2D_OverlapDeep");

            var startPos = Position();
            Assert.IsTrue(PenetratesWorld(startPos), "sanity: character starts DEEP inside the wall");

            // The adversarial probe: centre 0.45 inside the wall (deeper than CollisionOffset, ~the full radius). The
            // cast-back must recover a push-out direction and the iterative loop converge with no NaN / teleport.
            // Zero-drive each step so gravity does not free-fall the character (no floor in this fixture).
            Step(120, new float2(0f, 0f));

            var pos = Position();

            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "deep overlap must not produce NaN");
            // No teleport: the resolution moves a bounded distance out of a ~0.95-deep overlap, never flinging the
            // character across the world (a small downward gravity drift over the steps is allowed).
            Assert.Less(length(pos - startPos), 3f, $"depenetration must not teleport; moved {length(pos - startPos)} from {startPos} to {pos}");
            Assert.IsFalse(PenetratesWorld(pos), $"character still penetrating the wall after deep depenetration; pos {pos}");
        }

        [UnityTest]
        public IEnumerator Depenetration_SunkIntoFloorWhileGrounded_PopsStraightUp()
        {
            yield return LoadAndPrepare("CC2D_OverlapGround");

            var startX = Position().x;
            Assert.IsTrue(PenetratesWorld(Position()), "sanity: character starts sunk into the floor");

            // Grounding is ON here; let gravity + grounding + the grounded vertical-decollide settle the character.
            Step(150);

            var pos = Position();

            Assert.IsFalse(PenetratesWorld(pos), $"character still penetrating the floor; pos {pos}");
            // Popped UP out of the floor (grounded vertical-decollide), settling ~radius above the floor top, not
            // pushed sideways.
            Assert.GreaterOrEqual(pos.y, CharacterFixtureBuilderConstants.FloorTopY + CharacterRadius - 0.08f, $"must pop up to ~radius above the floor; Y {pos.y}");
            Assert.AreEqual(startX, pos.x, 0.1f, $"vertical decollide must not push the character sideways; X {startX} -> {pos.x}");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN");
        }
    }

    /// <summary>
    /// Geometry constants shared between the runtime gate and the editor-side <c>CharacterFixtureBuilder</c>. Kept in
    /// the runtime test assembly (the editor builder references constants it sets) so the assertion numbers and the
    /// authored geometry stay in lockstep without the runtime assembly referencing the editor one.
    /// </summary>
    public static class CharacterFixtureBuilderConstants
    {
        public const float FloorTopY = 0f;

        // C4b moving-platform: the lateral speed (m/s) the runtime gate drives the kinematic platform at. The
        // gate asserts the ridden character's X travel tracks the platform's; the editor builder references this
        // so the platform geometry and the drive speed share one home.
        public const float PlatformSpeedX = 2f;
    }
}
