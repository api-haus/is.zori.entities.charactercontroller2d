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
    /// The C4b behavioural gate: integration tests for the advanced solve features that layer onto the C4a core —
    /// step handling, jump, character ↔ character hit dynamics, character ↔ regular-dynamic-body push, and
    /// moving-platform parenting. Built adversarially from the decision points C4b's own deliverable flagged as
    /// uncertain (the authored-mass approximation for char ↔ dynamic-body, the recorded-not-applied displacement
    /// channel, and the step/slope angle sign), not from the inputs the implementer imagined. No mocks — the real
    /// default solve (<c>KinematicCharacterPhysicsSolveSystem2D</c>), the real
    /// <c>KinematicCharacterDeferredImpulsesSystem2D</c>, the real <c>StoreDynamicBodyDataSystem2D</c> snapshot,
    /// and the real <c>TrackedTransformSystem2D</c> run over a real Box2D world.
    /// </summary>
    /// <remarks>
    /// Same driving harness as the C4a gate (<c>CharacterSolveGate</c>): the FixedStepSimulationSystemGroup is
    /// ticked explicitly with a swapped <c>FixedRateSimpleManager</c> (the substrate's FallingBodyValidation
    /// pattern), the baked characters get the <see cref="DefaultCharacterController2DTag"/> added at runtime (the
    /// real opt-in API), and the solve's <c>MovePosition</c> applies on the NEXT step (one-step pipeline latency,
    /// design D3), so each test drives enough steps to settle. Render interpolation is off in every fixture so the
    /// test reads the raw fixed-step pose off <see cref="LocalToWorld"/>. Coroutines yield <c>null</c>, never
    /// <c>WaitForEndOfFrame</c> (does not tick in batchmode).
    /// </remarks>
    public sealed class CharacterAdvancedGate
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

        // ---- shared fixture driving (mirrors CharacterSolveGate) --------------------------------------------

        // Load the scene, wait for the SubScene to stream + bake N characters, add the default tag to each, swap
        // in the FixedRateSimpleManager, and run one update so the substrate creates the Box2D bodies.
        IEnumerator LoadAndPrepare(string sceneName, int expectedCharacters)
        {
            SceneManager.LoadScene(sceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");

            var em = _world.EntityManager;

            var bakedQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>());
            var frames = 0;
            while (bakedQuery.CalculateEntityCount() < expectedCharacters && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.AreEqual(
                expectedCharacters,
                bakedQuery.CalculateEntityCount(),
                $"Expected {expectedCharacters} baked character(s) after {frames} frames — build the fixtures via "
                    + "CharacterFixtureBuilder.BuildAll first.");

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

            // First update: body creation (the substrate does not step on the creation frame).
            _fixedGroup.Update();
        }

        // The single character (for the step/jump/dynamic-push fixtures, which carry exactly one).
        Entity TheCharacter()
        {
            using var ents = _characterQuery.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(1, ents.Length, "this fixture carries exactly one character");
            return ents[0];
        }

        // The N characters sorted by ascending X (for the char-char fixture, which carries two).
        Entity[] CharactersByX()
        {
            using var ents = _characterQuery.ToEntityArray(Allocator.Temp);
            var arr = ents.ToArray();
            System.Array.Sort(arr, (a, b) => PositionOf(a).x.CompareTo(PositionOf(b).x));
            return arr;
        }

        float2 PositionOf(Entity e)
        {
            return _world.EntityManager.GetComponentData<LocalToWorld>(e).Position.xy;
        }

        float2 Position() => PositionOf(TheCharacter());

        KinematicCharacterBody2D Body() =>
            _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(TheCharacter());

        void SetRelativeVelocity(Entity e, float2 v)
        {
            var body = _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);
            body.RelativeVelocity = v;
            _world.EntityManager.SetComponentData(e, body);
        }

        void Step(int count, float2? driveVelocity = null)
        {
            for (var i = 0; i < count; i++)
            {
                if (driveVelocity.HasValue)
                    SetRelativeVelocity(TheCharacter(), driveVelocity.Value);
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

        // Whether the given character's proxy penetrates any WORLD body other than itself (the same
        // entity-excluding, radius-shrunk probe CharacterSolveGate uses; a surface contact at zero separation is
        // not counted, only a body still inside the shrunk disc).
        bool PenetratesWorld(Entity character, float2 position)
        {
            var pw = GetPhysicsWorld();
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            PhysicsQueries2D.OverlapCircle(pw, position, CharacterRadius - (3f * Offset), 0ul, hits);
            for (var i = 0; i < hits.Length; i++)
            {
                if (hits[i].entity != character && hits[i].entity != Entity.Null)
                    return true;
            }
            return false;
        }

        // The single regular DYNAMIC body (the pushable box), found by its baked PhysicsBody2DDefinition.bodyType
        // — the push fixtures also carry a STATIC floor, so a plain non-character query is not unique; filter on
        // Dynamic. (A character is excluded — it has no PhysicsBody2DDefinition; the controller baker authors the
        // body directly.)
        Entity TheDynamicBox()
        {
            var em = _world.EntityManager;
            var q = em.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2DDefinition>(),
                ComponentType.ReadOnly<LocalToWorld>());
            using var ents = q.ToEntityArray(Allocator.Temp);
            Entity box = Entity.Null;
            var count = 0;
            foreach (var e in ents)
            {
                if (em.GetComponentData<PhysicsBody2DDefinition>(e).bodyType == PhysicsBody.BodyType.Dynamic)
                {
                    box = e;
                    count++;
                }
            }
            Assert.AreEqual(1, count, "this fixture carries exactly one dynamic box");
            return box;
        }

        float2 DynamicBoxCentre()
        {
            return _world.EntityManager.GetComponentData<LocalToWorld>(TheDynamicBox()).Position.xy;
        }

        // The single non-character physics body's entity (the moving platform).
        Entity TheNonCharacterBody()
        {
            var q = _world.EntityManager.CreateEntityQuery(
                new EntityQueryDesc
                {
                    All = new[] { ComponentType.ReadOnly<PhysicsBody2D>(), ComponentType.ReadOnly<LocalToWorld>() },
                    None = new[] { ComponentType.ReadOnly<KinematicCharacterBody2D>() },
                });
            using var ents = q.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(1, ents.Length, "this fixture carries exactly one non-character body (the platform)");
            return ents[0];
        }

        // ---- Step handling ---------------------------------------------------------------------------------

        // Step ≤ MaxStepHeight: the step-up logic must ENGAGE, lift the character onto the step top, AND hold a
        // stable stand there for several fixed steps with no slide-back or reset — including while the box's CENTRE
        // still overhangs the lower floor (the case the C4b gate-3 originally found regressing). The fix is the
        // localized grounding change (KinematicCharacterUtilities2D.cs): step-up lifts the proxy onto the step top
        // with the same CollisionOffset clearance normal ground-snapping keeps, and a one-frame snap-suppression
        // bridges the swept-MovePosition delivery latency (design D3), so the next frame's grounding re-grounds on
        // the STEP top instead of yanking the character down to the lower floor it climbed from. The constant Y the
        // character holds is the step top + radius + offset (~0.815 for the 0.3 step, radius 0.5, offset 0.01).
        // The contrast test below proves a too-high step is never climbed at all (the "> max → blocked" branch).
        [UnityTest]
        public IEnumerator Step_WalkIntoLowStep_StepsUp_HoldsStableStandOnStep()
        {
            yield return LoadAndPrepare("CC2D_StepLow", 1);

            // Settle on the floor.
            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded on the floor");
            var floorY = Position().y;

            // Walk +X into the low step (top at LowStepTopY=0.3, below MaxStepHeight 0.5) until the character has
            // mounted it. The step-up engages while the centre overhangs the lower floor; the character must climb
            // and STAY up, not climb-then-snap-back each frame. Detect the mount: grounded, Y near step-top+radius.
            var stepTop = CharacterFixtureBuilderConstants.LowStepTopY;
            var expectedStandY = stepTop + CharacterRadius; // + ~Offset; asserted to a tolerance below
            var mountedAtStep = -1;
            for (var i = 0; i < 120; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (Body().IsGrounded && abs(Position().y - expectedStandY) <= 0.08f)
                {
                    mountedAtStep = i;
                    break; // start the sustained-stand assertion right after mounting, while most of the step is ahead
                }
            }
            Assert.GreaterOrEqual(mountedAtStep, 0,
                $"step ≤ MaxStepHeight must lift the character onto the step top (~Y {expectedStandY}); never reached it");

            // The STRICT assertion: after mounting, the character holds a stable stand on the step for many fixed
            // steps while still walking +X ACROSS the step surface — no slide-back to the floor, no per-frame reset.
            // The step box spans X[2,7] (left face 2, the editor builder centres a 5-wide box at 4.5); assert the
            // hold only WHILE the character is on the step top (its centre well within the step span), since walking
            // off the far edge correctly descends back to the floor (the right behaviour, not the snap-back bug).
            const float stepRightEdgeX = 7f;
            var minYOnStep = float.MaxValue;
            var maxYOnStep = float.MinValue;
            var startXAfterMount = Position().x;
            var framesAssertedOnStep = 0;
            for (var i = 0; i < 70; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                // Only assert the stable stand while the proxy is fully over the step top (a margin inside both
                // edges so the lip-transition frames at either end are not counted).
                if (Position().x > 2f + CharacterRadius + 0.1f && Position().x < stepRightEdgeX - CharacterRadius - 0.1f)
                {
                    Assert.IsTrue(Body().IsGrounded, $"character must stay grounded standing on the step (step {i}, x={Position().x})");
                    var y = Position().y;
                    minYOnStep = min(minYOnStep, y);
                    maxYOnStep = max(maxYOnStep, y);
                    framesAssertedOnStep++;
                }
            }

            // It genuinely stood on the step for a sustained run (not a one-frame graze), and over that run its Y
            // never dipped back toward the floor (the snap-back bug) nor climbed above the step top — a stable stand.
            Assert.Greater(framesAssertedOnStep, 30,
                $"character must spend a sustained run standing ON the step top, not graze it; on-step frames {framesAssertedOnStep}");
            Assert.GreaterOrEqual(minYOnStep, expectedStandY - 0.08f,
                $"character must HOLD the step stand without snapping back to the floor; floor {floorY}, "
                    + $"expected stand ~{expectedStandY}, min Y on step {minYOnStep}");
            Assert.LessOrEqual(maxYOnStep, expectedStandY + 0.12f,
                $"the held stand must sit at the step top, not climb higher; max Y on step {maxYOnStep}");
            Assert.Greater(Position().x, startXAfterMount + 0.5f,
                $"character must keep walking across the step (not stick at the edge); {startXAfterMount} -> {Position().x}");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN during the step stand");
        }

        [UnityTest]
        public IEnumerator Step_WalkIntoHighStep_Blocked_DoesNotClimb()
        {
            yield return LoadAndPrepare("CC2D_StepHigh", 1);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded on the floor");
            var floorY = Position().y;

            // Walk +X into the high step (top at HighStepTopY=0.9, above MaxStepHeight 0.5). Track peak Y to prove
            // the step-up never engages (the > max decision point): the character must NOT rise at all.
            var peakY = floorY;
            for (var i = 0; i < 180; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                peakY = max(peakY, Position().y);
            }

            var pos = Position();
            // Blocked: the character never climbs a step over MaxStepHeight (peak Y stays at floor height), and its
            // right edge does not cross the step's left face (X = 2) — it slides into the step like an ordinary wall.
            Assert.Less(peakY, floorY + 0.08f, $"a step over MaxStepHeight must never be climbed; floor {floorY}, peak {peakY}");
            Assert.LessOrEqual(pos.x + CharacterRadius, 2.0f + 1e-2f, $"must not penetrate the too-high step face (X=2); centre {pos.x}");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN");
        }

        // ---- Jump ------------------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator Jump_StandardJump_Ungrounds_Rises_Falls_ReGrounds()
        {
            yield return LoadAndPrepare("CC2D_Jump", 1);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded before jumping");
            var groundedY = Position().y;

            // StandardJump2D: clear grounding + add an upward velocity. Applied to the body before a step; the
            // default solve does NOT re-apply gravity on the grounded frame, so the jump velocity carries the
            // character up. (cancelVelocityBeforeJump along grounding-up so the jump is a clean upward launch.)
            {
                var e = TheCharacter();
                var body = _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);
                CharacterControlUtilities2D.StandardJump(
                    ref body,
                    new float2(0f, 8f),
                    cancelVelocityBeforeJump: true,
                    velocityCancelingUpDirection: body.GroundingUp);
                _world.EntityManager.SetComponentData(e, body);
            }

            // Let the jump play out: rise, then gravity pulls it back. Do NOT drive velocity (the solve evolves it
            // with gravity); just step and watch Y.
            var peakY = groundedY;
            var sawUngrounded = false;
            for (var i = 0; i < 30; i++)
            {
                _fixedGroup.Update();
                var y = Position().y;
                if (y > peakY) peakY = y;
                if (!Body().IsGrounded) sawUngrounded = true;
            }

            Assert.IsTrue(sawUngrounded, "the jump must unground the character");
            Assert.Greater(peakY, groundedY + 0.5f, $"the jump must raise Y measurably; grounded {groundedY}, peak {peakY}");

            // Let it fall back and re-ground.
            Step(180);
            var pos = Position();
            Assert.IsTrue(Body().IsGrounded, "character must re-ground after falling back from the jump");
            Assert.AreEqual(groundedY, pos.y, 0.08f, $"must settle back at the floor rest height; {groundedY} -> {pos.y}");
            Assert.IsFalse(float.IsNaN(pos.x) || float.IsNaN(pos.y), "no NaN through the jump arc");
        }

        // ---- Character ↔ character -------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator CharacterVsCharacter_Overlapping_SeparateViaDeferredImpulses()
        {
            yield return LoadAndPrepare("CC2D_CharChar", 2);

            var initial = CharactersByX();
            var leftStart = PositionOf(initial[0]).x;
            var rightStart = PositionOf(initial[1]).x;
            var startGap = rightStart - leftStart;
            Assert.Less(startGap, 2f * CharacterRadius, "sanity: the two characters start overlapping");

            // Let the hit-dynamics impulse exchange separate them. Drive no velocity — the depenetration +
            // exchanged deferred impulses do the work. (Both SimulateDynamicBody, so each pushes the other.)
            for (var i = 0; i < 120; i++)
                _fixedGroup.Update();

            var ended = CharactersByX();
            var leftEnd = PositionOf(ended[0]).x;
            var rightEnd = PositionOf(ended[1]).x;
            var endGap = rightEnd - leftEnd;

            // They separated: the gap grew, the left one moved left, the right one moved right, and neither is NaN.
            Assert.Greater(endGap, startGap + 0.1f, $"the two characters must separate; gap {startGap} -> {endGap}");
            Assert.Less(leftEnd, leftStart + 1e-2f, $"the left character must be pushed left; {leftStart} -> {leftEnd}");
            Assert.Greater(rightEnd, rightStart - 1e-2f, $"the right character must be pushed right; {rightStart} -> {rightEnd}");
            Assert.IsFalse(float.IsNaN(leftEnd) || float.IsNaN(rightEnd), "no NaN after char-char separation");
        }

        // ---- Dynamic-body push -----------------------------------------------------------------------------

        // Shared driver: walk the character +X into the dynamic box for N steps and report how far the box moved
        // in X (its final X minus its start X) plus whether the character ever penetrated it.
        IEnumerator PushDynamicBox(string sceneName, int steps, System.Action<float, float2, bool> report)
        {
            yield return LoadAndPrepare(sceneName, 1);

            // A regular PhysicsBody2DAuthoring body is baked WITHOUT a PhysicsBody2DCommand buffer (the substrate
            // contract: the body owner adds the buffer — only the controller baker adds it, to the character). The
            // deferred-impulse system applies the controller's push to a regular body via AddForce(Impulse) ONLY
            // if the body has that buffer; without it the impulse is silently dropped and the box would move only
            // by Box2D's kinematic-sweep contact resolution (mass-independent). Add the buffer at runtime (the real
            // API — EntityManager.AddBuffer, the same opt-in pattern as the default tag) so the controller's
            // mass-scaled impulse actually reaches the box. This is the body-owner's responsibility for any regular
            // body that should be pushable by a character — a load-bearing integration fact for C6/Phase B.
            {
                var em = _world.EntityManager;
                var box = TheDynamicBox();
                if (!em.HasBuffer<PhysicsBody2DCommand>(box))
                    em.AddBuffer<PhysicsBody2DCommand>(box);
            }

            // Settle, and let StoreDynamicBodyDataSystem2D add + fill the box's snapshot (it is added the step
            // after the box's handle exists, then filled the following step).
            Step(30, new float2(0f, 0f));

            var boxStartX = DynamicBoxCentre().x;

            var character = TheCharacter();
            var penetrated = false;
            for (var i = 0; i < steps; i++)
            {
                SetRelativeVelocity(character, new float2(3f, 0f));
                _fixedGroup.Update();
                if (PenetratesWorld(character, PositionOf(character)))
                    penetrated = true;
            }

            var boxEnd = DynamicBoxCentre();
            report(boxEnd.x - boxStartX, PositionOf(character), penetrated);
        }

        [UnityTest]
        public IEnumerator DynamicPush_KinematicCharacterWalksIntoBox_BoxMoves_CharacterDoesNotPenetrate()
        {
            float boxMovedX = 0f;
            float2 charPos = default;
            var charPenetrated = false;
            yield return PushDynamicBox("CC2D_DynamicPush", 200, (dx, cp, pen) =>
            {
                boxMovedX = dx;
                charPos = cp;
                charPenetrated = pen;
            });

            // The box moved in the push direction (+X), by a plausible amount (not a teleport), and the character
            // never penetrated it. The DIRECTION is the load-bearing assertion (C4b flagged the magnitude as
            // mass-sensitive); the sign of the impulse exchange must push the box AWAY from the character.
            Assert.Greater(boxMovedX, 0.05f, $"the dynamic box must be pushed +X (away from the character); moved {boxMovedX}");
            Assert.Less(boxMovedX, 20f, $"the box push must be a plausible amount, not a teleport; moved {boxMovedX}");
            Assert.IsFalse(charPenetrated, "the character must not penetrate the dynamic box while pushing it");
            Assert.IsFalse(float.IsNaN(charPos.x) || float.IsNaN(charPos.y), "no NaN");
        }

        // Adversarial mass test (C4b's flagged authored-mass approximation): the SAME character push against a
        // box authored 50x heavier must move the box LESS. If the authored-mass read were wrong (ignored, or
        // inverted), the heavy box would move the same or more — this is the probe built from that decision point.
        [UnityTest]
        public IEnumerator DynamicPush_HeavierAuthoredMass_PushesLess()
        {
            float lightMovedX = 0f;
            yield return PushDynamicBox("CC2D_DynamicPush", 200, (dx, _, __) => lightMovedX = dx);

            float heavyMovedX = 0f;
            yield return PushDynamicBox("CC2D_DynamicPushHeavy", 200, (dx, _, __) => heavyMovedX = dx);

            Assert.Greater(lightMovedX, 0.05f, $"sanity: the light box is pushed +X; moved {lightMovedX}");
            Assert.Greater(heavyMovedX, 0f, $"the heavy box should still be nudged +X; moved {heavyMovedX}");
            Assert.Less(
                heavyMovedX,
                lightMovedX,
                $"a heavier authored mass (50x) must be pushed LESS by the same character push; "
                    + $"light {lightMovedX}, heavy {heavyMovedX}");
        }

        // ---- Moving platform -------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator MovingPlatform_CharacterRidesPlatform_CarriedAlongInX()
        {
            yield return LoadAndPrepare("CC2D_MovingPlatform", 1);

            var em = _world.EntityManager;

            // Make the platform a tracked moving platform: add TrackedTransform2D at runtime (no baker authors it
            // — the real API, the same opt-in pattern as the default tag). TrackedTransformSystem2D then records
            // its current/previous fixed-rate poses each step from its LocalToWorld.
            var platform = TheNonCharacterBody();
            if (!em.HasComponent<TrackedTransform2D>(platform))
                em.AddComponent<TrackedTransform2D>(platform);
            // A PhysicsBody2DAuthoring body has no PhysicsBody2DCommand buffer at bake (the substrate contract —
            // the body owner adds it). Add it so the test can drive the kinematic platform with MovePosition.
            if (!em.HasBuffer<PhysicsBody2DCommand>(platform))
                em.AddBuffer<PhysicsBody2DCommand>(platform);

            // Settle the character onto the platform.
            Step(40);
            Assert.IsTrue(Body().IsGrounded, "character must be grounded on the platform before it moves");
            var charStartX = Position().x;
            var charStartY = Position().y;

            // Drive the platform laterally each step with a kinematic MovePosition, and let the controller carry
            // the character along. The character is auto-parented to the tracked platform (Update_MovingPlatform-
            // Detection) and carried by the platform's pose delta (Update_ParentMovement).
            var platformStartX = em.GetComponentData<LocalToWorld>(platform).Position.x;
            const int DriveSteps = 120;
            for (var i = 0; i < DriveSteps; i++)
            {
                var platformPos = em.GetComponentData<LocalToWorld>(platform).Position.xy;
                var target = platformPos + new float2(CharacterFixtureBuilderConstants.PlatformSpeedX * FixedDt, 0f);
                var cmds = em.GetBuffer<PhysicsBody2DCommand>(platform);
                PhysicsBody2DCommands.MovePosition(cmds, target);
                _fixedGroup.Update();
            }

            var platformEndX = em.GetComponentData<LocalToWorld>(platform).Position.x;
            var platformTravel = platformEndX - platformStartX;

            var charEndX = Position().x;
            var charTravel = charEndX - charStartX;

            // The platform actually moved.
            Assert.Greater(platformTravel, 0.5f, $"the platform must have moved +X; travelled {platformTravel}");
            // The character was carried with it: its X travel tracks the platform's, within a tolerance for the
            // one-step pipeline latency (the character carries the PREVIOUS step's platform delta) and grounding
            // snap. It must move substantially +X, not stay put.
            Assert.Greater(charTravel, platformTravel - 0.3f, $"character must be carried along with the platform; platform {platformTravel}, char {charTravel}");
            Assert.Less(charTravel, platformTravel + 0.3f, $"character must not overshoot the platform; platform {platformTravel}, char {charTravel}");
            Assert.IsTrue(Body().IsGrounded, "character must stay grounded riding the platform");
            Assert.AreEqual(charStartY, Position().y, 0.08f, $"riding a flat platform must not change the character's Y; {charStartY} -> {Position().y}");
        }

        // ---- Future-slope / downward-ledge (gate 4, symptom 2 — the angle-sign behavioural arbiter) ---------

        // Configure the character's step/slope params at runtime (the real API, the same opt-in pattern as the
        // default tag): enable the future-slope feature(s) the test exercises.
        void ConfigureSlopeHandling(Entity e, bool preventOnNoGrounding, bool hasMaxDownward, float maxDownwardDeg)
        {
            var em = _world.EntityManager;
            var p = em.GetComponentData<BasicStepAndSlopeHandlingParameters2D>(e);
            p.PreventGroundingWhenMovingTowardsNoGrounding = preventOnNoGrounding;
            p.HasMaxDownwardSlopeChangeAngle = hasMaxDownward;
            p.MaxDownwardSlopeChangeAngle = maxDownwardDeg;
            em.SetComponentData(e, p);
        }

        // Running off a downward ledge with PreventGroundingWhenMovingTowardsNoGrounding on: the character must
        // UNGROUND as it crosses the edge (DetectFutureSlopeChange finds no ground ahead) and launch off — not
        // snap back onto the lip. This exercises the no-grounding branch of the future-slope path.
        [UnityTest]
        public IEnumerator FutureSlope_RunOffDownwardLedge_Ungrounds()
        {
            yield return LoadAndPrepare("CC2D_DownLedge", 1);
            ConfigureSlopeHandling(TheCharacter(), preventOnNoGrounding: true, hasMaxDownward: false, maxDownwardDeg: 90f);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded on the platform");

            // Walk +X toward and off the ledge (right edge at LedgeEdgeX). Watch for the ungrounding as it crosses.
            var sawUngrounded = false;
            for (var i = 0; i < 90; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (Position().x > CharacterFixtureBuilderConstants.LedgeEdgeX && !Body().IsGrounded)
                    sawUngrounded = true;
            }

            Assert.IsTrue(sawUngrounded,
                "running off a downward ledge (PreventGroundingWhenMovingTowardsNoGrounding) must unground the "
                    + "character so it launches off cleanly rather than snapping back onto the lip");
            Assert.Greater(Position().x, CharacterFixtureBuilderConstants.LedgeEdgeX,
                "the character must travel past the ledge edge");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN off the ledge");
        }

        // Walking onto a GENTLE downhill (within MaxDownwardSlopeChange): the character must STAY grounded — the
        // future-slope check finds a grounded slope ahead and the signed angle is within the limit. (Contrast with
        // the steep case below; this proves the angle sign/magnitude does not spuriously unground a shallow slope.)
        [UnityTest]
        public IEnumerator FutureSlope_GentleDownSlope_StaysGrounded()
        {
            yield return LoadAndPrepare("CC2D_DownSlopeGentle", 1);
            ConfigureSlopeHandling(
                TheCharacter(),
                preventOnNoGrounding: false, // isolate the max-downward-angle path (the SIGN arbiter)
                hasMaxDownward: true,
                maxDownwardDeg: CharacterFixtureBuilderConstants.MaxDownwardSlopeChangeForGate);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded on the flat top");

            // Walk +X over the lip onto the gentle (20°) downhill — under the 35° max — and continue down it.
            var groundedFrames = 0;
            for (var i = 0; i < 120; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (Body().IsGrounded) groundedFrames++;
            }

            // It descended the gentle slope (Y dropped below the flat-top rest) while staying grounded almost
            // throughout (a transient ungrounded frame at the very lip is acceptable; a gentle slope must not
            // launch the character).
            Assert.Less(Position().y, CharacterRadius + 0.05f, "character must have descended onto the gentle downhill");
            Assert.Greater(groundedFrames, 110, $"a gentle downslope (under the max) must keep the character grounded; grounded {groundedFrames}/120 frames");
            Assert.IsTrue(Body().IsGrounded, "character must end grounded on the gentle downhill");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN on the gentle slope");
        }

        // Walking onto a STEEP downhill (over MaxDownwardSlopeChange): the character must UNGROUND at the lip — the
        // SIGNED future-slope angle is negative and its magnitude exceeds the max, so degrees(angle) < -max fires.
        // This is the direct behavioural arbiter of CalculateAngleOfHitWithGroundUp's SIGN: a wrong (positive)
        // sign would never trip the test and the character would stay glued to the steep slope (the gentle case
        // above would then be the only outcome, and this contrast test is RED).
        [UnityTest]
        public IEnumerator FutureSlope_SteepDownSlope_Ungrounds()
        {
            yield return LoadAndPrepare("CC2D_DownSlopeSteep", 1);
            ConfigureSlopeHandling(
                TheCharacter(),
                preventOnNoGrounding: false, // isolate the max-downward-angle path (the SIGN arbiter)
                hasMaxDownward: true,
                maxDownwardDeg: CharacterFixtureBuilderConstants.MaxDownwardSlopeChangeForGate);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "character starts grounded on the flat top");

            // Walk +X over the lip toward the steep (55°) downhill — over the 35° max. The signed angle is a
            // downward change exceeding the max, so the character must unground at the lip and launch off.
            var sawUngrounded = false;
            for (var i = 0; i < 60; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (!Body().IsGrounded) sawUngrounded = true;
            }

            Assert.IsTrue(sawUngrounded,
                "a steep downward slope change OVER MaxDownwardSlopeChangeAngle must unground the character "
                    + "(the signed future-slope angle is negative and exceeds the max); if the angle SIGN were "
                    + "inverted this would never fire and the character would stay glued to the slope");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN at the steep lip");
        }

        // ---- P0 capsule character (the capsule mandate, end-to-end) ----------------------------------------

        // The capsule's grounded settle: the centre rests CapsuleBottomReach (+ ~Offset) above the surface it
        // stands on (the bottom cap touches the surface). Used by the capsule grounding + step tests.
        const float CapsuleBottomReach = CharacterFixtureBuilderConstants.CapsuleBottomReach;
        const float CapsuleCapRadius = CharacterFixtureBuilderConstants.CapsuleCapRadius;

        // A capsule proxy probe (the capsule analogue of PenetratesWorld): does the character's capsule proxy,
        // shrunk by 3*Offset, still enclose another world body? A surface contact at zero separation is not
        // counted, only a body genuinely inside the shrunk capsule.
        bool CapsulePenetratesWorld(Entity character, float2 centre)
        {
            var pw = GetPhysicsWorld();
            using var hits = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            var half = CapsuleBottomReach - CapsuleCapRadius; // segment half-length (caps on Y)
            PhysicsQueries2D.OverlapCapsule(
                pw,
                centre + new float2(0f, -half),
                centre + new float2(0f, half),
                CapsuleCapRadius - (3f * Offset),
                0ul,
                hits);
            for (var i = 0; i < hits.Length; i++)
            {
                if (hits[i].entity != character && hits[i].entity != Entity.Null)
                    return true;
            }
            return false;
        }

        // Capsule grounding: a capsule dropped above a flat floor settles at CapsuleBottomReach (+ ~Offset) above
        // the floor top, grounded, with no X drift — the capsule analogue of CharacterSolveGate's grounding test.
        // Proves the CapsuleCast-driven solve grounds a capsule the same way a circle proxy does.
        [UnityTest]
        public IEnumerator Capsule_DroppedOnFloor_Grounds_AtCapsuleBottomReach()
        {
            yield return LoadAndPrepare("CC2D_CapsuleGround", 1);

            var startX = Position().x;
            Step(120);

            Assert.IsTrue(Body().IsGrounded, "the capsule must ground on the floor");
            var settleY = Position().y;
            var expected = CharacterFixtureBuilderConstants.FloorTopY + CapsuleBottomReach;
            Assert.Less(
                abs(settleY - expected),
                0.08f,
                $"the capsule must settle ~CapsuleBottomReach above the floor; expected ~{expected}, got {settleY}");
            Assert.Less(abs(Position().x - startX), 0.05f, "the capsule must not drift in X on a flat floor");
            Assert.IsFalse(CapsulePenetratesWorld(TheCharacter(), Position()), "the settled capsule must not penetrate the floor");
        }

        // Capsule collide-and-slide: a grounded capsule driven +X into a wall (left face X=3) must clamp at the
        // wall (its right cap edge at the face), travel up to it, and never penetrate — the capsule analogue of
        // CharacterSolveGate's wall test. Proves the capsule cast resolves a horizontal collide-and-slide.
        [UnityTest]
        public IEnumerator Capsule_CollideAndSlideIntoWall_ClampsAtWall_NoPenetration()
        {
            yield return LoadAndPrepare("CC2D_CapsuleWall", 1);

            Step(30); // settle on the floor
            Assert.IsTrue(Body().IsGrounded, "the capsule starts grounded near the floor");

            for (var i = 0; i < 180; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(4f, 0f));
                _fixedGroup.Update();
                Assert.IsFalse(
                    CapsulePenetratesWorld(TheCharacter(), Position()),
                    $"the capsule penetrated the wall at step {i} (centre {Position()})");
            }

            // The right cap edge (centre + cap radius) must clamp at or before the wall face X=3.
            var rightEdge = Position().x + CapsuleCapRadius;
            Assert.LessOrEqual(rightEdge, 3f + 1e-2f, $"the capsule's right cap crossed the wall face X=3; right edge {rightEdge}");
            Assert.Greater(Position().x, 2f, $"the capsule must travel up to the wall (centre near it); centre {Position().x}");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN during the capsule wall slide");
        }

        // Capsule step-up (the UNVERIFIED case the box-only gate-4 fix left open): a grounded capsule walks +X
        // into the low step (top 0.3, below the 0.5 max). The capsule must MOUNT it and HOLD a stable stand
        // (grounded, Y at step-top + CapsuleBottomReach) across the step surface — the capsule analogue of the
        // strict box step test (Step_WalkIntoLowStep_StepsUp_HoldsStableStandOnStep). A capsule's rounded bottom
        // is expected to behave at least as well as a box (no top-left-corner catch); this gate is the evidence.
        [UnityTest]
        public IEnumerator Capsule_WalkIntoLowStep_StepsUp_HoldsStableStandOnStep()
        {
            yield return LoadAndPrepare("CC2D_CapsuleStepLow", 1);

            Step(60);
            Assert.IsTrue(Body().IsGrounded, "the capsule starts grounded on the floor");
            var floorY = Position().y;

            var stepTop = CharacterFixtureBuilderConstants.LowStepTopY;
            var expectedStandY = stepTop + CapsuleBottomReach; // + ~Offset; asserted to a tolerance
            var mountedAtStep = -1;
            for (var i = 0; i < 120; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (Body().IsGrounded && abs(Position().y - expectedStandY) <= 0.08f)
                {
                    mountedAtStep = i;
                    break;
                }
            }
            Assert.GreaterOrEqual(mountedAtStep, 0,
                $"a step ≤ MaxStepHeight must lift the capsule onto the step top (~Y {expectedStandY}); never reached it");

            const float stepRightEdgeX = 7f;
            var minYOnStep = float.MaxValue;
            var maxYOnStep = float.MinValue;
            var startXAfterMount = Position().x;
            var framesAssertedOnStep = 0;
            for (var i = 0; i < 70; i++)
            {
                SetRelativeVelocity(TheCharacter(), new float2(3f, 0f));
                _fixedGroup.Update();
                if (Position().x > 2f + CapsuleCapRadius + 0.1f && Position().x < stepRightEdgeX - CapsuleCapRadius - 0.1f)
                {
                    Assert.IsTrue(Body().IsGrounded, $"the capsule must stay grounded standing on the step (step {i}, x={Position().x})");
                    var y = Position().y;
                    minYOnStep = min(minYOnStep, y);
                    maxYOnStep = max(maxYOnStep, y);
                    framesAssertedOnStep++;
                }
            }

            Assert.Greater(framesAssertedOnStep, 30,
                $"the capsule must spend a sustained run standing ON the step top, not graze it; on-step frames {framesAssertedOnStep}");
            Assert.GreaterOrEqual(minYOnStep, expectedStandY - 0.08f,
                $"the capsule must HOLD the step stand without snapping back to the floor; floor {floorY}, "
                    + $"expected stand ~{expectedStandY}, min Y on step {minYOnStep}");
            Assert.LessOrEqual(maxYOnStep, expectedStandY + 0.12f,
                $"the held stand must sit at the step top, not climb higher; max Y on step {maxYOnStep}");
            Assert.Greater(Position().x, startXAfterMount + 0.5f,
                $"the capsule must keep walking across the step (not stick at the edge); {startXAfterMount} -> {Position().x}");
            Assert.IsFalse(float.IsNaN(Position().x) || float.IsNaN(Position().y), "no NaN during the capsule step stand");
        }
    }
}
