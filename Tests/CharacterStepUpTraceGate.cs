using System.Collections;
using System.Text;
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
    /// Continuous-simulation step-up trace gate (the user-requested e2e observation harness). Drives a capsule with
    /// a constant directional input over many fixed ticks and records EVERY event each tick — the full pose, the
    /// grounding state and ground-hit normal, the relative velocity, the per-tick <c>MovePosition</c> TARGET the
    /// solve enqueues (read off the character's <see cref="PhysicsBody2DCommand"/> buffer after the solve, before
    /// the substrate drains it next tick), and the pose the body ACTUALLY LANDS the following tick. The gap between
    /// the enqueued target and the landed pose is the swept-move clamp the substrate applies (<c>MovePosition</c> is
    /// <c>SetTransformTarget</c> — a velocity-based, collision-aware kinematic move, NOT a teleport like the 3D's
    /// direct <c>LocalTransform.Position</c> write).
    ///
    /// <para>No mocks — the real default solve (<c>KinematicCharacterPhysicsSolveSystem2D</c>), the real Box2D-v3
    /// substrate, the real swept <c>MovePosition</c>. The trace is logged at full per-tick resolution so the exact
    /// tick where X diverges from intent (the "correct Y, incorrect X" the user reports) is visible.</para>
    /// </summary>
    /// <remarks>
    /// Same driving harness as <c>CharacterAdvancedGate</c> (the FixedStepSimulationSystemGroup ticked with a
    /// swapped FixedRateSimpleManager, the default tag added at runtime, render interpolation off). Coroutines yield
    /// <c>null</c>, never <c>WaitForEndOfFrame</c> (does not tick in batchmode).
    /// </remarks>
    public sealed class CharacterStepUpTraceGate
    {
        const int LoadTimeoutFrames = 600;
        const float FixedDt = 1f / 60f;
        const float CapsuleBottomReach = CharacterFixtureBuilderConstants.CapsuleBottomReach;
        const float Offset = KinematicCharacterUtilities2D.Constants.CollisionOffset;

        // The Platformer GroundMove / AirMove drive (matches CharacterAdvancedGate so the trace reproduces the same
        // dynamics the user runs — interpolated ground-line velocity when grounded; gravity + interpolated
        // horizontal when airborne).
        const float GroundSpeed = 7f;
        const float GroundSharpness = 90f;
        const float Gravity = 20f;
        const float AirSharpness = 30f;

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

        IEnumerator LoadAndPrepare(string sceneName)
        {
            SceneManager.LoadScene(sceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");

            var em = _world.EntityManager;

            var bakedQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var frames = 0;
            while (bakedQuery.CalculateEntityCount() < 1 && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.AreEqual(
                1,
                bakedQuery.CalculateEntityCount(),
                $"Expected 1 baked character after {frames} frames — build the fixtures via "
                    + "CharacterFixtureBuilder.BuildAll first."
            );

            using (var ents = bakedQuery.ToEntityArray(Allocator.Temp))
            {
                foreach (var e in ents)
                    if (!em.HasComponent<DefaultCharacterController2DTag>(e))
                        em.AddComponent<DefaultCharacterController2DTag>(e);
            }

            _characterQuery = em.CreateEntityQuery(
                ComponentType.ReadWrite<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            _fixedGroup = _world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(
                _fixedGroup,
                "No FixedStepSimulationSystemGroup in the default world."
            );
            _savedRateManager = _fixedGroup.RateManager;
            _fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // First update: body creation (the substrate does not step on the creation frame).
            _fixedGroup.Update();
        }

        Entity TheCharacter()
        {
            using var ents = _characterQuery.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(1, ents.Length, "this fixture carries exactly one character");
            return ents[0];
        }

        float2 PositionOf(Entity e) =>
            _world.EntityManager.GetComponentData<LocalToWorld>(e).Position.xy;

        KinematicCharacterBody2D BodyOf(Entity e) =>
            _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);

        // The MovePosition target the solve enqueued THIS tick — the last MovePosition command still sitting in the
        // character's command buffer (the substrate drains + clears it at the start of the NEXT tick). float2.NaN if
        // none was enqueued.
        float2 EnqueuedMoveTarget(Entity e)
        {
            var em = _world.EntityManager;
            if (!em.HasBuffer<PhysicsBody2DCommand>(e))
                return new float2(float.NaN, float.NaN);
            var buf = em.GetBuffer<PhysicsBody2DCommand>(e);
            var target = new float2(float.NaN, float.NaN);
            for (var i = 0; i < buf.Length; i++)
                if (
                    buf[i].kind == PhysicsBody2DCommandKind.MovePosition
                    || buf[i].kind == PhysicsBody2DCommandKind.MovePositionAndRotation
                )
                    target = buf[i].linear;
            return target;
        }

        void SetVelocity(Entity e, float2 v)
        {
            var b = _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);
            b.RelativeVelocity = v;
            _world.EntityManager.SetComponentData(e, b);
        }

        void SetSnapToGround(Entity e, bool snap)
        {
            var em = _world.EntityManager;
            var p = em.GetComponentData<KinematicCharacterProperties2D>(e);
            p.SnapToGround = snap;
            em.SetComponentData(e, p);
        }

        // Applies the Platformer GroundMove/AirMove drive for one tick, then updates the fixed group.
        void Drive(Entity e, float moveX, float2 up)
        {
            var b = _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(e);
            if (b.IsGrounded)
            {
                var v = b.RelativeVelocity;
                CharacterControlUtilities2D.StandardGroundMove_Interpolated(
                    ref v,
                    new float2(moveX * GroundSpeed, 0f),
                    GroundSharpness,
                    FixedDt,
                    up,
                    b.GroundHit.Normal
                );
                b.RelativeVelocity = v;
            }
            else
            {
                var v = b.RelativeVelocity;
                v += new float2(0f, -Gravity) * FixedDt;
                v.x = lerp(v.x, moveX * GroundSpeed, saturate(AirSharpness * FixedDt));
                b.RelativeVelocity = v;
            }
            _world.EntityManager.SetComponentData(e, b);
            _fixedGroup.Update();
        }

        // ---- the continuous-sim trace + monotonic-progress assertion --------------------------------------------

        // Drives the capsule with a constant directional input for driveSteps ticks over a plain single step (the
        // user's geometry 2), logging the full per-tick event record, then asserts the character ADVANCED through
        // the step (monotonic forward X progress past the step face, no snap-back to the climb-start X) and SETTLED
        // at the step-top stand height — in the given snap mode.
        IEnumerator TraceTallStep(
            string sceneName,
            float moveX,
            bool snapToGround,
            int driveSteps = 200
        )
        {
            yield return LoadAndPrepare(sceneName);

            var e = TheCharacter();
            SetSnapToGround(e, snapToGround);
            var up = new float2(0f, 1f);

            // Settle on the starting surface.
            for (var s = 0; s < 30; s++)
                Drive(e, 0f, up);
            Assert.IsTrue(
                BodyOf(e).IsGrounded,
                $"{sceneName}: must settle grounded before walking"
            );

            var startPos = PositionOf(e);
            var stepFaceX = 4f; // TallStepLeftFaceX in the fixture
            var stepTop = CharacterFixtureBuilderConstants.TallStepTopY;
            var standY = stepTop + CapsuleBottomReach; // the climbed stand height

            var sb = new StringBuilder();
            sb.AppendLine(
                $"[STEPUP-TRACE {sceneName} moveX={moveX:+0;-0} snap={snapToGround}] "
                    + $"start=({startPos.x:F3},{startPos.y:F3}) stepFaceX={stepFaceX} stepTop={stepTop} standY~{standY:F3}"
            );

            // Per-tick record: the pre-tick pose, the velocity that drove the tick, the MovePosition target the
            // solve enqueued, the actual landed pose this tick, and the gap (target − landed) = the swept clamp.
            var prevPos = PositionOf(e);
            var maxForward = moveX > 0f ? float.MinValue : float.MaxValue; // furthest progress in drive dir
            var extremeBack = moveX > 0f ? startPos.x : startPos.x;
            var worstSnapBackDx = 0f; // largest single-tick move AGAINST the drive direction
            var worstSnapBackStep = -1;
            var biggestTargetVsLandGap = 0f;
            var biggestGapStep = -1;

            for (var i = 0; i < driveSteps; i++)
            {
                var preBody = BodyOf(e);
                var prePos = PositionOf(e);

                Drive(e, moveX, up); // ticks the fixed group

                var landed = PositionOf(e);
                var postBody = BodyOf(e);
                var target = EnqueuedMoveTarget(e);

                var dx = landed.x - prePos.x;
                var dy = landed.y - prePos.y;
                // The gap between where the solve WANTED the body (target) and where the swept move LANDED it. Read
                // the target enqueued THIS tick against the pose it produces NEXT tick is the cleaner comparison,
                // but the target this tick vs this tick's landed pose still exposes a same-tick discrepancy when the
                // pose was already applied; the dominant signal is the across-tick lag, captured by target vs prev.
                var targetVsLandGap = any(isnan(target)) ? 0f : length(target - landed);

                if (moveX > 0f)
                {
                    maxForward = max(maxForward, landed.x);
                    extremeBack = min(extremeBack, landed.x);
                    if (dx < worstSnapBackDx)
                    {
                        worstSnapBackDx = dx;
                        worstSnapBackStep = i;
                    }
                }
                else
                {
                    maxForward = min(maxForward, landed.x);
                    extremeBack = max(extremeBack, landed.x);
                    if (dx > -worstSnapBackDx)
                    { /* track largest +X move while driving −X */
                    }
                    if (dx > 0f && dx > -worstSnapBackDx)
                    {
                        worstSnapBackDx = -dx;
                        worstSnapBackStep = i;
                    }
                }
                if (targetVsLandGap > biggestTargetVsLandGap)
                {
                    biggestTargetVsLandGap = targetVsLandGap;
                    biggestGapStep = i;
                }

                Assert.IsFalse(
                    float.IsNaN(landed.x) || float.IsNaN(landed.y),
                    $"{sceneName}: NaN at tick {i}"
                );

                // Log the step-crossing region densely (near the step face, ±2 u), plus any tick with a big
                // against-direction move or a big target-vs-land gap, plus a periodic heartbeat.
                bool nearFace = abs(prePos.x - stepFaceX) < 2.5f;
                bool anomalous = (moveX > 0f ? dx < -0.02f : dx > 0.02f) || targetVsLandGap > 0.05f;
                if (nearFace || anomalous || (i % 20) == 0)
                {
                    sb.AppendLine(
                        $"  t{i, 3}: pre=({prePos.x:F3},{prePos.y:F3}) land=({landed.x:F3},{landed.y:F3}) "
                            + $"dx={dx:+0.000;-0.000} dy={dy:+0.000;-0.000} "
                            + $"target=({(any(isnan(target)) ? float.NaN : target.x):F3},{(any(isnan(target)) ? float.NaN : target.y):F3}) "
                            + $"gap={targetVsLandGap:F3} "
                            + $"|v|={length(postBody.RelativeVelocity):F2} v=({postBody.RelativeVelocity.x:+0.0;-0.0},{postBody.RelativeVelocity.y:+0.0;-0.0}) "
                            + $"grnd={postBody.IsGrounded} gN=({postBody.GroundHit.Normal.x:+0.0;-0.0},{postBody.GroundHit.Normal.y:+0.0;-0.0}) "
                            + $"sup={postBody.SuppressGroundSnappingUntilSteppedClear}"
                    );
                }
                prevPos = landed;
            }

            var endPos = PositionOf(e);
            sb.AppendLine(
                $"  END pos=({endPos.x:F3},{endPos.y:F3}) maxForward={maxForward:F3} extremeBack={extremeBack:F3} "
                    + $"worstAgainstDx={worstSnapBackDx:F3}@{worstSnapBackStep} "
                    + $"biggestTargetVsLandGap={biggestTargetVsLandGap:F3}@{biggestGapStep}"
            );
            Debug.Log(sb.ToString());

            // The decision-point assertions (the user's intent): the capsule must ADVANCE THROUGH the step —
            // monotonic forward progress crossing the step face — and SETTLE at the step-top stand height, with no
            // snap-back to the climb-start X.
            var b = BodyOf(e);
            Assert.IsTrue(
                b.IsGrounded,
                $"{sceneName}: capsule must end grounded (on the step / floor)"
            );

            if (moveX > 0f)
            {
                // L2R: it must climb onto the step (cross the face X=4, end well past it) at the step-top height.
                Assert.Greater(
                    endPos.x,
                    stepFaceX + 1f,
                    $"{sceneName}: capsule must advance THROUGH the step face (X={stepFaceX}); ended at X={endPos.x:F3} "
                        + "(the 'incorrect X — snapped back to climb-start' symptom)"
                );
                Assert.Less(
                    abs(endPos.y - standY),
                    0.12f,
                    $"{sceneName}: capsule must settle at the step-top stand height ~{standY:F3}; ended Y={endPos.y:F3}"
                );
            }
            else
            {
                // R2L: it descends off the step's far edge and keeps going −X; must make net −X progress and not be
                // flung back +X past start.
                Assert.Less(
                    endPos.x,
                    startPos.x - 1f,
                    $"{sceneName}: capsule must advance (−X) through/off the step; ended at X={endPos.x:F3}"
                );
                Assert.LessOrEqual(
                    extremeBack,
                    startPos.x + 0.25f,
                    $"{sceneName}: capsule jumped BACKWARD (+X) past start while walking −X; furthest back {extremeBack:F3}"
                );
            }
        }

        [UnityTest]
        public IEnumerator TallStep_WalkRight_SnapOn_AdvancesThroughStep() =>
            TraceTallStep("CC2D_TallStepL2R", +1f, snapToGround: true);

        [UnityTest]
        public IEnumerator TallStep_WalkRight_SnapOff_AdvancesThroughStep() =>
            TraceTallStep("CC2D_TallStepL2R", +1f, snapToGround: false);

        [UnityTest]
        public IEnumerator TallStep_WalkLeft_SnapOn_AdvancesThroughStep() =>
            TraceTallStep("CC2D_TallStepR2L", -1f, snapToGround: true);

        [UnityTest]
        public IEnumerator TallStep_WalkLeft_SnapOff_AdvancesThroughStep() =>
            TraceTallStep("CC2D_TallStepR2L", -1f, snapToGround: false);

        // ---- the literal Platformer step+slope cluster (geometry 1), both directions, both snap modes ------------

        // Drives the capsule across the cluster (the user's geometry 1 — a step abutting a downward slope), tracing
        // every tick, and asserts net forward progress with no backward overshoot past start and no vertical fling,
        // in the given snap mode. The R2L direction is the one the user reports is "blocked" (the regression).
        IEnumerator TraceCluster(
            string sceneName,
            float moveX,
            bool snapToGround,
            int driveSteps = 260
        )
        {
            yield return LoadAndPrepare(sceneName);

            var e = TheCharacter();
            SetSnapToGround(e, snapToGround);
            var up = new float2(0f, 1f);

            for (var s = 0; s < 30; s++)
                Drive(e, 0f, up);
            Assert.IsTrue(
                BodyOf(e).IsGrounded,
                $"{sceneName}: must settle grounded before walking"
            );

            var startPos = PositionOf(e);
            var sb = new StringBuilder();
            sb.AppendLine(
                $"[CLUSTER-TRACE {sceneName} moveX={moveX:+0;-0} snap={snapToGround}] start=({startPos.x:F3},{startPos.y:F3})"
            );

            var extremeBack = startPos.x;
            var maxY = startPos.y;
            var worstDx = 0f;
            var worstStep = -1;
            var prevPos = startPos;

            for (var i = 0; i < driveSteps; i++)
            {
                var prePos = PositionOf(e);
                Drive(e, moveX, up);
                var landed = PositionOf(e);
                var postBody = BodyOf(e);
                var target = EnqueuedMoveTarget(e);
                var dx = landed.x - prePos.x;
                var dy = landed.y - prePos.y;

                Assert.IsFalse(
                    float.IsNaN(landed.x) || float.IsNaN(landed.y),
                    $"{sceneName}: NaN at tick {i}"
                );

                if (abs(dx) > abs(worstDx))
                {
                    worstDx = dx;
                    worstStep = i;
                }
                if (moveX > 0f)
                    extremeBack = min(extremeBack, landed.x);
                else
                    extremeBack = max(extremeBack, landed.x);
                maxY = max(maxY, landed.y);

                bool anomalous = abs(dx) > 0.4f || abs(dy) > 0.4f;
                if (anomalous || (i % 20) == 0)
                {
                    sb.AppendLine(
                        $"  t{i, 3}: pre=({prePos.x:F3},{prePos.y:F3}) land=({landed.x:F3},{landed.y:F3}) "
                            + $"dx={dx:+0.000;-0.000} dy={dy:+0.000;-0.000} "
                            + $"target=({(any(isnan(target)) ? float.NaN : target.x):F3},{(any(isnan(target)) ? float.NaN : target.y):F3}) "
                            + $"|v|={length(postBody.RelativeVelocity):F2} grnd={postBody.IsGrounded} "
                            + $"gN=({postBody.GroundHit.Normal.x:+0.0;-0.0},{postBody.GroundHit.Normal.y:+0.0;-0.0})"
                    );
                }
                prevPos = landed;
            }

            var endPos = PositionOf(e);
            sb.AppendLine(
                $"  END pos=({endPos.x:F3},{endPos.y:F3}) extremeBack={extremeBack:F3} maxY={maxY:F3} worstDx={worstDx:F3}@{worstStep}"
            );
            Debug.Log(sb.ToString());

            Assert.Less(
                maxY,
                startPos.y + 6f,
                $"{sceneName}: flung vertically (maxY {maxY:F3} vs start {startPos.y:F3})"
            );
            if (moveX > 0f)
            {
                Assert.GreaterOrEqual(
                    extremeBack,
                    startPos.x - 0.25f,
                    $"{sceneName}: jumped BACKWARD past start while walking +X; furthest back {extremeBack:F3}"
                );
                Assert.Greater(
                    endPos.x,
                    startPos.x + 1f,
                    $"{sceneName}: must make forward (+X) progress: {startPos.x:F3} -> {endPos.x:F3}"
                );
            }
            else
            {
                Assert.LessOrEqual(
                    extremeBack,
                    startPos.x + 0.25f,
                    $"{sceneName}: jumped BACKWARD past start while walking −X; furthest back {extremeBack:F3}"
                );
                Assert.Less(
                    endPos.x,
                    startPos.x - 1f,
                    $"{sceneName}: must make forward (−X) progress: {startPos.x:F3} -> {endPos.x:F3}"
                );
            }
        }

        [UnityTest]
        public IEnumerator Cluster_WalkRight_SnapOn_Traverses() =>
            TraceCluster("CC2D_ClusterL2R", +1f, snapToGround: true);

        [UnityTest]
        public IEnumerator Cluster_WalkRight_SnapOff_Traverses() =>
            TraceCluster("CC2D_ClusterL2R", +1f, snapToGround: false);

        [UnityTest]
        public IEnumerator Cluster_WalkLeft_SnapOn_Traverses() =>
            TraceCluster("CC2D_ClusterR2L", -1f, snapToGround: true);

        [UnityTest]
        public IEnumerator Cluster_WalkLeft_SnapOff_Traverses() =>
            TraceCluster("CC2D_ClusterR2L", -1f, snapToGround: false);

        // ---- the user's step+DOWNSLOPE corner (geometry 1 — the reproduction) ------------------------------------
        //
        // A small step (top StepCornerTopY=0.4, within MaxStepHeight 0.5) whose left vertical face / corner (X=0)
        // abuts a downslope descending to the lower-left. Climbing +X UP the slope onto the step is where the step-up
        // slope-width down-probe samples the slope: the 606d056 forwardSlopeCheckDirection reduces to (0,0) for every
        // step normal (its ReorientVectorOnPlaneAlongDirection2D(up, n, up) is degenerate — vector == alongDirection,
        // so cross2 vanishes), so the down-probe samples STRAIGHT DOWN at the contact, which at this corner lands on
        // the downslope and reports a steep normal → a large extraHeight that pushes steppedHeight past maxStepHeight
        // → the step-up is REJECTED (the capsule cannot climb / is blocked). RED pre-fix; GREEN once the slope-check
        // direction is the true down-slope tangent (the 3D's −normalize(cross(cross(up,n),n))).

        [UnityTest]
        public IEnumerator SlopeStepCorner_ClimbRight_SnapOn_StepsUp() =>
            TraceCorner("CC2D_SlopeStepCornerL2R", +1f, snapToGround: true);

        [UnityTest]
        public IEnumerator SlopeStepCorner_ClimbRight_SnapOff_StepsUp() =>
            TraceCorner("CC2D_SlopeStepCornerL2R", +1f, snapToGround: false);

        [UnityTest]
        public IEnumerator SlopeStepCorner_WalkLeft_SnapOn_Traverses() =>
            TraceCorner("CC2D_SlopeStepCornerR2L", -1f, snapToGround: true);

        [UnityTest]
        public IEnumerator SlopeStepCorner_WalkLeft_SnapOff_Traverses() =>
            TraceCorner("CC2D_SlopeStepCornerR2L", -1f, snapToGround: false);

        // Drives the capsule across the step+downslope corner, traces every tick, and asserts it climbs/crosses the
        // corner (net progress past the step face, settles at the step-top stand when climbing) with no snap-back.
        IEnumerator TraceCorner(
            string sceneName,
            float moveX,
            bool snapToGround,
            int driveSteps = 200
        )
        {
            yield return LoadAndPrepare(sceneName);

            var e = TheCharacter();
            SetSnapToGround(e, snapToGround);
            var up = new float2(0f, 1f);

            for (var s = 0; s < 30; s++)
                Drive(e, 0f, up);
            Assert.IsTrue(
                BodyOf(e).IsGrounded,
                $"{sceneName}: must settle grounded before walking"
            );

            var startPos = PositionOf(e);
            var faceX = CharacterFixtureBuilderConstants.StepCornerFaceX;
            var stepTop = CharacterFixtureBuilderConstants.StepCornerTopY;
            var standY = stepTop + CapsuleBottomReach;

            var sb = new StringBuilder();
            sb.AppendLine(
                $"[CORNER-TRACE {sceneName} moveX={moveX:+0;-0} snap={snapToGround}] start=({startPos.x:F3},{startPos.y:F3}) "
                    + $"stepFaceX={faceX} stepTop={stepTop} standY~{standY:F3}"
            );

            var extremeBack = startPos.x;
            var maxY = startPos.y;
            var worstAgainstDx = 0f;
            var worstStep = -1;
            var sawSteppedUp = false; // a tick where SuppressGroundSnappingUntilSteppedClear got set = step-up engaged
            var prevPos = startPos;

            for (var i = 0; i < driveSteps; i++)
            {
                var prePos = PositionOf(e);
                Drive(e, moveX, up);
                var landed = PositionOf(e);
                var b = BodyOf(e);
                var target = EnqueuedMoveTarget(e);
                var dx = landed.x - prePos.x;
                var dy = landed.y - prePos.y;
                var gap = any(isnan(target)) ? 0f : length(target - landed);

                Assert.IsFalse(
                    float.IsNaN(landed.x) || float.IsNaN(landed.y),
                    $"{sceneName}: NaN at tick {i}"
                );
                if (b.SuppressGroundSnappingUntilSteppedClear)
                    sawSteppedUp = true;

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
                maxY = max(maxY, landed.y);

                bool nearFace = abs(prePos.x - faceX) < 2.5f;
                bool anomalous = against > 0.05f || gap > 0.1f;
                if (nearFace || anomalous || (i % 15) == 0)
                {
                    sb.AppendLine(
                        $"  t{i, 3}: pre=({prePos.x:F3},{prePos.y:F3}) land=({landed.x:F3},{landed.y:F3}) "
                            + $"dx={dx:+0.000;-0.000} dy={dy:+0.000;-0.000} "
                            + $"target=({(any(isnan(target)) ? float.NaN : target.x):F3},{(any(isnan(target)) ? float.NaN : target.y):F3}) "
                            + $"gap={gap:F3} |v|={length(b.RelativeVelocity):F2} v=({b.RelativeVelocity.x:+0.0;-0.0},{b.RelativeVelocity.y:+0.0;-0.0}) "
                            + $"grnd={b.IsGrounded} gN=({b.GroundHit.Normal.x:+0.0;-0.0},{b.GroundHit.Normal.y:+0.0;-0.0}) "
                            + $"sup={b.SuppressGroundSnappingUntilSteppedClear}"
                    );
                }
                prevPos = landed;
            }

            var endPos = PositionOf(e);
            sb.AppendLine(
                $"  END pos=({endPos.x:F3},{endPos.y:F3}) extremeBack={extremeBack:F3} maxY={maxY:F3} "
                    + $"worstAgainstDx={worstAgainstDx:F3}@{worstStep} sawSteppedUp={sawSteppedUp}"
            );
            Debug.Log(sb.ToString());

            Assert.IsTrue(BodyOf(e).IsGrounded, $"{sceneName}: capsule must end grounded");
            if (moveX > 0f)
            {
                // Climbing +X up the slope onto the step: must mount the step (cross the face, rise to the step top)
                // and not be flung/pushed backward. The 'blocked' symptom is the capsule never crossing the face;
                // the 'pushed away' symptom is a backward overshoot.
                Assert.Greater(
                    endPos.x,
                    faceX + 1f,
                    $"{sceneName}: capsule must climb THROUGH the step face X={faceX} onto the step top — the "
                        + $"'R-to-L step blocked' / 'pushed away' symptom; ended at X={endPos.x:F3}"
                );
                Assert.Greater(
                    endPos.y,
                    stepTop + 0.6f,
                    $"{sceneName}: capsule must stand on the step top (top {stepTop}); ended Y={endPos.y:F3}"
                );
                Assert.GreaterOrEqual(
                    extremeBack,
                    startPos.x - 0.3f,
                    $"{sceneName}: capsule pushed BACKWARD past start while climbing +X; furthest back {extremeBack:F3}"
                );
            }
            else
            {
                // Walking −X off the step top down the corner onto the slope/floor: must keep advancing −X.
                Assert.Less(
                    endPos.x,
                    startPos.x - 1f,
                    $"{sceneName}: capsule must traverse the corner (−X); ended at X={endPos.x:F3}"
                );
                Assert.LessOrEqual(
                    extremeBack,
                    startPos.x + 0.3f,
                    $"{sceneName}: capsule pushed BACKWARD (+X) past start while walking −X; furthest back {extremeBack:F3}"
                );
            }
            Assert.Less(maxY, startPos.y + 6f, $"{sceneName}: flung vertically (maxY {maxY:F3})");
        }
    }
}
