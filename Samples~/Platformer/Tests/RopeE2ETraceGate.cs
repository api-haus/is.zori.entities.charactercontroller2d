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
// The substrate and Unity.U2D.Physics both define a PhysicsShape2D; the substrate one (the IComponentData the
// bakers emit, carrying categoryBits/contactBits) is the one this trace reads off the baked anchor entity.
using PhysicsShape2D = Zori.Entities.Physics2D.PhysicsShape2D;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Tests
{
    /// <summary>
    /// The rope-mechanic e2e trace harness the user asked for: it drives a SCRIPTED CONTINUOUS INPUT against the real
    /// <c>PlatformerSample.unity</c> scene (the same FixedRateSimpleManager fixed-step drive the
    /// <see cref="PlatformerCourseSmokeGate"/> uses) and records the full per-tick trajectory of a NATURAL rope-grab
    /// attempt — the character starts grounded on the launch ledge, runs toward the gap, jumps, and holds / taps the
    /// grab. Every tick records the stance, the grab input, whether the AirMove+grab detection gate fired, what an
    /// independent detection probe returns at the live position (hit entity + point, or nothing), the distance to the
    /// anchor, and the character pose. The exact tick + surface where the rope trajectory dies is therefore observable.
    ///
    /// <para>It is BOTH the e2e gate the user wants (grab → swing → land over real input) AND the instrument that
    /// decomposes the two upstream suspects Phase 0 left un-adjudicated: "never reached AirMove+grab in range"
    /// (trigger/input/stance) vs "reached it but detection returned nothing" (scene bake) vs "detected but never
    /// swung/landed" (swing path). The substrate masked closest-point primitive itself is already proven sound by
    /// <c>RopeAnchorMaskedClosestPointGate</c> (physics2d) and <c>RopeGrabQueryDiagnosisGate</c> (this assembly) — this
    /// trace sits one layer up, on the continuous control trajectory through the real scene.</para>
    ///
    /// <para>No mocks: the real control intent path (<see cref="PlatformerCharacterControl2D"/> latches),
    /// the real Bursted Platformer solve, the real baked anchor on its dedicated category bit, the real Box2D world.
    /// </para>
    /// </summary>
    [TestFixture]
    public sealed class RopeE2ETraceGate
    {
        const string ParentSceneName = "PlatformerSample";
        const int LoadTimeoutFrames = 1200;
        const float FixedDt = 1f / 60f;

        World _world;
        FixedStepSimulationSystemGroup _fixedGroup;
        IRateManager _savedRateManager;
        Entity _character;
        Entity _anchor;
        EntityManager _em;

        [TearDown]
        public void TearDown()
        {
            if (_fixedGroup != null && _savedRateManager != null)
                _fixedGroup.RateManager = _savedRateManager;
            _fixedGroup = null;
            _savedRateManager = null;
        }

        // ----- harness (mirrors PlatformerCourseSmokeGate's proven fixed-step drive) --------------------------------

        IEnumerator LoadCourse()
        {
            SceneManager.LoadScene(ParentSceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");
            _em = _world.EntityManager;

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

            // The baked rope-anchor entity (the RopeAnchor2D tag rides the same entity the substrate collider baker
            // put the PhysicsShape2D on — same authored GameObject, one entity).
            var anchorQuery = _em.CreateEntityQuery(ComponentType.ReadOnly<RopeAnchor2D>());
            Assert.Greater(
                anchorQuery.CalculateEntityCount(),
                0,
                "No baked RopeAnchor2D entity — the scene's rope anchor did not bake."
            );
            using (var ents = anchorQuery.ToEntityArray(Allocator.Temp))
                _anchor = ents[0];

            _fixedGroup = _world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(_fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            _savedRateManager = _fixedGroup.RateManager;
            _fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            var initGroup = _world.GetExistingSystemManaged<InitializationSystemGroup>();
            if (initGroup != null)
                initGroup.Update();

            _fixedGroup.Update();
        }

        // ----- observation accessors --------------------------------------------------------------------------------

        float2 Position() => _em.GetComponentData<LocalToWorld>(_character).Position.xy;

        float2 AnchorPosition() => _em.GetComponentData<LocalToWorld>(_anchor).Position.xy;

        KinematicCharacterBody2D Body() => _em.GetComponentData<KinematicCharacterBody2D>(_character);

        PlatformerStance2D Stance() =>
            _em.GetComponentData<PlatformerCharacterState2D>(_character).Stance;

        PlatformerCharacterTuning2D Tuning() =>
            _em.GetComponentData<PlatformerCharacterTuning2D>(_character);

        bool GrabLatched() =>
            _em.GetComponentData<PlatformerCharacterControl2D>(_character).GrabPressed;

        PhysicsWorld LiveWorld()
        {
            var q = _em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // The SAME detection the AirMove+grab transition runs, called read-only at the character's live position so
        // the trace sees what a grab WOULD find this tick — independent of whether the stance/input gate let the solve
        // actually call it. Distinguishes "the grab was never attempted in range" from "attempted but found nothing".
        bool ProbeDetect(float searchRadius, out Entity hit, out float2 point)
        {
            using var scratch = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            var tuning = Tuning();
            return PlatformerRopeMath.TryDetectRopeAnchor(
                LiveWorld(),
                Position(),
                searchRadius,
                tuning.RopeAnchorLayerMask,
                scratch,
                out hit,
                out point
            );
        }

        // Drive one fixed step with intent. Edges (jump/grab/release) latch only when requested; the caller decides
        // per-tick whether to re-latch grab (tap repeatedly) or latch once (hold) — the real control system only
        // latches on the input rising edge, so a held key produces no further edges.
        void Step(float moveX = 0f, bool jump = false, bool grab = false, bool release = false)
        {
            var c = _em.GetComponentData<PlatformerCharacterControl2D>(_character);
            c.MoveX = moveX;
            if (jump) c.JumpPressed = true;
            if (grab) c.GrabPressed = true;
            if (release) c.ReleasePressed = true;
            _em.SetComponentData(_character, c);
            _fixedGroup.Update();
        }

        // Place the kinematic character at a world pose by repeated MovePosition (the substrate clamps linear speed,
        // so a far target needs several steps), detaching the solve tag so it does not fight the placement. The proven
        // PlatformerCourseSmokeGate placement, inlined here so this trace is self-contained.
        void PlaceCharacter(float2 pos, PlatformerStance2D stance)
        {
            var hadTag = _em.HasComponent<PlatformerCharacterTag>(_character);
            if (hadTag)
                _em.RemoveComponent<PlatformerCharacterTag>(_character);

            for (var i = 0; i < 200; i++)
            {
                var commands = _em.GetBuffer<PhysicsBody2DCommand>(_character);
                PhysicsBody2DCommands.MovePosition(commands, pos);
                _fixedGroup.Update();
                if (length(Position() - pos) < 0.5f)
                    break;
            }

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

        // ----- pre-flight bake dump ---------------------------------------------------------------------------------

        [UnityTest]
        public IEnumerator PreFlight_BakedAnchorHasQueryableColliderOnCharacterRopeLayer()
        {
            yield return LoadCourse();

            var tuning = Tuning();
            var hasShape = _em.HasComponent<PhysicsShape2D>(_anchor);
            var sb = new StringBuilder();
            sb.AppendLine("[ROPE-E2E PRE-FLIGHT] baked scene anchor vs character rope mask:");
            sb.AppendLine($"  anchor entity = {_anchor}, pos = {AnchorPosition()}");
            sb.AppendLine($"  character entity = {_character}, pos = {Position()}");
            sb.AppendLine(
                $"  character tuning: RopeAnchorLayerMask = 0x{tuning.RopeAnchorLayerMask:X}, "
                    + $"RopeLength = {tuning.RopeLength}, RopeAnchorSearchRadius = {tuning.RopeAnchorSearchRadius}"
            );

            ulong anchorCat = 0, anchorContacts = 0;
            if (hasShape)
            {
                var shape = _em.GetComponentData<PhysicsShape2D>(_anchor);
                anchorCat = shape.categoryBits;
                anchorContacts = shape.contactBits;
                sb.AppendLine(
                    $"  anchor PhysicsShape2D: kind = {shape.kind}, categoryBits = 0x{anchorCat:X}, "
                        + $"contactBits = 0x{anchorContacts:X}"
                );
            }
            else
            {
                sb.AppendLine("  anchor has NO PhysicsShape2D IComponentData (no queryable collider baked!)");
            }

            // What an actual grab-time detection returns against the LIVE world, from the anchor's own position (so
            // distance is ~0 — this isolates the FILTER MATCH from any reachability question).
            var detectedAtAnchor = ProbeDetectAt(AnchorPosition(), tuning.RopeAnchorSearchRadius,
                tuning.RopeAnchorLayerMask, out var hitEnt, out var hitPt);
            sb.AppendLine(
                $"  detection probe AT anchor pos (radius {tuning.RopeAnchorSearchRadius}, mask 0x{tuning.RopeAnchorLayerMask:X}): "
                    + $"found = {detectedAtAnchor}, entity = {hitEnt}, point = {hitPt}"
            );
            Debug.Log(sb.ToString());

            Assert.IsTrue(hasShape,
                "PRE-FLIGHT: the baked scene anchor must carry a queryable PhysicsShape2D collider.");
            Assert.AreEqual(tuning.RopeAnchorLayerMask & anchorCat, tuning.RopeAnchorLayerMask,
                $"PRE-FLIGHT: the anchor's categoryBits 0x{anchorCat:X} must contain the character's rope mask "
                    + $"0x{tuning.RopeAnchorLayerMask:X} — the mask must select this anchor.");
            Assert.IsTrue(detectedAtAnchor,
                "PRE-FLIGHT: the grab detection must find the anchor when standing on it — the scene bake + substrate "
                    + "query are sound, so any in-game failure is upstream (input/stance reachability).");
            Assert.AreEqual(_anchor, hitEnt, "PRE-FLIGHT: detection resolved a different entity than the anchor.");
        }

        bool ProbeDetectAt(float2 at, float searchRadius, ulong mask, out Entity hit, out float2 point)
        {
            using var scratch = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);
            return PlatformerRopeMath.TryDetectRopeAnchor(LiveWorld(), at, searchRadius, mask, scratch,
                out hit, out point);
        }

        // ----- the natural-approach continuous-input e2e trace ------------------------------------------------------

        // Two input policies over the airborne window, traced separately:
        //   HoldGrab  — grab latched on the FIRST airborne tick only (faithful "press E once and hold").
        //   TapGrab   — grab re-latched EVERY airborne tick (faithful "mash E across the arc" — best-case input).
        // The decisive observation is whether EITHER reaches a grab from AirMove with the anchor in range.

        [UnityTest]
        public IEnumerator RopeE2E_NaturalApproach_HoldGrab_Trace()
        {
            yield return LoadCourse();
            yield return RunNaturalApproach(reLatchGrabEachAirTick: false, label: "HOLD-GRAB");
        }

        [UnityTest]
        public IEnumerator RopeE2E_NaturalApproach_TapGrab_Trace()
        {
            yield return LoadCourse();
            yield return RunNaturalApproach(reLatchGrabEachAirTick: true, label: "TAP-GRAB");
        }

        IEnumerator RunNaturalApproach(bool reLatchGrabEachAirTick, string label)
        {
            var tuning = Tuning();
            var anchor = AnchorPosition();
            var searchRadius = tuning.RopeAnchorSearchRadius;

            // Place the character grounded on the launch ledge (Ledge_HighWind, top Y = 9, right edge X = 78), a few
            // units left of the edge so it has run-up room. The capsule half-height ~1 sits the centre at Y ~10.
            PlaceCharacter(new float2(72f, 10.1f), PlatformerStance2D.GroundMove);
            for (var i = 0; i < 40; i++)
            {
                Step(moveX: 0f);
                if (Body().IsGrounded)
                    break;
            }

            var sb = new StringBuilder();
            sb.AppendLine(
                $"[ROPE-E2E {label}] anchor={anchor} searchRadius={searchRadius} ropeLen={tuning.RopeLength} "
                    + $"mask=0x{tuning.RopeAnchorLayerMask:X}; start grounded={Body().IsGrounded} pos={Position()}"
            );

            // Phase 1: run right toward the gap edge to build speed (grounded).
            // Phase 2: jump at/near the edge.
            // Phase 3: air control + grab across the arc — the grounded→AirMove sync makes the grab gate reachable,
            //          so the grab fires and the stance becomes RopeSwing.
            // Phase 4: swing; once the pendulum carries the character forward PAST the anchor's X on an upswing,
            //          release by jump so the forward momentum launches it onto the far ledge (Ledge_FarRope, X∈[91,99],
            //          top Y=9).
            // Phase 5: fall onto the far ledge and re-ground — the landing.
            const int runUpTicks = 25;
            const int airTicks = 320; // generous: covers the arc, the full swing build-up, the release, and the fall

            var everAirborneNearAnchor = false; // got within searchRadius of the anchor while airborne pre-grab
            var everGrabbed = false;            // the grab transition fired into RopeSwing from a non-swing stance
            var everProbeFoundPreGrab = false;  // the read-only probe found the anchor while airborne pre-grab
            var minDistAir = float.MaxValue;    // closest the (pre-grab) airborne character got to the anchor
            var enteredSwing = false;
            var released = false;
            var landedAfterRelease = false;
            var firstGrabTick = -1;
            var releaseTick = -1;
            var landTick = -1;
            var swingMinX = float.MaxValue;
            var swingMaxX = float.MinValue;
            var firstAir = true;

            // run-up
            for (var i = 0; i < runUpTicks; i++)
            {
                var pre = Position();
                Step(moveX: 1f);
                if ((i % 8) == 0 || i == runUpTicks - 1)
                    sb.AppendLine(TickLine($"run{i,3}", pre, searchRadius));
            }

            // jump off the launch ledge
            Step(moveX: 1f, jump: true);
            sb.AppendLine(TickLine("JUMP  ", Position(), searchRadius));

            // airborne arc + grab, then swing, then release-by-jump near the forward apex, then the fall/landing
            for (var i = 0; i < airTicks; i++)
            {
                var pre = Position();
                var stancePre = Stance();
                var grounded = Body().IsGrounded;

                // Grab policy applies only while NOT yet swinging, airborne, and NOT yet having released (a player who
                // has let go to land does not keep mashing grab — otherwise TAP would re-grab the rope the instant it
                // released). HOLD latches once on the first airborne tick; TAP re-latches every airborne tick until the
                // grab takes (after which the swing owns the stance) and never again after a release.
                bool wantGrab = false;
                if (stancePre != PlatformerStance2D.RopeSwing && !grounded && !released)
                {
                    if (reLatchGrabEachAirTick)
                        wantGrab = true;
                    else if (firstAir)
                        wantGrab = true;
                }
                if (!grounded && firstAir)
                    firstAir = false;

                // Pre-grab airborne metrics (the death-point decomposition): record near-anchor + probe BEFORE the grab.
                if (!enteredSwing && stancePre != PlatformerStance2D.RopeSwing && !grounded)
                {
                    var dPre = length(pre - anchor);
                    if (dPre < minDistAir) minDistAir = dPre;
                    if (dPre <= searchRadius) everAirborneNearAnchor = true;
                    if (ProbeDetect(searchRadius, out _, out _))
                        everProbeFoundPreGrab = true;
                }

                // Release decision: once swinging and the character is FORWARD of the anchor X (past the bottom of the
                // arc, on the rising forward side, X > anchor.x) with upward velocity, jump-release so the momentum
                // carries it onto the far ledge. One-shot.
                var bodyPre = Body();
                bool wantRelease = false;
                if (enteredSwing && !released
                    && stancePre == PlatformerStance2D.RopeSwing
                    && pre.x > anchor.x + 0.5f
                    && bodyPre.RelativeVelocity.y > 0f)
                {
                    wantRelease = true;
                }

                var grabLatchedBefore = GrabLatched() || wantGrab;
                Step(moveX: 1f, grab: wantGrab, jump: wantRelease);

                var stancePost = Stance();
                var post = Position();

                // Grab fired: any non-swing -> RopeSwing edge (the grounded→AirMove sync collapses the GroundMove and
                // AirMove labels into one transition block, so the observed pre-stance at the grab tick may read either).
                if (stancePre != PlatformerStance2D.RopeSwing && stancePost == PlatformerStance2D.RopeSwing)
                {
                    everGrabbed = true;
                    enteredSwing = true;
                    firstGrabTick = i;
                    sb.AppendLine(
                        $"  >>> GRAB FIRED at air-tick {i}: {stancePre} -> RopeSwing, pos={post}, "
                            + $"dist={length(post - anchor):F3}"
                    );
                }

                if (stancePost == PlatformerStance2D.RopeSwing)
                {
                    swingMinX = min(swingMinX, post.x);
                    swingMaxX = max(swingMaxX, post.x);
                }

                // Release fired: RopeSwing -> AirMove (the jump-release edge).
                if (wantRelease && stancePre == PlatformerStance2D.RopeSwing
                    && stancePost != PlatformerStance2D.RopeSwing)
                {
                    released = true;
                    releaseTick = i;
                    sb.AppendLine(
                        $"  >>> RELEASE (jump) at air-tick {i}: RopeSwing -> {stancePost}, pos={post}, "
                            + $"v=({Body().RelativeVelocity.x:F2},{Body().RelativeVelocity.y:F2})"
                    );
                }

                // Landing after release: re-grounded on the far ledge (X within the Ledge_FarRope span, well past the gap).
                if (released && Body().IsGrounded && post.x > 89f)
                {
                    landedAfterRelease = true;
                    landTick = i;
                    sb.AppendLine($"  >>> LANDED at air-tick {i}: grounded on far ledge, pos={post}");
                    break;
                }

                bool interesting = (i % 12) == 0 || grabLatchedBefore || stancePre != stancePost || wantRelease;
                if (interesting)
                    sb.AppendLine(TickLine($"air{i,3}", pre, searchRadius));
            }

            var swingX = enteredSwing ? (swingMaxX - swingMinX) : 0f;
            sb.AppendLine(
                $"  SUMMARY[{label}]: everAirborneNearAnchor={everAirborneNearAnchor} minDistAir={minDistAir:F3} "
                    + $"(searchRadius={searchRadius}) everProbeFoundPreGrab={everProbeFoundPreGrab} "
                    + $"everGrabbed={everGrabbed}@{firstGrabTick} enteredSwing={enteredSwing} swingXrange={swingX:F3} "
                    + $"released={released}@{releaseTick} landedAfterRelease={landedAfterRelease}@{landTick} "
                    + $"endPos={Position()} endStance={Stance()} endGrounded={Body().IsGrounded}"
            );
            Debug.Log(sb.ToString());

            // The e2e assertion chain: a natural approach (run, jump, grab) must complete grab -> swing -> release ->
            // land. Each sub-claim is asserted separately so the trace above pins exactly which one fails.
            Assert.IsTrue(everGrabbed,
                $"ROPE-E2E[{label}]: the natural run+jump never entered RopeSwing — the grab was not reachable from "
                    + $"the launch ledge. (everAirborneNearAnchor={everAirborneNearAnchor}, minDistAir={minDistAir:F3} "
                    + $"vs searchRadius {searchRadius}, everProbeFoundPreGrab={everProbeFoundPreGrab}.)");
            Assert.IsTrue(enteredSwing && swingX > 0.5f,
                $"ROPE-E2E[{label}]: entered RopeSwing but the pendulum did not sweep a horizontal arc "
                    + $"(X range {swingX:F3}).");
            Assert.IsTrue(released,
                $"ROPE-E2E[{label}]: the swing never released (jump-off the rope) — could not launch onto the far ledge.");
            Assert.IsTrue(landedAfterRelease,
                $"ROPE-E2E[{label}]: the released swing never delivered the character to a landing on the far ledge "
                    + $"(X > 89). endPos={Position()} endGrounded={Body().IsGrounded}.");
            yield break;
        }

        string TickLine(string tag, float2 prePos, float searchRadius)
        {
            var body = Body();
            var stance = Stance();
            var pos = Position();
            var anchor = AnchorPosition();
            var dist = length(pos - anchor);
            var grab = GrabLatched();
            var probe = ProbeDetect(searchRadius, out var hitEnt, out _);
            return
                $"  {tag}: pos=({pos.x:F2},{pos.y:F2}) stance={stance} grnd={body.IsGrounded} "
                    + $"v=({body.RelativeVelocity.x:+0.0;-0.0},{body.RelativeVelocity.y:+0.0;-0.0}) "
                    + $"distAnchor={dist:F2} grabLatched={grab} probeFinds={probe}{(probe ? $"({hitEnt})" : "")}";
        }
    }
}
