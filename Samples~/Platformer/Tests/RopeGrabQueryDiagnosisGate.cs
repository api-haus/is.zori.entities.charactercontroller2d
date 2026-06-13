using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using UnityEngine;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;
// Zori.Entities.Physics2D and Unity.U2D.Physics both define a PhysicsShape2D; the substrate one (an
// IComponentData the bakers emit) is the one this test authors, so name it explicitly.
using PhysicsShape2D = Zori.Entities.Physics2D.PhysicsShape2D;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Tests
{
    /// <summary>
    /// Diagnostic E2E gate for the rope-grab anchor query (the user-reported "grab does nothing" bug). It does NOT
    /// load a scene — it builds a disposable Box2D world with the package's four FixedStep systems, authors a single
    /// static rope anchor on a dedicated collision category (the exact runtime shape the Unity-layer bake produces:
    /// <c>categoryBits = 1 &lt;&lt; layer</c>, <c>contactBits = GetLayerCollisionMask(layer)</c>), and then runs the
    /// SAME substrate query the RopeSwing grab transition runs (<see cref="PlatformerRopeMath.TryDetectRopeAnchor"/> →
    /// <see cref="PhysicsQueries2D.ClosestPoint"/>) with the SAME layer mask the character's
    /// <c>RopeAnchorLayerMask</c> carries.
    ///
    /// <para>It instruments every stage of the query mechanism and logs the raw result so the failing stage is
    /// observable: the broad-phase <c>OverlapCircle</c> hit count, the exact filter bits in play (query mask vs the
    /// anchor's baked category / contact bits), and the final <c>ClosestPoint</c> result. The decisive A/B is the
    /// SAME anchor queried with the SAME mask under two contact-bit configurations — collide-with-everything vs a
    /// restrictive contact row — which isolates whether the query's match depends on the anchor's <em>contact</em>
    /// bits (a property the grab query has no business depending on).</para>
    /// </summary>
    public sealed class RopeGrabQueryDiagnosisGate
    {
        const float Dt = 1f / 60f;

        // The dedicated rope-anchor collision layer the user creates and assigns. Bit 6 is the same one the scene
        // builder uses; the point is that it is a SINGLE dedicated category bit, matched by a single-bit mask.
        const int AnchorLayer = 6;
        const ulong AnchorCategoryMask = 1ul << AnchorLayer;

        const ulong All = 0xFFFFFFFFul;

        static World MakePhysicsWorld(out FixedStepSimulationSystemGroup group)
        {
            var world = new World("RopeGrabQueryDiagnosisWorld");
            var fixedGroup = world.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
            fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(Dt);

            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsWorld2DSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DCleanupSystem>());
            fixedGroup.AddSystemToUpdateList(world.GetOrCreateSystem<PhysicsBody2DWriteBackSystem>());
            fixedGroup.SortSystems();

            group = fixedGroup;
            return world;
        }

        static PhysicsWorld GetWorld(EntityManager em)
        {
            var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingleton<PhysicsWorldSingleton2D>().world;
        }

        // A static circle rope anchor authored exactly as the Unity-layer / scene-builder bake produces it: a
        // dedicated category bit, and an explicit contacts row (the discriminator).
        static Entity SpawnAnchor(EntityManager em, float2 pos, float radius, ulong categoryBits, ulong contactBits)
        {
            return DirectPhysics2DAuthoring.Create(
                em,
                new PhysicsBody2DDefinition { bodyType = PhysicsBody.BodyType.Static, initialPosition = pos },
                new PhysicsShape2D
                {
                    kind = PhysicsShape2DKind.Circle,
                    radius = radius,
                    density = 1f,
                    friction = 0.4f,
                    categoryBits = categoryBits,
                    contactBits = contactBits,
                }
            );
        }

        // Runs the grab query exactly as the RopeSwing transition does, with full instrumentation, and returns
        // whether an anchor was detected (plus the broad-phase hit count it found).
        static bool RunGrabQuery(
            PhysicsWorld pw,
            float2 grabPoint,
            float searchRadius,
            ulong anchorLayerMask,
            out int broadPhaseHits,
            out Entity foundEntity,
            out float2 foundPoint
        )
        {
            using var scratch = new NativeList<PhysicsQueryHit2D>(8, Allocator.Temp);

            // (c) the broad phase, alone, with the SAME mask — this is what ClosestPoint runs first.
            broadPhaseHits = PhysicsQueries2D.OverlapCircle(pw, grabPoint, searchRadius, anchorLayerMask, scratch);

            var got = PlatformerRopeMath.TryDetectRopeAnchor(
                pw,
                grabPoint,
                searchRadius,
                anchorLayerMask,
                scratch,
                out foundEntity,
                out foundPoint
            );
            return got;
        }

        [UnityTest]
        public IEnumerator GrabQuery_AnchorOnDedicatedLayer_RestrictiveContacts_DiagnosesFilterMatch()
        {
            var world = MakePhysicsWorld(out var group);
            var em = world.EntityManager;

            // The character's RopeAnchorLayerMask, baked from the authored Unity LayerMask exactly as
            // PlatformerCharacterBaker does: (ulong)(uint)layerMask.value, a single dedicated bit.
            ulong characterRopeMask = AnchorCategoryMask;

            // CASE A — the scene-builder setup: anchor collides with EVERYTHING (contactBits = ~0). This is the
            // OverrideFilterBits path the authored course uses.
            var anchorAllContacts = SpawnAnchor(
                em,
                new float2(0f, 0f),
                radius: 0.3f,
                categoryBits: AnchorCategoryMask,
                contactBits: All
            );

            // CASE B — the user's Unity-layer setup, dedicated layer with a RESTRICTIVE collision matrix row. A
            // dedicated "anchor" layer often has most/all matrix entries UNCHECKED (the anchor is a non-blocking
            // marker, not a solid the player bonks into). The substrate bakes contactBits = GetLayerCollisionMask
            // (layer); an all-unchecked row bakes contactBits = 0. This is the case to discriminate.
            var anchorNoContacts = SpawnAnchor(
                em,
                new float2(20f, 0f),
                radius: 0.3f,
                categoryBits: AnchorCategoryMask,
                contactBits: 0ul
            );

            // CASE C — dedicated layer that collides only with itself (a plausible matrix row: anchor↔anchor only).
            var anchorSelfContacts = SpawnAnchor(
                em,
                new float2(40f, 0f),
                radius: 0.3f,
                categoryBits: AnchorCategoryMask,
                contactBits: AnchorCategoryMask
            );

            group.Update(); // create the Box2D bodies

            var pw = GetWorld(em);
            const float searchRadius = 12f;

            var gotA = RunGrabQuery(
                pw,
                new float2(0f, 0f),
                searchRadius,
                characterRopeMask,
                out var hitsA,
                out var entA,
                out var ptA
            );
            var gotB = RunGrabQuery(
                pw,
                new float2(20f, 0f),
                searchRadius,
                characterRopeMask,
                out var hitsB,
                out var entB,
                out var ptB
            );
            var gotC = RunGrabQuery(
                pw,
                new float2(40f, 0f),
                searchRadius,
                characterRopeMask,
                out var hitsC,
                out var entC,
                out var ptC
            );

            // Control: the SAME case-B anchor queried with an UNFILTERED mask (0 = hit everything). If this finds
            // it but the filtered query does not, the bug is squarely in the layer-mask filter, not the geometry.
            var gotBUnfiltered = RunGrabQuery(
                pw,
                new float2(20f, 0f),
                searchRadius,
                0ul,
                out var hitsBUnf,
                out _,
                out _
            );

            // --- FIX PROBE: query the contacts=0 case-B anchor directly via world.OverlapGeometry with two
            //     candidate QueryFilter shapes, to pin the exact substrate fix. The substrate's Filter() starts
            //     from QueryFilter.Everything (categories = PhysicsMask.All) and only sets hitCategories — so the
            //     native match's (query.categories & shape.contacts) clause requires shape.contacts != 0, which an
            //     anchor on an all-unchecked layer fails. The fix lives in PhysicsQueries2D.Filter.
            var probeGeom = new CircleGeometry { radius = searchRadius, center = new Vector2(20f, 0f) };

            // Candidate 1 (CURRENT behaviour, reproduced): Everything + hitCategories = mask. categories stays All.
            var filterCurrent = PhysicsQuery.QueryFilter.Everything;
            filterCurrent.hitCategories = new PhysicsMask { bitMask = characterRopeMask };
            var resCurrent = pw.OverlapGeometry(probeGeom, filterCurrent, Allocator.Temp);
            var nCurrent = resCurrent.Length;
            resCurrent.Dispose();

            // Candidate 2 (CANDIDATE FIX): a query category that shares a bit with EVERY shape's contacts is
            // impossible when contacts = 0, so the fix is the OTHER direction — leave the categories-vs-contacts
            // clause satisfied for all shapes by making the query's own category match what shapes contact. The
            // cleanest expression: set hitCategories = mask (or All when unfiltered) AND set categories = All AND
            // verify whether a shape with contacts=0 is reachable at all. If candidate 2 also returns 0, the
            // categories/contacts clause cannot be satisfied for a contacts=0 shape by ANY QueryFilter, which means
            // the substrate fix must be at BODY/SHAPE creation (a queryable shape must carry non-zero contacts), or
            // the grab must use a non-filter query path.
            var filterCat2 = new PhysicsQuery.QueryFilter(
                new PhysicsMask { bitMask = characterRopeMask }, // categories = the anchor category
                new PhysicsMask { bitMask = characterRopeMask }
            ); // hitCategories = the anchor category
            var resCat2 = pw.OverlapGeometry(probeGeom, filterCat2, Allocator.Temp);
            var nCat2 = resCat2.Length;
            resCat2.Dispose();

            Debug.Log(
                $"[ROPE-GRAB-FIXPROBE] contacts=0 anchor via world.OverlapGeometry:\n"
                    + $"  current (Everything + hitCategories=mask, categories=All): hits={nCurrent}\n"
                    + $"  candidate (categories=mask, hitCategories=mask): hits={nCat2}\n"
                    + "  If BOTH are 0, a contacts=0 shape is unreachable by any QueryFilter — the fix is at shape "
                    + "creation (a query-visible shape needs non-zero contacts), not in Filter()."
            );

            Debug.Log(
                $"[ROPE-GRAB-DIAG] characterRopeMask=0x{characterRopeMask:X} (bit {AnchorLayer}); searchRadius={searchRadius}\n"
                    + $"  CASE A (contacts=~0, scene-builder): broadPhase={hitsA} got={gotA} entity={entA} point={ptA}\n"
                    + $"  CASE B (contacts=0,  user dedicated layer): broadPhase={hitsB} got={gotB} entity={entB} point={ptB}\n"
                    + $"  CASE C (contacts=self only): broadPhase={hitsC} got={gotC} entity={entC} point={ptC}\n"
                    + $"  CASE B UNFILTERED (mask=0): broadPhase={hitsBUnf} got={gotBUnfiltered}\n"
                    + $"  anchors: A={anchorAllContacts} B={anchorNoContacts} C={anchorSelfContacts}"
            );

            // --- The PROVEN diagnosis (these are GREEN against the current substrate; they pin the defect) -------
            //
            // CASE A is the authored-course setup: an anchor whose contactBits = ~0. It is FOUND — so the sample's
            // mask construction (1<<layer matched against an anchor on categoryBits = 1<<layer) is CORRECT; there
            // is no sample-side layer-index-vs-bitmask error. (This is the scene the E_RopeSwing gate exercises.)
            Assert.IsTrue(
                gotA,
                "CASE A: the grab query finds an anchor whose contactBits = ~0 — the sample mask path is correct."
            );
            Assert.AreEqual(anchorAllContacts, entA, "CASE A resolved the wrong anchor entity.");

            // CASE C: an anchor that collides with ONLY its own category (any non-zero contacts row) is ALSO found —
            // confirming the discriminating variable is solely the anchor's contactBits being non-zero, not its
            // specific value, and not the query mask.
            Assert.IsTrue(
                gotC,
                "CASE C: an anchor with any non-zero contactBits is found — any non-empty contacts row is queryable."
            );

            // THE ROOT CAUSE, isolated to one variable, and now FIXED. CASE B is the user's setup: a dedicated
            // rope-anchor layer whose 2D collision-matrix row is fully UNCHECKED, which the substrate bakes as
            // contactBits = 0. The original defect made such a shape invisible to EVERY PhysicsQueries2D spatial
            // query — even the UNFILTERED query (mask 0, the documented "hit everything" path) — because Box2D-v3's
            // query-vs-shape match is bidirectional and ANDs (shape.contacts & query.categories) != 0, which a
            // contacts = 0 shape can never pass for any QueryFilter (proven by the FIXPROBE above against the broken
            // substrate). The substrate fix (is.zori.entities.physics2d, cc2d-substrate-additions) decouples
            // query-visibility from the collision-matrix row at shape creation: a categorized shape whose authored
            // contacts row is empty is baked with contacts = its own categoryBits, so it still collides with nothing
            // on every OTHER category while becoming query-visible by its category. With that fix CASE B is now
            // FOUND by the same masked grab query the AirMove->RopeSwing transition runs, and by the unfiltered
            // (mask 0) "hit everything" path, so the grab engages on the user's hand-authored anchor.
            Assert.IsTrue(
                gotB,
                "CASE B: a contactBits = 0 anchor on a dedicated layer is now found by the masked grab query — the "
                    + "substrate decouples query-visibility from the collision-matrix row. The rope grab engages."
            );
            Assert.AreEqual(anchorNoContacts, entB, "CASE B resolved the wrong anchor entity.");
            Assert.IsTrue(
                gotBUnfiltered,
                "CASE B UNFILTERED: the same contactBits = 0 anchor is found by the mask-0 'hit everything' path — "
                    + "the documented PhysicsQueries2D contract is honoured for a non-colliding marker shape."
            );

            world.Dispose();
            yield break;
        }
    }
}
