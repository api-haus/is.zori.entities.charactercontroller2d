using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// The rope-swing math the RopeSwing stance runs each fixed step (P4). Two static helpers, both HPC#-clean and
    /// reached from the Bursted solve job's <c>Execute</c> (no <c>[BurstCompile]</c> — the entry-point-only rule,
    /// docs/unity/burst/compilation-context.md:31): the pendulum constraint <see cref="ConstrainToRope2D"/> and the
    /// grab-time anchor query <see cref="TryDetectRopeAnchor"/>.
    ///
    /// <para>The rope swing is NOT a physics joint. The pendulum motion is an emergent consequence of three forces
    /// the stance applies in order — gravity pulls the character down, <see cref="ConstrainToRope2D"/> clamps the
    /// character onto the rope-length circle, and the same call projects out the radial (rope-stretching) velocity
    /// component, leaving only the tangential swinging component. A Box2D distance joint would constrain a DYNAMIC
    /// body through the solver; the kinematic character ignores joint forces, so the position-clamp is both the
    /// faithful 3D-reference approach (REF3D RopeSwingState.ConstrainToRope) and the right tool for a body the
    /// controller drives by position.</para>
    /// </summary>
    public static class PlatformerRopeMath
    {
        /// <summary>
        /// Constrains a position-driven body to a rope: if the distance from the rope's anchor point to the body's
        /// own rope-attachment point is at least <paramref name="ropeLength"/>, clamps the body back onto the
        /// rope-length circle AND projects out the radial (rope-stretching) component of velocity. The faithful
        /// <c>float2</c> reduction of the 3D <c>RopeSwingState.ConstrainToRope</c>
        /// (REF3D/Character/States/RopeSwingState.cs:135-155) — the routine is dimension-agnostic
        /// (<c>ProjectOnPlane</c> + <c>ClampToMaxLength</c> exist verbatim in 2D), so the only change is
        /// <c>float3 → float2</c>.
        ///
        /// <para><b>Position clamp.</b> <c>characterToRopeVector = anchor - anchorOnCharacter</c> points from the
        /// body's rope-attachment point toward the anchor. When the rope is taut (<c>length ≥ ropeLength</c>), the
        /// target attachment point is <c>anchor - ClampToMaxLength(characterToRopeVector, ropeLength)</c> — the point
        /// on the rope-length circle nearest the body — and the body is translated by
        /// <c>target - anchorOnCharacter</c> so its attachment point lands on the circle.</para>
        ///
        /// <para><b>Velocity projection.</b> <c>ropeNormal</c> is the unit vector toward the anchor. A velocity with
        /// a NEGATIVE component along <c>ropeNormal</c> is moving AWAY from the anchor (stretching the rope); that
        /// radial component is removed by projecting velocity onto the line orthogonal to <c>ropeNormal</c>, leaving
        /// only the tangential swing. A velocity moving toward the anchor (slackening the rope) is left untouched, so
        /// the character can swing inward / climb the arc freely. This is verbatim from the 3D reference — the sign
        /// test <c>dot(velocity, ropeNormal) &lt; 0</c> and the <c>ProjectOnPlane</c> both port unchanged.</para>
        /// </summary>
        /// <param name="position"> The body's tracked position; translated by the clamp when the rope is taut </param>
        /// <param name="velocity"> The body's velocity; its radial (rope-stretching) component is removed when taut </param>
        /// <param name="ropeLength"> The rope length — the radius of the circle the body is constrained onto </param>
        /// <param name="anchor"> The rope's fixed world-space anchor point (the circle centre) </param>
        /// <param name="anchorOnCharacter"> The body's own rope-attachment point in world space </param>
        public static void ConstrainToRope2D(
            ref float2 position,
            ref float2 velocity,
            float ropeLength,
            float2 anchor,
            float2 anchorOnCharacter)
        {
            float2 characterToRopeVector = anchor - anchorOnCharacter;
            float2 ropeNormal = normalizesafe(characterToRopeVector);

            if (length(characterToRopeVector) >= ropeLength)
            {
                float2 targetAnchorPointOnCharacter = anchor - MathUtilities2D.ClampToMaxLength(characterToRopeVector, ropeLength);
                position += targetAnchorPointOnCharacter - anchorOnCharacter;

                if (dot(velocity, ropeNormal) < 0f)
                {
                    velocity = MathUtilities2D.ProjectOnPlane(velocity, ropeNormal);
                }
            }
        }

        /// <summary>
        /// The grab-time anchor query: the nearest rope anchor within <paramref name="searchRadius"/> of the grab
        /// point, on the rope-anchor collision layer. The 2D substitute for the 3D
        /// <c>physicsWorld.CalculateDistance(PointDistanceInput)</c> the reference uses
        /// (REF3D RopeSwingState.DetectRopePoints, :105-133) — the substrate's <see cref="PhysicsQueries2D.ClosestPoint"/>
        /// is the world-level closest-point query added for exactly this (09-substrate-additions.md): a circle
        /// broad-phase of radius <paramref name="searchRadius"/> filtered to <paramref name="anchorLayerMask"/>, then
        /// the exact closest body by surface distance. It returns the owning entity and the closest point on the
        /// anchor's surface — for a point-like anchor that surface point is the pivot, the <c>AnchorPoint</c> the
        /// pendulum clamps to (the analogue of the 3D <c>closestHit.Position</c>).
        ///
        /// <para>The 3D reference used the single <c>RopeLength</c> as the <c>MaxDistance</c>; the 2D sample uses the
        /// separate <see cref="PlatformerCharacterTuning2D.RopeAnchorSearchRadius"/> (the grab reach) so a course can
        /// place an anchor farther than the rope length without making the grab unreachable.</para>
        ///
        /// <para>Chosen over the design's fallback (<c>OverlapCircle</c> + nearest-centre): <c>OverlapCircle</c>
        /// reports overlapping shapes with a ZERO point/normal (PhysicsQueries2D.ToHit(WorldOverlapResult)), so that
        /// path needs a separate <c>ComponentLookup&lt;LocalToWorld&gt;</c> read per candidate to recover each
        /// anchor's centre and a manual nearest-by-distance scan. <see cref="PhysicsQueries2D.ClosestPoint"/> returns
        /// the entity, the surface point, and the separation distance in one call against the same scratch list,
        /// matching the 3D point-distance query one-to-one. Both are sufficient for point-like anchors; ClosestPoint
        /// is the cleaner one now that it exists.</para>
        /// </summary>
        /// <param name="world"> The stepped substrate world (read from <c>KinematicCharacterUpdateContext2D.PhysicsWorld</c>) </param>
        /// <param name="grabPoint"> The character's grab/detection point in world space (the rope-attachment point) </param>
        /// <param name="searchRadius"> The max grab distance (grab reach) — an anchor beyond this is out of reach </param>
        /// <param name="anchorLayerMask"> The rope-anchor collision-layer mask the query filters to </param>
        /// <param name="scratch"> A caller-owned scratch hit list the broad-phase reuses (the context's <c>TmpQueryHits</c>) </param>
        /// <param name="anchorEntity"> The grabbed anchor entity, or <see cref="Entity.Null"/> when none is in range </param>
        /// <param name="anchorPoint"> The grabbed anchor's pivot (the closest surface point); valid only when this returns true </param>
        /// <returns> True when an anchor is in range and was selected </returns>
        public static bool TryDetectRopeAnchor(
            PhysicsWorld world,
            float2 grabPoint,
            float searchRadius,
            ulong anchorLayerMask,
            NativeList<PhysicsQueryHit2D> scratch,
            out Entity anchorEntity,
            out float2 anchorPoint)
        {
            anchorEntity = Entity.Null;
            anchorPoint = grabPoint;

            if (PhysicsQueries2D.ClosestPoint(world, grabPoint, searchRadius, anchorLayerMask, scratch, out ClosestPoint2D closest))
            {
                anchorEntity = closest.entity;
                anchorPoint = closest.point;
                return true;
            }

            return false;
        }
    }
}
