using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The ported 2D kinematic-character solve algorithm — the dimension-reduced re-expression of
    /// <c>Unity.CharacterController.KinematicCharacterUtilities</c> (REF/KinematicCharacterUtilities.cs), built on
    /// the <c>is.zori.entities.physics2d</c> query surface (<see cref="PhysicsQueries2D"/>) instead of
    /// <c>com.unity.physics</c>. The 3D <c>KinematicCharacterAspect</c> facade is gone (Entities 6.5 dropped
    /// <c>IAspect</c>); the static methods here keep the reference's signature shape — they take the character's
    /// components by <c>ref</c>/<c>in</c>, the transient buffers, and the two contexts — and the caller (a solve
    /// <c>IJobEntity</c>) supplies those from the entity's own data plus the per-system
    /// <see cref="KinematicCharacterUpdateContext2D"/>.
    ///
    /// <para><b>Scope (chunk C4a — the core).</b> This file implements the critical path:
    /// <see cref="Update_Initialize{T,C}"/>, <see cref="Update_Grounding{T,C}"/> + <see cref="GroundDetection{T,C}"/>,
    /// <see cref="Update_MovementAndDecollisions{T,C}"/> = collide-and-slide
    /// (<see cref="MoveWithCollisions{T,C}"/>) + depenetration (<see cref="SolveOverlaps{T,C}"/> via the D2
    /// overlap-cast-back) + <see cref="DecollideFromHit{T,C}"/>, the query-mapping filter loops, the velocity
    /// projection collapsed to corner-only (D6, <see cref="Default_ProjectVelocityOnHits"/>), and the
    /// <c>Default_*</c> processor callbacks. The features that LAYER on this core — step handling, the
    /// prevent-grounding-from-future-slope-change pass, character ↔ character + dynamic-body hit dynamics
    /// (<c>ProcessCharacterHitDynamics</c>), ground pushing, parent / moving-platform movement, and the stateful-hit
    /// Enter/Stay/Exit diff — are chunk C4b; every place one plugs in is marked with a <c>// C4b:</c> seam comment so
    /// C4b integrates without rewriting this code.</para>
    ///
    /// <para><b>Burst.</b> Plain <c>static</c> helpers, NO <c>[BurstCompile]</c> (entry-point-only rule,
    /// docs/unity/burst/compilation-context.md:31-56): the solve <c>IJobEntity</c> is the Burst entry point and
    /// every method here auto-compiles from it. Every method is HPC#-clean: the substrate queries it calls
    /// (<see cref="PhysicsQueries2D"/>) are HPC#, and the core path never reads a regular dynamic body's managed
    /// velocity (that read, <see cref="PhysicsUtilities2D.TryGetDynamicBodyMotion"/>, lives only on the C4b
    /// hit-dynamics path, which runs main-thread).</para>
    /// </summary>
    public static class KinematicCharacterUtilities2D
    {
        /// <summary>
        /// Constants used throughout the 2D character update. 2D port of
        /// <c>KinematicCharacterUtilities.Constants</c> (REF/KinematicCharacterUtilities.cs:447) — every value is
        /// scalar and ports verbatim.
        /// </summary>
        public static class Constants
        {
            /// <summary>Desired distance to stay away from collisions for the character.</summary>
            public const float CollisionOffset = 0.01f;

            /// <summary>Minimum squared velocity length required to make grounding ignore checks.</summary>
            public const float MinVelocityLengthSqForGroundingIgnoreCheck = 0.01f * 0.01f;

            /// <summary>Error margin for considering two normalized vectors to be the same direction.</summary>
            public const float DotProductSimilarityEpsilon = 0.001f;

            /// <summary>Default max length multiplier of reverse projection.</summary>
            public const float DefaultReverseProjectionMaxLengthRatio = 10f;

            /// <summary>Max distance of valid ground hits compared to the closest detected ground hit.</summary>
            public const float GroundedHitDistanceTolerance = CollisionOffset * 6f;

            /// <summary>Squared <see cref="GroundedHitDistanceTolerance"/>.</summary>
            public const float GroundedHitDistanceToleranceSq =
                GroundedHitDistanceTolerance * GroundedHitDistanceTolerance;

            /// <summary>Minimum dot product between grounding-up and slope normal for vertical decollision.</summary>
            public const float MinDotRatioForVerticalDecollision = 0.1f;

            /// <summary>
            /// The layer mask the controller's casts/overlaps use. C4a hits every layer (the
            /// <see cref="PhysicsQueries2D"/> "0 means all" convention); a later chunk may surface a per-character
            /// collision filter. The <see cref="IKinematicCharacterProcessor2D{C}.CanCollideWithHit"/> callback is
            /// the per-hit accept/reject gate in the meantime.
            /// </summary>
            public const ulong CharacterHitLayerMask = 0ul;
        }

        // =====================================================================================================
        // Cast-proxy query wrappers — the single place the circle-or-box proxy (design D1) chooses CircleCast vs
        // BoxCast / OverlapCircle vs OverlapBox. The reference cast the character's actual collider; the substrate
        // offers only circle/box casts (PhysicsQueries2D.cs:245,277), so the proxy decides the shape.
        // =====================================================================================================

        /// <summary>
        /// Sweeps the character's cast proxy from <paramref name="origin"/> along <paramref name="direction"/> for
        /// <paramref name="distance"/>, writing nearest-first hits into the context scratch list. Dispatches to
        /// <see cref="PhysicsQueries2D.CircleCast"/> or <see cref="PhysicsQueries2D.BoxCast"/> by proxy
        /// <see cref="KinematicCharacterColliderProxy2D.Kind"/>. The proxy box is swept axis-aligned at the
        /// character's z-rotation (<paramref name="rotationRadians"/>).
        /// </summary>
        static int CastProxy(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D proxy,
            float2 origin,
            float rotationRadians,
            float2 direction,
            float distance)
        {
            if (proxy.Kind == PhysicsShape2DKind.Box)
            {
                return PhysicsQueries2D.BoxCast(
                    baseContext.PhysicsWorld,
                    origin,
                    proxy.BoxSize,
                    rotationRadians,
                    direction,
                    distance,
                    Constants.CharacterHitLayerMask,
                    baseContext.TmpQueryHits);
            }

            return PhysicsQueries2D.CircleCast(
                baseContext.PhysicsWorld,
                origin,
                proxy.Radius,
                direction,
                distance,
                Constants.CharacterHitLayerMask,
                baseContext.TmpQueryHits);
        }

        /// <summary>
        /// Overlap-tests the character's cast proxy at <paramref name="center"/>, writing the overlapping shapes
        /// (no contact geometry — substrate overlaps carry zero point/normal/fraction) into the context scratch
        /// list. Dispatches by proxy kind. Used by <see cref="SolveOverlaps{T,C}"/> to find penetrating bodies
        /// before the D2 cast-back recovers each one's normal+depth.
        /// </summary>
        static int OverlapProxy(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D proxy,
            float2 center,
            float rotationRadians)
        {
            if (proxy.Kind == PhysicsShape2DKind.Box)
            {
                return PhysicsQueries2D.OverlapBox(
                    baseContext.PhysicsWorld,
                    center,
                    proxy.BoxSize,
                    rotationRadians,
                    Constants.CharacterHitLayerMask,
                    baseContext.TmpQueryHits);
            }

            return PhysicsQueries2D.OverlapCircle(
                baseContext.PhysicsWorld,
                center,
                proxy.Radius,
                Constants.CharacterHitLayerMask,
                baseContext.TmpQueryHits);
        }

        /// <summary>
        /// The proxy's bounding radius — half the box diagonal for a box, the radius for a circle. The D2
        /// cast-back starts from a point this far outside the overlapping shape and casts back toward the
        /// character, guaranteeing the cast begins clear of the overlap so it can register a clean contact normal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float ProxyBoundingRadius(in KinematicCharacterColliderProxy2D proxy)
        {
            if (proxy.Kind == PhysicsShape2DKind.Box)
            {
                return length(proxy.BoxSize) * 0.5f;
            }

            return proxy.Radius;
        }

        // =====================================================================================================
        // Character hit construction
        // =====================================================================================================

        /// <summary>
        /// Builds a <see cref="KinematicCharacterHit2D"/> from a basic hit and the current grounding state. 2D port
        /// of <c>CreateCharacterHit</c> (REF/KinematicCharacterUtilities.cs:386).
        /// </summary>
        public static KinematicCharacterHit2D CreateCharacterHit(
            in BasicHit2D newHit,
            bool characterIsGrounded,
            float2 characterRelativeVelocity,
            bool isGroundedOnHit)
        {
            return new KinematicCharacterHit2D
            {
                Entity = newHit.Entity,
                RigidBodyIndex = newHit.RigidBodyIndex,
                Normal = newHit.Normal,
                Position = newHit.Position,
                WasCharacterGroundedOnHitEnter = characterIsGrounded,
                IsGroundedOnHit = isGroundedOnHit,
                CharacterVelocityBeforeHit = characterRelativeVelocity,
                CharacterVelocityAfterHit = characterRelativeVelocity,
            };
        }

        // =====================================================================================================
        // Step 1 — Initialize (REF/KinematicCharacterUtilities.cs:504)
        // =====================================================================================================

        /// <summary>
        /// The initialization step of the character update. Clears the transient buffers, snapshots the
        /// was-grounded / previous-parent state, resets the per-step fields, records the delta time, and calls the
        /// processor's <see cref="IKinematicCharacterProcessor2D{C}.UpdateGroundingUp"/>. 2D port of
        /// <c>Update_Initialize</c> (REF/KinematicCharacterUtilities.cs:504): the only reduction is
        /// <c>RotationFromParent = quaternion.identity</c> becoming <c>= 0f</c> (the identity z-angle).
        /// </summary>
        public static void Update_Initialize<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            float deltaTime)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            characterHitsBuffer.Clear();
            deferredImpulsesBuffer.Clear();
            velocityProjectionHitsBuffer.Clear();

            characterBody.WasGroundedBeforeCharacterUpdate = characterBody.IsGrounded;
            characterBody.PreviousParentEntity = characterBody.ParentEntity;

            characterBody.RotationFromParent = 0f;
            characterBody.IsGrounded = false;
            characterBody.GroundHit = default;
            characterBody.LastPhysicsUpdateDeltaTime = deltaTime;

            processor.UpdateGroundingUp(ref context, ref baseContext);
        }

        // C4b: Update_ParentMovement (REF/KinematicCharacterUtilities.cs:544) plugs in HERE, between Initialize and
        // Grounding. It carries the character's pose rigidly with a TrackedTransform2D parent's pose delta. It is
        // omitted from the core; the per-entity context still exposes everything it would need (the character
        // position, the TrackedTransformLookup on baseContext, the ParentEntity on characterBody).

        // =====================================================================================================
        // Step 3 — Grounding (REF/KinematicCharacterUtilities.cs:642) + GroundDetection (:2449)
        // =====================================================================================================

        /// <summary>
        /// Detects grounding at the current pose, snaps to ground when configured, and (when grounded) records the
        /// ground hit as a velocity-projection hit + a character hit and projects velocity onto it. 2D port of
        /// <c>Update_Grounding</c> (REF/KinematicCharacterUtilities.cs:642). The probe sweeps the cast proxy along
        /// <c>-GroundingUp</c>; <paramref name="characterPosition"/> is moved by the snap.
        /// </summary>
        public static void Update_Grounding<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            Entity characterEntity,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            ref float2 characterPosition)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            bool newIsGrounded = false;
            BasicHit2D newGroundHit = default;

            if (characterProperties.EvaluateGrounding)
            {
                // Probe length: a short fixed probe normally; the snapping distance when SnapToGround and we were
                // grounded last step (so a character keeps stuck to a downward slope edge it is walking over).
                float groundDetectionLength = Constants.CollisionOffset * 3f;
                if (characterProperties.SnapToGround && characterBody.WasGroundedBeforeCharacterUpdate)
                {
                    groundDetectionLength = characterProperties.GroundSnappingDistance;
                }

                GroundDetection(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterEntity,
                    characterPosition,
                    characterRotation,
                    in characterBody,
                    in characterProperties,
                    in colliderProxy,
                    groundDetectionLength,
                    out newIsGrounded,
                    out newGroundHit,
                    out float distanceToGround);

                // Ground snapping: pull the character down onto the ground hit, then lift by the collision offset.
                if (characterProperties.SnapToGround && newIsGrounded)
                {
                    characterPosition -= characterBody.GroundingUp * distanceToGround;
                    characterPosition += characterBody.GroundingUp * Constants.CollisionOffset;
                }

                if (newIsGrounded)
                {
                    KinematicCharacterHit2D groundCharacterHit = CreateCharacterHit(
                        in newGroundHit,
                        characterBody.WasGroundedBeforeCharacterUpdate,
                        characterBody.RelativeVelocity,
                        newIsGrounded);
                    velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit2D(groundCharacterHit));

                    bool tmpIsGrounded = characterBody.WasGroundedBeforeCharacterUpdate;
                    processor.ProjectVelocityOnHits(
                        ref context,
                        ref baseContext,
                        ref characterBody.RelativeVelocity,
                        ref tmpIsGrounded,
                        ref newGroundHit,
                        in velocityProjectionHitsBuffer,
                        normalizesafe(characterBody.RelativeVelocity));

                    groundCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                    characterHitsBuffer.Add(groundCharacterHit);
                }
            }

            characterBody.IsGrounded = newIsGrounded;
            characterBody.GroundHit = newGroundHit;
        }

        /// <summary>
        /// Sweeps the cast proxy downward along <c>-GroundingUp</c> for <paramref name="groundProbingLength"/>,
        /// filters the hits to the closest obstructing one (<see cref="FilterColliderCastHitsForGroundProbing"/>),
        /// asks the processor whether that hit grounds the character, and — if not — tries the remaining hits in
        /// ascending distance within <see cref="Constants.GroundedHitDistanceTolerance"/>. 2D port of
        /// <c>GroundDetection</c> (REF/KinematicCharacterUtilities.cs:2449). The 3D
        /// <c>EnhancedGroundPrecision</c> distance correction relied on <c>CalculateDistance</c> against a mesh
        /// leaf, which the substrate does not expose — that refinement is dropped (the substrate's casts already
        /// return a precise fraction).
        /// </summary>
        public static void GroundDetection<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            float2 characterPosition,
            float characterRotation,
            in KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float groundProbingLength,
            out bool isGrounded,
            out BasicHit2D groundHit,
            out float distanceToGround)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            isGrounded = false;
            groundHit = default;
            distanceToGround = 0f;

            float2 castDirection = -characterBody.GroundingUp;
            CastProxy(
                ref baseContext,
                in colliderProxy,
                characterPosition,
                characterRotation,
                castDirection,
                groundProbingLength);

            if (FilterColliderCastHitsForGroundProbing(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterEntity,
                    castDirection,
                    characterProperties.ShouldIgnoreDynamicBodies(),
                    out PhysicsQueryHit2D closestHit,
                    out int closestHitIndex))
            {
                groundHit = new BasicHit2D(closestHit);
                distanceToGround = closestHit.fraction * groundProbingLength;

                if (characterProperties.EvaluateGrounding)
                {
                    bool isGroundedOnClosestHit = processor.IsGroundedOnHit(
                        ref context,
                        ref baseContext,
                        in groundHit,
                        (int)GroundingEvaluationType2D.GroundProbing);

                    if (isGroundedOnClosestHit)
                    {
                        isGrounded = true;
                    }
                    else if (baseContext.TmpQueryHits.Length > 1)
                    {
                        // The substrate already returns hits nearest-first (WorldCastMode.AllSorted), so the 3D
                        // explicit Sort is unnecessary — the list IS the ascending-fraction order the reference
                        // sorts to produce. Walk it, skip the already-rejected closest hit, and accept the first
                        // grounded hit within tolerance distance.
                        for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
                        {
                            if (i == closestHitIndex)
                                continue;

                            PhysicsQueryHit2D tmpHit = baseContext.TmpQueryHits[i];
                            float tmpHitDistance = tmpHit.fraction * groundProbingLength;

                            if (distancesq(tmpHitDistance, distanceToGround) <= Constants.GroundedHitDistanceToleranceSq)
                            {
                                BasicHit2D tmpClosestGroundedHit = new BasicHit2D(tmpHit);
                                bool isGroundedOnHit = processor.IsGroundedOnHit(
                                    ref context,
                                    ref baseContext,
                                    in tmpClosestGroundedHit,
                                    (int)GroundingEvaluationType2D.GroundProbing);
                                if (isGroundedOnHit)
                                {
                                    isGrounded = true;
                                    distanceToGround = tmpHitDistance;
                                    groundHit = tmpClosestGroundedHit;
                                    break;
                                }
                            }
                            else
                            {
                                // Sorted ascending — once past the tolerance, no further hit qualifies.
                                break;
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Filters the just-cast hit list to the closest obstructing hit (normal opposing the cast direction),
        /// skipping the character itself, optionally dynamic bodies, and any hit the processor rejects. Returns the
        /// closest accepted hit and its INDEX in the (still-sorted) scratch list so the caller can skip it while
        /// walking the rest. 2D port of <c>FilterColliderCastHitsForGroundProbing</c>
        /// (REF/KinematicCharacterUtilities.cs:2596).
        ///
        /// <para>Unlike the 3D filter (which removes rejected hits with <c>RemoveAtSwapBack</c>, destroying the
        /// sort), this 2D version PRESERVES the substrate's nearest-first order by not mutating the list — the
        /// tolerance walk in <see cref="GroundDetection{T,C}"/> relies on the list staying sorted. Rejected hits
        /// are simply skipped, not removed.</para>
        /// </summary>
        static bool FilterColliderCastHitsForGroundProbing<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            float2 castDirection,
            bool ignoreDynamicBodies,
            out PhysicsQueryHit2D closestHit,
            out int closestHitIndex)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            closestHit = default;
            closestHitIndex = -1;
            float closestFraction = float.MaxValue;
            bool found = false;

            for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
            {
                PhysicsQueryHit2D hit = baseContext.TmpQueryHits[i];

                if (hit.entity == characterEntity || hit.entity == Entity.Null)
                    continue;

                // Ignore hits we are moving away from (normal not opposing the cast).
                float dotRatio = dot(hit.normal, castDirection);
                if (dotRatio >= -Constants.DotProductSimilarityEpsilon)
                    continue;

                // C4b: dynamic-body skip. The kinematic-only core treats every body as collidable static; a body
                // is "dynamic" only once C4b can read its motion. ignoreDynamicBodies is plumbed through verbatim,
                // but with no IsBodyDynamic predicate yet it is a no-op here (no hit is classified dynamic).

                if (!processor.CanCollideWithHit(ref context, ref baseContext, new BasicHit2D(hit)))
                    continue;

                if (hit.fraction < closestFraction)
                {
                    closestFraction = hit.fraction;
                    closestHit = hit;
                    closestHitIndex = i;
                    found = true;
                }
            }

            return found;
        }

        // C4b: Update_PreventGroundingFromFutureSlopeChange (REF/KinematicCharacterUtilities.cs:817) plugs in HERE,
        // between Grounding and MovementAndDecollisions. It raycasts ahead/down along the velocity to predict a
        // no-grounding or too-steep downward slope change and forces IsGrounded=false so the character launches off
        // a ledge cleanly. Omitted from the core; it would use PhysicsQueries2D.RaycastClosest and the step/slope
        // params already on the entity.

        // =====================================================================================================
        // Step 5 — Movement and decollisions (REF/KinematicCharacterUtilities.cs:734)
        // =====================================================================================================

        /// <summary>
        /// The movement-and-decollision step: collide-and-slide (<see cref="MoveWithCollisions{T,C}"/>) then
        /// depenetration (<see cref="SolveOverlaps{T,C}"/>). 2D port of <c>Update_MovementAndDecollisions</c>
        /// (REF/KinematicCharacterUtilities.cs:734). The third 3D sub-phase, hit dynamics
        /// (<c>ProcessCharacterHitDynamics</c>, REF :3166), is chunk C4b — its call site is the marked seam at the
        /// end of this method.
        /// </summary>
        public static void Update_MovementAndDecollisions<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            ref float2 characterPosition)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            float2 originalVelocityDirectionBeforeMove = normalizesafe(characterBody.RelativeVelocity);

            MoveWithCollisions(
                in processor,
                ref context,
                ref baseContext,
                ref characterEntity,
                ref characterBody,
                characterRotation,
                in characterProperties,
                in colliderProxy,
                ref characterPosition,
                originalVelocityDirectionBeforeMove,
                characterHitsBuffer,
                velocityProjectionHitsBuffer,
                out bool moveConfirmedThereWereNoOverlaps);

            // Decollide AFTER movement, so the move can first carry us out of an overlap before we push out of it.
            if (characterProperties.DecollideFromOverlaps && !moveConfirmedThereWereNoOverlaps)
            {
                SolveOverlaps(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterEntity,
                    ref characterBody,
                    characterRotation,
                    in characterProperties,
                    in colliderProxy,
                    ref characterPosition,
                    originalVelocityDirectionBeforeMove,
                    in bodyTransformLookup,
                    deferredImpulsesBuffer,
                    velocityProjectionHitsBuffer,
                    characterHitsBuffer);
            }

            // C4b: ProcessCharacterHitDynamics (REF/KinematicCharacterUtilities.cs:3166) plugs in HERE. For each
            // recorded character hit on a body with a real rigidbody (≠ parent), it solves a collision impulse
            // (PhysicsUtilities2D.SolveCollisionImpulses), applies the self-impulse to RelativeVelocity, and emits a
            // deferred impulse on the other body. It reads the other body's velocity — via StoredKinematicCharacterData2D
            // for another character (Burst-clean) or PhysicsUtilities2D.TryGetDynamicBodyMotion for a regular body
            // (MANAGED, main-thread only — C2 D5). The core records every character hit into characterHitsBuffer so
            // C4b has the full set; it just does not yet consume them for dynamics.
            // if (characterHitsBuffer.Length > 0) { ProcessCharacterHitDynamics2D(...); }
        }

        /// <summary>
        /// Collide-and-slide: casts the proxy along the velocity, advances to the closest obstructing hit, projects
        /// velocity, rescales the remaining movement, and repeats up to
        /// <see cref="KinematicCharacterProperties2D.MaxContinuousCollisionsIterations"/>. 2D port of
        /// <c>MoveWithCollisions</c> (REF/KinematicCharacterUtilities.cs:2157).
        ///
        /// <para><b>Overlap detection for the depenetration gate.</b> The 3D <c>FilterColliderCastHitsForMove</c>
        /// reports <c>foundAnyOverlaps</c> by treating a zero-fraction cast hit as an overlap (the 3D cast returns a
        /// zero-distance hit for an already-penetrating body). The substrate's swept cast also reports a
        /// <c>fraction == 0</c> hit when the proxy starts inside a body, so the same zero-fraction test detects
        /// initial overlaps and drives the <c>confirmedNoOverlapsOnLastMoveIteration</c> optimisation exactly as
        /// 3D.</para>
        ///
        /// <para>The 3D <c>ProjectVelocityOnInitialOverlaps</c> pre-pass (REF :2192) used
        /// <c>CalculateDistance</c>, which the substrate lacks; for the core it is omitted (a C4b refinement —
        /// marked below). It only mitigates tunneling when a rotation changes the detected collisions, which a
        /// mostly-upright 2D platformer character does not do.</para>
        /// </summary>
        public static void MoveWithCollisions<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            float characterRotation,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            ref float2 characterPosition,
            float2 originalVelocityDirection,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            out bool confirmedNoOverlapsOnLastMoveIteration)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            confirmedNoOverlapsOnLastMoveIteration = false;

            // Reorient velocity onto the ground line when grounded (magnitude-preserving; zero when velocity is
            // along grounding-up). Lets the character walk along a slope without being slowed by the slope angle.
            if (characterBody.IsGrounded)
            {
                ProjectVelocityOnGrounding(
                    ref characterBody.RelativeVelocity,
                    characterBody.GroundHit.Normal,
                    characterBody.GroundingUp);
            }

            float remainingMovementLength = length(characterBody.RelativeVelocity) * baseContext.Time.DeltaTime;
            float2 remainingMovementDirection = normalizesafe(characterBody.RelativeVelocity);

            // C4b: the ProjectVelocityOnInitialOverlaps pre-pass (REF/KinematicCharacterUtilities.cs:2192) plugs in
            // HERE. It used CalculateDistance (no substrate 1:1); the D2 cast-back of SolveOverlaps already recovers
            // overlap normals, so a C4b refinement can reuse OverlapProxy + the cast-back to seed projection hits
            // from initial overlaps when characterProperties.ProjectVelocityOnInitialOverlaps is set.

            if (characterProperties.DetectMovementCollisions)
            {
                int movementCastIterationsMade = 0;
                while (movementCastIterationsMade < characterProperties.MaxContinuousCollisionsIterations
                       && remainingMovementLength > 0f)
                {
                    confirmedNoOverlapsOnLastMoveIteration = false;

                    float2 castStartPosition = characterPosition;
                    float2 castDirection = remainingMovementDirection;
                    float castLength = remainingMovementLength + Constants.CollisionOffset;

                    CastProxy(
                        ref baseContext,
                        in colliderProxy,
                        castStartPosition,
                        characterRotation,
                        castDirection,
                        castLength);

                    bool foundMovementHit = FilterColliderCastHitsForMove(
                        in processor,
                        ref context,
                        ref baseContext,
                        ref characterEntity,
                        castDirection,
                        Entity.Null,
                        characterProperties.ShouldIgnoreDynamicBodies(),
                        out PhysicsQueryHit2D closestHit,
                        out bool foundAnyOverlaps);

                    if (!foundAnyOverlaps)
                    {
                        confirmedNoOverlapsOnLastMoveIteration = true;
                    }

                    if (foundMovementHit)
                    {
                        BasicHit2D movementHit = new BasicHit2D(closestHit);
                        float movementHitDistance = castLength * closestHit.fraction;
                        movementHitDistance = max(0f, movementHitDistance - Constants.CollisionOffset);

                        bool isGroundedOnMovementHit = false;
                        if (characterProperties.EvaluateGrounding)
                        {
                            isGroundedOnMovementHit = processor.IsGroundedOnHit(
                                ref context,
                                ref baseContext,
                                in movementHit,
                                (int)GroundingEvaluationType2D.MovementHit);
                        }

                        KinematicCharacterHit2D currentCharacterHit = CreateCharacterHit(
                            in movementHit,
                            characterBody.IsGrounded,
                            characterBody.RelativeVelocity,
                            isGroundedOnMovementHit);

                        OnMovementHit(
                            in processor,
                            ref context,
                            ref baseContext,
                            ref characterEntity,
                            ref characterBody,
                            in characterProperties,
                            in colliderProxy,
                            characterRotation,
                            ref characterPosition,
                            velocityProjectionHitsBuffer,
                            ref currentCharacterHit,
                            ref remainingMovementDirection,
                            ref remainingMovementLength,
                            originalVelocityDirection,
                            movementHitDistance);

                        currentCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                        characterHitsBuffer.Add(currentCharacterHit);
                    }
                    else
                    {
                        // No hit — consume the rest of the movement and end the loop.
                        characterPosition += remainingMovementDirection * remainingMovementLength;
                        remainingMovementLength = 0f;
                    }

                    movementCastIterationsMade++;
                }

                // If movement remains after all iterations, optionally kill velocity / discard the leftover move.
                if (remainingMovementLength > 0f)
                {
                    if (characterProperties.KillVelocityWhenExceedMaxIterations)
                    {
                        characterBody.RelativeVelocity = new float2(0f, 0f);
                    }

                    if (!characterProperties.DiscardMovementWhenExceedMaxIterations)
                    {
                        characterPosition += remainingMovementDirection * remainingMovementLength;
                    }
                }
            }
            else
            {
                characterPosition += characterBody.RelativeVelocity * baseContext.Time.DeltaTime;
            }
        }

        /// <summary>
        /// Filters the just-cast move hit list to the closest obstructing hit and reports whether any overlap was
        /// found (a zero-fraction hit = the proxy started inside that body). 2D port of
        /// <c>FilterColliderCastHitsForMove</c> (REF/KinematicCharacterUtilities.cs:2072), keeping the
        /// equal-distance tie-break that prefers the more-obstructing hit (the smaller — more negative — dot of
        /// normal with cast direction).
        /// </summary>
        static bool FilterColliderCastHitsForMove<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            float2 castDirection,
            Entity ignoredEntity,
            bool ignoreDynamicBodies,
            out PhysicsQueryHit2D closestHit,
            out bool foundAnyOverlaps)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            foundAnyOverlaps = false;
            closestHit = default;
            float closestFraction = float.MaxValue;
            float dotRatioOfSelectedHit = float.MaxValue;
            bool found = false;

            for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
            {
                PhysicsQueryHit2D hit = baseContext.TmpQueryHits[i];

                if (hit.entity == ignoredEntity || hit.entity == characterEntity || hit.entity == Entity.Null)
                    continue;

                if (!processor.CanCollideWithHit(ref context, ref baseContext, new BasicHit2D(hit)))
                    continue;

                // A zero-fraction hit means the proxy already overlaps this body — remember it for the
                // depenetration gate. (C4b would also flag a kinematic-vs-dynamic overlap here.)
                if (hit.fraction <= 0f)
                {
                    foundAnyOverlaps = true;
                }

                // C4b: dynamic-body skip — ignoreDynamicBodies is plumbed through but is a no-op until C4b can
                // classify a hit body as dynamic.

                // Ignore hits we are moving away from.
                float dotRatio = dot(hit.normal, castDirection);
                if (dotRatio < -Constants.DotProductSimilarityEpsilon)
                {
                    if (hit.fraction <= closestFraction)
                    {
                        bool isCloser = hit.fraction < closestFraction;
                        if (isCloser || dotRatio < dotRatioOfSelectedHit)
                        {
                            closestHit = hit;
                            closestFraction = hit.fraction;
                            dotRatioOfSelectedHit = dotRatio;
                            found = true;
                        }
                    }
                }
            }

            return found;
        }

        /// <summary>
        /// The "OnMovementHit" core behaviour: advances the character to the hit, records the velocity-projection
        /// hit, projects velocity, and rescales the remaining movement by the projected/original velocity-length
        /// ratio. 2D port of <c>Default_OnMovementHit</c> (REF/KinematicCharacterUtilities.cs:1388) with the
        /// step-up branch lifted out as the marked C4b seam.
        ///
        /// <para>It is an internal static (not the processor callback) so the core solve calls it directly; the
        /// processor interface still exposes <see cref="IKinematicCharacterProcessor2D{C}.OnMovementHit"/> for a
        /// consumer that wants to override the whole behaviour, and the default processor forwards to this.</para>
        /// </summary>
        public static void OnMovementHit<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            ref float2 characterPosition,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float2 originalVelocityDirection,
            float movementHitDistance)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            bool hasSteppedUp = false;

            // C4b: step-up hook. The 3D Default_OnMovementHit calls CheckForSteppingUpHit
            // (REF/KinematicCharacterUtilities.cs:1414) when step-handling is on, the hit is non-grounded, and the
            // character is moving roughly horizontally — it tries to lift the character over a step ≤ MaxStepHeight
            // and sets hasSteppedUp, skipping the slide below. The core leaves the hook here: when C4b adds
            // CheckForSteppingUpHit2D, it sets hasSteppedUp before the velocityProjection-hit is recorded (the 3D
            // order: step-up correction first, THEN add the projection hit). The step params live on the entity's
            // BasicStepAndSlopeHandlingParameters2D; the proxy + characterRotation + position are all in scope here.
            // if (stepHandling && !hit.IsGroundedOnHit && dot(normalizesafe(RelativeVelocity), GroundingUp) > MinVel...)
            //     CheckForSteppingUpHit2D(..., out hasSteppedUp);

            // Record the velocity-projection hit only after a potential step-up correction.
            velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit2D(hit));

            if (!hasSteppedUp)
            {
                // Advance to the hit.
                characterPosition += remainingMovementDirection * movementHitDistance;
                remainingMovementLength -= movementHitDistance;

                // Project velocity on the running set of hit lines.
                float2 velocityBeforeProjection = characterBody.RelativeVelocity;

                processor.ProjectVelocityOnHits(
                    ref context,
                    ref baseContext,
                    ref characterBody.RelativeVelocity,
                    ref characterBody.IsGrounded,
                    ref characterBody.GroundHit,
                    in velocityProjectionHitsBuffer,
                    originalVelocityDirection);

                // Rescale the remaining movement by how much the projection shortened the velocity.
                float beforeLength = length(velocityBeforeProjection);
                float projectedVelocityLengthFactor =
                    beforeLength > 0f ? (length(characterBody.RelativeVelocity) / beforeLength) : 0f;
                remainingMovementLength *= projectedVelocityLengthFactor;
                remainingMovementDirection = normalizesafe(characterBody.RelativeVelocity);
            }
        }

        /// <summary>
        /// The grounded velocity reorient: 100% of magnitude when velocity is parallel to the ground, 0% when it is
        /// along grounding-up, interpolated in between. 2D port of <c>ProjectVelocityOnGrounding</c>
        /// (REF/KinematicCharacterUtilities.cs:2354) — the body is dimension-agnostic and reduces directly with the
        /// 2D <see cref="MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D"/>.
        /// </summary>
        public static void ProjectVelocityOnGrounding(ref float2 velocity, float2 groundNormal, float2 groundingUp)
        {
            if (lengthsq(velocity) > 0f)
            {
                float velocityLength = length(velocity);
                float2 originalDirection = normalizesafe(velocity);
                float2 reorientedDirection = normalizesafe(
                    MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, groundNormal, groundingUp));
                float dotOriginalWithUp = dot(originalDirection, groundingUp);
                float dotReorientedWithUp = dot(reorientedDirection, groundingUp);

                float ratioFromVerticalToSlopeDirection;
                if (dotOriginalWithUp < dotReorientedWithUp)
                {
                    ratioFromVerticalToSlopeDirection =
                        distance(dotOriginalWithUp, -1f) / distance(dotReorientedWithUp, -1f);
                }
                else
                {
                    ratioFromVerticalToSlopeDirection =
                        distance(dotOriginalWithUp, 1f) / distance(dotReorientedWithUp, 1f);
                }

                velocity = reorientedDirection * lerp(0f, velocityLength, ratioFromVerticalToSlopeDirection);
            }
        }

        // =====================================================================================================
        // Step 5b — Depenetration (REF/KinematicCharacterUtilities.cs:2666) via the D2 overlap-cast-back
        // =====================================================================================================

        /// <summary>
        /// Detects penetrating overlaps and decollides the character out of them. 2D port of <c>SolveOverlaps</c>
        /// (REF/KinematicCharacterUtilities.cs:2666), KINEMATIC mode only (the dynamic-mode push-back of overlapping
        /// dynamic bodies is chunk C4b — see the seam below).
        ///
        /// <para><b>The D2 reconstruction (the highest-uncertainty port).</b> The reference's
        /// <c>CalculateDistance</c> returns each overlap's penetration depth + surface normal directly; the
        /// substrate has NO closest-point/distance query — its overlaps report only WHICH shapes overlap, with zero
        /// point/normal/fraction (PhysicsQueries2D.cs:98-108). So each iteration:
        /// <list type="number">
        /// <item><see cref="OverlapProxy"/> at the current pose lists every overlapping shape (no geometry).</item>
        /// <item>For each overlap, <see cref="ReconstructOverlap"/> recovers its normal + depth by a SHORT cast of
        /// the proxy from a point just OUTSIDE the overlapping body back toward the character: it casts from
        /// <c>characterPosition + dirToCharacter * (boundingRadius + margin)</c> (a point guaranteed clear of the
        /// overlap, since the cast origin is the character pushed out along the body→character direction) along
        /// <c>-dirToCharacter</c>, and the first hit on that body gives the contact normal and a fraction from which
        /// the penetration depth follows. The body→character direction is the gradient that pushes the character
        /// apart, so the recovered normal points the right way.</item>
        /// <item>The most-penetrating non-dynamic overlap is decollided via <see cref="DecollideFromHit{T,C}"/>;
        /// the loop repeats up to <see cref="KinematicCharacterProperties2D.MaxOverlapDecollisionIterations"/>.</item>
        /// </list>
        /// This is the behavioural gate after C4a — the cast-back is approximate (it recovers the normal at the
        /// closest surface point along the body→character axis, not the exact MTV), but for a circle/box proxy
        /// against convex world shapes the body→character axis IS the separating axis, so the recovered normal
        /// matches the MTV direction.</para>
        /// </summary>
        public static void SolveOverlaps<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            float characterRotation,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            ref float2 characterPosition,
            float2 originalVelocityDirection,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            int decollisionIterationsMade = 0;
            while (decollisionIterationsMade < characterProperties.MaxOverlapDecollisionIterations)
            {
                decollisionIterationsMade++;

                // (1) Which shapes overlap the proxy at the current pose? (no geometry from the overlap itself)
                int overlapCount = OverlapProxy(
                    ref baseContext,
                    in colliderProxy,
                    characterPosition,
                    characterRotation);

                // The overlap list IS the context scratch list; reconstructing an overlap re-uses that same list
                // for its cast-back, so snapshot the overlapping entities first (a small fixed-cap local set).
                if (overlapCount <= 0)
                {
                    break;
                }

                // (2) Reconstruct each overlap and pick the most-penetrating accepted, non-dynamic one.
                BasicHit2D mostPenetratingHit = default;
                float mostPenetratingDepth = 0f;
                bool foundHitForDecollision = false;

                // Copy overlapping entities out before the cast-back overwrites the scratch list.
                var overlappingEntities = new NativeList<Entity>(overlapCount, Allocator.Temp);
                for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
                {
                    Entity e = baseContext.TmpQueryHits[i].entity;
                    if (e != characterEntity && e != Entity.Null)
                    {
                        overlappingEntities.Add(e);
                    }
                }

                for (int i = 0; i < overlappingEntities.Length; i++)
                {
                    Entity overlapEntity = overlappingEntities[i];

                    if (ReconstructOverlap(
                            ref baseContext,
                            in colliderProxy,
                            characterPosition,
                            characterRotation,
                            overlapEntity,
                            in bodyTransformLookup,
                            out float2 overlapNormal,
                            out float overlapDepth))
                    {
                        BasicHit2D basicOverlapHit = new BasicHit2D(overlapEntity, characterPosition, overlapNormal);

                        if (!processor.CanCollideWithHit(ref context, ref baseContext, in basicOverlapHit))
                            continue;

                        // C4b: dynamic-body classification. The kinematic-only core decollides from every
                        // accepted overlap (treated as non-dynamic). C4b splits this into most-penetrating overall /
                        // dynamic / non-dynamic (the 3D FilterDistanceHitsForSolveOverlaps, REF :3038) and, in
                        // kinematic mode, decollides only from the closest NON-dynamic and pushes back dynamic
                        // bodies with a displacement impulse on the last iteration (REF :2796-2823).

                        if (overlapDepth > mostPenetratingDepth)
                        {
                            mostPenetratingDepth = overlapDepth;
                            mostPenetratingHit = basicOverlapHit;
                            foundHitForDecollision = true;
                        }
                    }
                }

                overlappingEntities.Dispose();

                if (!foundHitForDecollision)
                {
                    break;
                }

                bool isGroundedOnHit = false;
                if (characterProperties.EvaluateGrounding)
                {
                    isGroundedOnHit = processor.IsGroundedOnHit(
                        ref context,
                        ref baseContext,
                        in mostPenetratingHit,
                        (int)GroundingEvaluationType2D.OverlapDecollision);
                }

                DecollideFromHit(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterEntity,
                    ref characterBody,
                    in characterProperties,
                    in colliderProxy,
                    characterRotation,
                    ref characterPosition,
                    in mostPenetratingHit,
                    mostPenetratingDepth,
                    originalVelocityDirection,
                    deferredImpulsesBuffer,
                    velocityProjectionHitsBuffer,
                    characterHitsBuffer,
                    isGroundedOnHit);
            }
        }

        /// <summary>
        /// Recovers an overlapping body's contact normal and penetration depth via the D2 cast-back. The character
        /// proxy is conceptually pushed out to a point guaranteed clear of the overlap (along the
        /// body→character direction, by the proxy's bounding radius plus a margin), then the proxy is swept back
        /// toward the character; the first hit ON THAT BODY gives the surface normal, and the un-travelled distance
        /// is the penetration depth. Returns false if the cast-back finds no hit on the body (a degenerate overlap,
        /// e.g. fully-contained — rare for a character against world geometry).
        /// </summary>
        static bool ReconstructOverlap(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float2 characterPosition,
            float characterRotation,
            Entity overlapEntity,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            out float2 overlapNormal,
            out float overlapDepth)
        {
            overlapNormal = default;
            overlapDepth = 0f;

            // The body's center, from its LocalToWorld — used to choose the body→character separating direction.
            // Every substrate body (static or dynamic) is baked with TransformUsageFlags.Dynamic, so it carries a
            // LocalToWorld the write-back keeps in sync (Collider2DBaking.AddStaticBodyIfNoRigidbody / the body
            // bakers). The matrix translation (c3.xy) is the body origin.
            float2 bodyCenter = characterPosition;
            if (bodyTransformLookup.HasComponent(overlapEntity))
            {
                bodyCenter = bodyTransformLookup[overlapEntity].Value.c3.xy;
            }

            float2 bodyToCharacter = characterPosition - bodyCenter;
            // If the centers coincide (proxy centered on the body), fall back to grounding-down-ish +Y so the
            // cast-back still has a direction; any non-degenerate overlap of a character vs world geometry has a
            // non-zero center offset, so this is a safety fallback only.
            float2 dirToCharacter = normalizesafe(bodyToCharacter, new float2(0f, 1f));

            float boundingRadius = ProxyBoundingRadius(in colliderProxy);
            // Start the cast clear of the overlap: push the proxy out along body→character by its bounding radius
            // plus a margin, so the swept proxy begins fully separated and registers a clean entry contact.
            float clearance = (boundingRadius * 2f) + Constants.CollisionOffset;
            float2 castOrigin = characterPosition + (dirToCharacter * clearance);
            float2 castDirection = -dirToCharacter;
            float castLength = clearance + boundingRadius;

            CastProxy(
                ref baseContext,
                in colliderProxy,
                castOrigin,
                characterRotation,
                castDirection,
                castLength);

            // Find the first (nearest) hit on the target body.
            for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
            {
                PhysicsQueryHit2D hit = baseContext.TmpQueryHits[i];
                if (hit.entity != overlapEntity)
                    continue;

                overlapNormal = normalizesafe(hit.normal, dirToCharacter);

                // Distance from the cast origin to the contact along the cast.
                float distanceToContact = hit.fraction * castLength;
                // The proxy surface reaches the body when its CENTER is `boundingRadius` short of the contact, so
                // the center travels (distanceToContact - boundingRadius) to first touch. The character's current
                // center is `clearance` from the origin along the same axis; the penetration depth is how far the
                // character center is PAST first contact:
                float centerTravelToTouch = distanceToContact - boundingRadius;
                overlapDepth = max(0f, clearance - centerTravelToTouch);
                return true;
            }

            return false;
        }

        private static void RecalculateDecollisionVector(
            ref float2 decollisionVector,
            float2 originalHitNormal,
            float2 newDecollisionDirection,
            float decollisionDistance)
        {
            float overlapDistance = max(decollisionDistance, 0f);
            if (overlapDistance > 0f)
            {
                decollisionVector = MathUtilities2D.ReverseProjectOnVector(
                    originalHitNormal * overlapDistance,
                    newDecollisionDirection,
                    overlapDistance * Constants.DefaultReverseProjectionMaxLengthRatio);
            }
        }

        /// <summary>
        /// Moves the character out of a single overlap by <paramref name="decollisionDistance"/> along the hit
        /// normal (reoriented to grounding-up when grounded on the hit, or onto the ground line when grounded and
        /// the hit is non-grounded), records the projection hit, projects velocity, and records a character hit. 2D
        /// port of <c>DecollideFromHit</c> (REF/KinematicCharacterUtilities.cs:2907), KINEMATIC + non-dynamic-hit
        /// path only.
        ///
        /// <para>The 3D <c>characterSimulateDynamic &amp;&amp; hitIsDynamic</c> branch (obstruction-checked
        /// decollision off a dynamic body + a displacement impulse, REF :2952) is chunk C4b — the core always takes
        /// the "fully decollide" path because every hit is treated non-dynamic. The seam is marked below.</para>
        /// </summary>
        public static void DecollideFromHit<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            ref float2 characterPosition,
            in BasicHit2D hit,
            float decollisionDistance,
            float2 originalVelocityDirection,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            bool isGroundedOnHit)
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            float2 decollisionDirection = hit.Normal;
            float2 decollisionVector = decollisionDirection * decollisionDistance;

            // Always decollide vertically from grounded hits (so we pop straight up out of the floor, not along the
            // slope normal which would slide us sideways).
            if (isGroundedOnHit)
            {
                if (dot(characterBody.GroundingUp, hit.Normal) > Constants.MinDotRatioForVerticalDecollision)
                {
                    decollisionDirection = characterBody.GroundingUp;
                    RecalculateDecollisionVector(ref decollisionVector, hit.Normal, decollisionDirection, decollisionDistance);
                }
            }
            // If grounded and the hit is non-grounded (a wall while standing), decollide along the ground line so
            // pushing out of the wall does not lift the character off the ground.
            else if (characterBody.IsGrounded)
            {
                decollisionDirection = normalizesafe(
                    MathUtilities2D.ProjectOnPlane(decollisionDirection, characterBody.GroundHit.Normal));
                RecalculateDecollisionVector(ref decollisionVector, hit.Normal, decollisionDirection, decollisionDistance);
            }

            // C4b: dynamic-body decollision. The 3D code, when simulateDynamic AND the hit body is dynamic, casts
            // along the decollision direction first and, if obstructed by a THIRD body, moves only as far as the
            // obstruction and emits a displacement impulse to push the dynamic body the rest of the way
            // (REF/KinematicCharacterUtilities.cs:2952-2984). The kinematic-only core always fully decollides.
            characterPosition += decollisionVector;

            // Velocity projection on the obstructing overlap.
            float2 characterRelativeVelocityBeforeProjection = characterBody.RelativeVelocity;
            velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit2D(hit, isGroundedOnHit));

            if (dot(characterBody.RelativeVelocity, hit.Normal) < 0f)
            {
                processor.ProjectVelocityOnHits(
                    ref context,
                    ref baseContext,
                    ref characterBody.RelativeVelocity,
                    ref characterBody.IsGrounded,
                    ref characterBody.GroundHit,
                    in velocityProjectionHitsBuffer,
                    originalVelocityDirection);
            }

            KinematicCharacterHit2D overlapCharacterHit = CreateCharacterHit(
                in hit,
                characterBody.IsGrounded,
                characterRelativeVelocityBeforeProjection,
                isGroundedOnHit);
            overlapCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
            characterHitsBuffer.Add(overlapCharacterHit);
        }

        // =====================================================================================================
        // Default processor callbacks (REF/KinematicCharacterUtilities.cs:1131,1185,1202)
        // =====================================================================================================

        /// <summary>
        /// Default grounding test for a hit: the slope-angle check (<see cref="IsGroundedOnSlopeNormal"/>) plus the
        /// going-away-from-ground velocity guard (<see cref="ShouldPreventGroundingBasedOnVelocity"/>). 2D port of
        /// <c>Default_IsGroundedOnHit</c> (REF/KinematicCharacterUtilities.cs:1131) with the step-grounding branch
        /// removed (step handling is chunk C4b — the seam is marked).
        /// </summary>
        public static bool Default_IsGroundedOnHit(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in BasicHit2D hit,
            int groundingEvaluationType)
        {
            if (ShouldPreventGroundingBasedOnVelocity(
                    in hit,
                    characterBody.WasGroundedBeforeCharacterUpdate,
                    characterBody.RelativeVelocity))
            {
                return false;
            }

            bool isGroundedOnSlope = IsGroundedOnSlopeNormal(
                characterProperties.MaxGroundedSlopeDotProduct,
                hit.Normal,
                characterBody.GroundingUp);

            // C4b: step-grounding. The 3D Default_IsGroundedOnHit, when NOT grounded on the slope and step handling
            // is on, runs IsGroundedOnSteps (REF/KinematicCharacterUtilities.cs:1151-1175) — a set of extra
            // downward raycasts that let a round/box base be grounded on a step edge. The core returns the slope
            // result alone; C4b ORs in the step result. The step params (StepHandling/MaxStepHeight/...) live on the
            // entity's BasicStepAndSlopeHandlingParameters2D, which the default processor can pass in when C4b lands.

            return isGroundedOnSlope;
        }

        /// <summary>
        /// Whether a slope is shallow enough to ground on: <c>dot(groundingUp, normal) &gt;
        /// maxGroundedSlopeDotProduct</c>. Verbatim 2D port of <c>IsGroundedOnSlopeNormal</c>
        /// (REF/KinematicCharacterUtilities.cs:3532) — scalar, no dimensional content.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsGroundedOnSlopeNormal(
            float maxGroundedSlopeDotProduct,
            float2 slopeSurfaceNormal,
            float2 groundingUp)
        {
            return dot(groundingUp, slopeSurfaceNormal) > maxGroundedSlopeDotProduct;
        }

        /// <summary>
        /// Prevents grounding when the character is airborne and moving away from the ground normal (so it does not
        /// snap to a wall it is launching off). 2D port of <c>ShouldPreventGroundingBasedOnVelocity</c>
        /// (REF/KinematicCharacterUtilities.cs:3618).
        ///
        /// <para>The 3D version, for a DYNAMIC ground body, read that body's velocity and only prevented grounding
        /// if the character was escaping the ground's own velocity. The kinematic-only core has no body-velocity
        /// read on the Burst path (C2 D5 — the read is managed), so it always takes the "ground has no velocity"
        /// branch: a non-character ground body is treated as stationary. This is exactly correct for static world
        /// geometry. C4b restores the dynamic-ground-velocity comparison via the main-thread
        /// <see cref="PhysicsUtilities2D.TryGetDynamicBodyMotion"/> — the seam is marked.</para>
        /// </summary>
        public static bool ShouldPreventGroundingBasedOnVelocity(
            in BasicHit2D hit,
            bool wasGroundedBeforeCharacterUpdate,
            float2 relativeVelocity)
        {
            if (!wasGroundedBeforeCharacterUpdate
                && dot(relativeVelocity, hit.Normal) > Constants.DotProductSimilarityEpsilon
                && lengthsq(relativeVelocity) > Constants.MinVelocityLengthSqForGroundingIgnoreCheck)
            {
                // C4b: dynamic-ground-velocity comparison. For a dynamic ground body the 3D code compares the
                // character's normal-velocity against the ground's normal-velocity and only prevents grounding when
                // the character is escaping (REF/KinematicCharacterUtilities.cs:3630-3648). The core treats every
                // ground body as stationary (the "else" branch), correct for static geometry.
                return true;
            }

            return false;
        }

        /// <summary>
        /// The 2D velocity projection — the algorithm's hardest math, collapsed to corner-only (design D6). 2D port
        /// of <c>Default_ProjectVelocityOnHits</c> (REF/KinematicCharacterUtilities.cs:1202).
        ///
        /// <para><b>The D6 collapse.</b> In 3D a surface is a plane and the projection walks pairs of planes to find
        /// CREASES (the 1-D line <c>cross(planeA, planeB)</c> where two planes meet) and CORNERS (a third plane the
        /// crease-projected velocity would re-enter → kill velocity). In 2D every surface is a LINE and every normal
        /// is a <c>float2</c>; two non-parallel lines always meet at a POINT, so there is no crease — only the
        /// corner case. The triple-nested 3D plane walk (REF :1292 second-hit loop, :1335 third-hit loop) therefore
        /// collapses to: project onto the latest blocking line; then test every PRIOR line — if the projected
        /// velocity would re-enter one (its dot with that line's normal goes negative), the character is wedged in a
        /// 2D corner → kill velocity. No cross product, no crease reorient.</para>
        ///
        /// <para>The single-hit projection sub-behaviour (grounded/ungrounded × hit-grounded/not) and the
        /// ground-replacement bookkeeping port verbatim from the 3D <c>ProjectVelocityOnSingleHit</c> with
        /// <c>float2</c> — the crease branch of that sub-routine (REF :1230 <c>cross</c>) becomes the regular line
        /// projection, since a "crease between ground normal and obstruction" in 2D is just the single direction
        /// orthogonal to the obstruction line constrained to the ground line, which is the corner case handled by
        /// the outer kill.</para>
        /// </summary>
        public static void Default_ProjectVelocityOnHits(
            ref float2 velocity,
            ref bool characterIsGrounded,
            ref BasicHit2D characterGroundHit,
            in DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            float2 originalVelocityDirection,
            bool constrainToGroundPlane,
            in KinematicCharacterBody2D characterBody)
        {
            if (lengthsq(velocity) <= 0f || lengthsq(originalVelocityDirection) <= 0f)
            {
                return;
            }

            int hitsCount = velocityProjectionHitsBuffer.Length;
            int firstHitIndex = velocityProjectionHitsBuffer.Length - 1;
            KinematicVelocityProjectionHit2D firstHit = velocityProjectionHitsBuffer[firstHitIndex];
            float2 velocityDirection = normalizesafe(velocity);

            if (dot(velocityDirection, firstHit.Normal) < 0f)
            {
                // Project on the most-recent blocking line.
                ProjectVelocityOnSingleHit(
                    ref velocity,
                    ref characterIsGrounded,
                    ref characterGroundHit,
                    in firstHit,
                    characterBody.GroundingUp,
                    constrainToGroundPlane);
                velocityDirection = normalizesafe(velocity);

                // The original velocity direction acts as a constraint line too (index -1), preventing velocity from
                // reversing back the way it came. Its "normal" is the original direction itself (projected onto the
                // ground plane when grounded), so a re-entry test against it catches a U-turn corner.
                KinematicVelocityProjectionHit2D originalVelocityHit = default;
                originalVelocityHit.Normal = characterIsGrounded
                    ? normalizesafe(MathUtilities2D.ProjectOnPlane(originalVelocityDirection, characterBody.GroundingUp))
                    : originalVelocityDirection;

                // Corner detection: after projecting on the first line, does the projected velocity re-enter any
                // OTHER previously-detected line? In 2D, re-entering a second non-parallel line means the two lines
                // wedge the character into a corner → kill velocity. (There is no crease to project onto; two lines
                // meet at a point.)
                for (int secondHitIndex = -1; secondHitIndex < hitsCount; secondHitIndex++)
                {
                    if (secondHitIndex == firstHitIndex)
                        continue;

                    KinematicVelocityProjectionHit2D secondHit = originalVelocityHit;
                    if (secondHitIndex >= 0)
                    {
                        secondHit = velocityProjectionHitsBuffer[secondHitIndex];
                    }

                    if (IsSamePlane(firstHit.Normal, secondHit.Normal))
                        continue;

                    // Would the projected velocity drive into this second line?
                    if (dot(velocityDirection, secondHit.Normal) > -Constants.DotProductSimilarityEpsilon)
                        continue;

                    // The velocity re-enters a non-parallel line after projecting on the first → wedged corner.
                    // (The 3D code, after detecting this crease, would project onto the crease line and then re-scan
                    // for a third plane; in 2D the crease IS a point, so the character cannot move along it — kill.)
                    velocity = new float2(0f, 0f);
                    break;
                }
            }
        }

        /// <summary>
        /// Whether two line normals describe the same line (within the similarity epsilon). 2D port of the 3D
        /// <c>IsSamePlane</c> local function (REF/KinematicCharacterUtilities.cs:1211).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsSamePlane(float2 planeA, float2 planeB)
        {
            return dot(planeA, planeB) > (1f - Constants.DotProductSimilarityEpsilon);
        }

        /// <summary>
        /// Projects velocity on a single hit line, branching on the grounded × hit-grounded cases exactly as the 3D
        /// <c>ProjectVelocityOnSingleHit</c> local function (REF/KinematicCharacterUtilities.cs:1216), and replaces
        /// the character's effective ground when the hit grounds it. The 3D crease branch (project on
        /// <c>cross(groundNormal, hitNormal)</c>) becomes the regular line projection in 2D — see the D6 note on
        /// <see cref="Default_ProjectVelocityOnHits"/>.
        /// </summary>
        static void ProjectVelocityOnSingleHit(
            ref float2 velocity,
            ref bool characterIsGrounded,
            ref BasicHit2D characterGroundHit,
            in KinematicVelocityProjectionHit2D hit,
            float2 groundingUp,
            bool constrainToGroundPlane)
        {
            if (characterIsGrounded)
            {
                if (hit.IsGroundedOnHit)
                {
                    velocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, hit.Normal, groundingUp);
                }
                else
                {
                    if (constrainToGroundPlane)
                    {
                        // 3D projected onto the crease formed by the ground normal and the obstruction. In 2D the
                        // "crease" of a ground line and a wall line is the single point where they meet, so velocity
                        // along it is zero — but the corner-kill in the caller handles the wedge. Here, the
                        // ground-constrained slide is: project onto the ground line first (stay on the ground), then
                        // onto the obstruction line (stop entering the wall).
                        velocity = MathUtilities2D.ProjectOnPlane(velocity, characterGroundHit.Normal);
                        velocity = MathUtilities2D.ProjectOnPlane(velocity, hit.Normal);
                    }
                    else
                    {
                        velocity = MathUtilities2D.ProjectOnPlane(velocity, hit.Normal);
                    }
                }
            }
            else
            {
                if (hit.IsGroundedOnHit)
                {
                    // Grounded landing: kill vertical velocity, then reorient onto the ground line.
                    velocity = MathUtilities2D.ProjectOnPlane(velocity, groundingUp);
                    velocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, hit.Normal, groundingUp);
                }
                else
                {
                    velocity = MathUtilities2D.ProjectOnPlane(velocity, hit.Normal);
                }
            }

            // Replace the effective ground when the hit grounds the character (or when not constraining to ground).
            if (hit.IsGroundedOnHit || !constrainToGroundPlane)
            {
                if (hit.Entity != Entity.Null)
                {
                    if (dot(groundingUp, hit.Normal) > Constants.DotProductSimilarityEpsilon)
                    {
                        characterIsGrounded = hit.IsGroundedOnHit;
                        characterGroundHit = new BasicHit2D(hit.Entity, hit.Position, hit.Normal);
                    }
                }
            }
        }
    }
}
