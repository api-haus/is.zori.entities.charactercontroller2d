using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
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
            /// Horizontal offset of the extra step-grounding raycasts (2D port of
            /// <c>StepGroundingDetectionHorizontalOffset</c>, REF/KinematicCharacterUtilities.cs:476).
            /// </summary>
            public const float StepGroundingDetectionHorizontalOffset = 0.01f;

            /// <summary>
            /// Minimum dot of the (normalized) velocity with grounding-up for a movement hit to be considered for
            /// stepping up. Negative because a character is usually moving slightly down a slope while still wanting
            /// to step up over a forward step (2D port of
            /// <c>MinVelocityDotRatioWithGroundingUpForSteppingUpHits</c>, REF/KinematicCharacterUtilities.cs:480).
            /// </summary>
            public const float MinVelocityDotRatioWithGroundingUpForSteppingUpHits = -0.85f;

            /// <summary>
            /// The layer mask the controller's casts/overlaps use. C4a hits every layer (the
            /// <see cref="PhysicsQueries2D"/> "0 means all" convention); a later chunk may surface a per-character
            /// collision filter. The <see cref="IKinematicCharacterProcessor2D{C}.CanCollideWithHit"/> callback is
            /// the per-hit accept/reject gate in the meantime.
            /// </summary>
            public const ulong CharacterHitLayerMask = 0ul;
        }

        // =====================================================================================================
        // Cast-proxy query wrappers — the single place the circle/box/capsule proxy (design D1) chooses CircleCast
        // vs BoxCast vs CapsuleCast / OverlapCircle vs OverlapBox vs OverlapCapsule. The reference cast the
        // character's actual collider; the substrate offers circle/box/capsule casts (PhysicsQueries2D.cs), so the
        // proxy decides the shape. A capsule's two local end-cap centers are translated by the character centre
        // before casting (the proxy stores them in the body's local frame, matching the baked world shape).
        // =====================================================================================================

        /// <summary>
        /// Sweeps the character's cast proxy from <paramref name="origin"/> along <paramref name="direction"/> for
        /// <paramref name="distance"/>, writing nearest-first hits into the context scratch list. Dispatches to
        /// <see cref="PhysicsQueries2D.CircleCast"/>, <see cref="PhysicsQueries2D.BoxCast"/>, or
        /// <see cref="PhysicsQueries2D.CapsuleCast"/> by proxy
        /// <see cref="KinematicCharacterColliderProxy2D.Kind"/>. The proxy box is swept axis-aligned at the
        /// character's z-rotation (<paramref name="rotationRadians"/>); the capsule's local centers are offset by
        /// <paramref name="origin"/> (rotation not applied — the character does not rotate).
        /// </summary>
        static int CastProxy(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D proxy,
            float2 origin,
            float rotationRadians,
            float2 direction,
            float distance
        ) =>
            CastProxyInto(
                ref baseContext,
                in proxy,
                origin,
                rotationRadians,
                direction,
                distance,
                baseContext.TmpQueryHits
            );

        // The proxy cast against an explicit target hit list. The OUTER move/grounding/overlap iterations cast into
        // TmpQueryHits (the list they then walk); the INNER step/depenetration helpers cast into TmpInnerQueryHits so
        // they never clobber the outer iteration's list (see KinematicCharacterUpdateContext2D.TmpInnerQueryHits).
        static int CastProxyInto(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D proxy,
            float2 origin,
            float rotationRadians,
            float2 direction,
            float distance,
            NativeList<PhysicsQueryHit2D> hits
        )
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
                    hits
                );
            }

            if (proxy.Kind == PhysicsShape2DKind.Capsule)
            {
                return PhysicsQueries2D.CapsuleCast(
                    baseContext.PhysicsWorld,
                    origin + proxy.CapsuleCenter1,
                    origin + proxy.CapsuleCenter2,
                    proxy.Radius,
                    direction,
                    distance,
                    Constants.CharacterHitLayerMask,
                    hits
                );
            }

            return PhysicsQueries2D.CircleCast(
                baseContext.PhysicsWorld,
                origin,
                proxy.Radius,
                direction,
                distance,
                Constants.CharacterHitLayerMask,
                hits
            );
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
            float rotationRadians
        )
        {
            if (proxy.Kind == PhysicsShape2DKind.Box)
            {
                return PhysicsQueries2D.OverlapBox(
                    baseContext.PhysicsWorld,
                    center,
                    proxy.BoxSize,
                    rotationRadians,
                    Constants.CharacterHitLayerMask,
                    baseContext.TmpQueryHits
                );
            }

            if (proxy.Kind == PhysicsShape2DKind.Capsule)
            {
                return PhysicsQueries2D.OverlapCapsule(
                    baseContext.PhysicsWorld,
                    center + proxy.CapsuleCenter1,
                    center + proxy.CapsuleCenter2,
                    proxy.Radius,
                    Constants.CharacterHitLayerMask,
                    baseContext.TmpQueryHits
                );
            }

            return PhysicsQueries2D.OverlapCircle(
                baseContext.PhysicsWorld,
                center,
                proxy.Radius,
                Constants.CharacterHitLayerMask,
                baseContext.TmpQueryHits
            );
        }

        /// <summary>
        /// The proxy's bounding radius — half the box diagonal for a box, half the capsule's tip-to-tip span (the
        /// segment half-length plus the cap radius) for a capsule, the radius for a circle. The D2 cast-back
        /// starts from a point this far outside the overlapping shape and casts back toward the character,
        /// guaranteeing the cast begins clear of the overlap so it can register a clean contact normal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float ProxyBoundingRadius(in KinematicCharacterColliderProxy2D proxy)
        {
            if (proxy.Kind == PhysicsShape2DKind.Box)
            {
                return length(proxy.BoxSize) * 0.5f;
            }

            if (proxy.Kind == PhysicsShape2DKind.Capsule)
            {
                return length(proxy.CapsuleCenter2 - proxy.CapsuleCenter1) * 0.5f + proxy.Radius;
            }

            return proxy.Radius;
        }

        /// <summary>
        /// Whether a hit body is a regular dynamic body, read from the main-thread <see cref="StoredDynamicBodyData2D"/>
        /// snapshot on the base context (the D5 resolution — Burst-safe component read, never the live handle). A
        /// character body or a static body returns false (a character carries no dynamic snapshot; a static body's
        /// snapshot, if any, has <c>IsDynamic == false</c>).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsHitDynamic(ref KinematicCharacterUpdateContext2D baseContext, Entity entity)
        {
            return baseContext.DynamicBodyDataLookup.HasComponent(entity)
                && baseContext.DynamicBodyDataLookup[entity].IsDynamic;
        }

        /// <summary>
        /// Whether a query hit is on a SENSOR (<c>isTrigger</c>) shape, read off the raw Box2D shape the substrate
        /// carries on every hit (<see cref="PhysicsQueryHit2D.shape"/>). A kinematic character controller treats a
        /// sensor as NON-SOLID to its collision/grounding sweeps: the character passes THROUGH a sensor volume (a
        /// zone, a teleporter pad) as a visitor rather than grounding on it or sliding off it. Every accept/reject
        /// site in the solve — ground probing, the move sweep, the step/slope raycasts, and the overlap
        /// depenetration — skips a sensor hit through this predicate, so the substrate's casts (which return sensor
        /// shapes — Box2D's <c>QueryFilter</c> has no sensor exclusion, only a layer mask) never make the controller
        /// stand on a trigger. The separate trigger-EVENT channel (<see cref="PhysicsTriggerEvent2D"/>) still reports
        /// the character entering/leaving the sensor — this predicate only governs the COLLISION/GROUNDING response,
        /// not event reporting.
        ///
        /// <para><b>Burst.</b> <c>PhysicsShape.isTrigger</c> is a <c>[NativeMethod(IsThreadSafe = true)]</c> binding
        /// (<c>Scripting2D.PhysicsShape_GetIsTrigger</c>), the same thread-safe class as <c>shape.isValid</c> and
        /// <c>shape.body</c> that <see cref="PhysicsQueries2D.ResolveEntity"/> / <see cref="PhysicsQueries2D.ClosestPoint"/>
        /// already call from the HPC#-clean substrate query surface, so it is safe to read from this Bursted solve.
        /// A degenerate (invalid) shape is treated as non-sensor — the entity/null guards alongside this predicate
        /// already drop a hit with no resolvable owner.</para>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool IsSensorHit(in PhysicsQueryHit2D hit)
        {
            return hit.shape.isValid && hit.shape.isTrigger;
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
            bool isGroundedOnHit
        )
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
            float deltaTime
        )
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

        // Update_ParentMovement (the C4b parent-movement feature) chains between Initialize and Grounding — its call
        // site is in KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D; the method itself is in the C4b block below.

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
            ref float2 characterPosition
        )
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
                    out float distanceToGround
                );

                // Ground snapping: pull the character down onto the ground hit, then lift by the collision offset.
                if (characterProperties.SnapToGround && newIsGrounded)
                {
                    // Suppress the downward snap while a just-stepped-up character's centre still overhangs the lower
                    // surface it climbed from. The accepted ground hit there is the FAR lower floor (the box edge is
                    // flush against the step's vertical face, so the step top is only a zero-fraction overlap with no
                    // opposing normal — see CheckForSteppingUpHit / SuppressGroundSnappingUntilSteppedClear). Snapping
                    // down onto that far floor would drive the character back off the step. Hold the snap until the
                    // centre clears the edge and the step top is itself the accepted ground hit at a near distance,
                    // then clear the flag and resume normal snapping. A "near" hit means the character is genuinely
                    // resting on its accepted ground (the normal grounded state), so the suppression is over.
                    bool acceptedGroundIsNear = distanceToGround <= Constants.GroundedHitDistanceTolerance;
                    if (characterBody.SuppressGroundSnappingUntilSteppedClear && !acceptedGroundIsNear)
                    {
                        // Hold position: do not pull down onto the far floor. The character stays on the step it
                        // climbed; subsequent steps advance its centre over the step edge.
                    }
                    else
                    {
                        characterPosition -= characterBody.GroundingUp * distanceToGround;
                        characterPosition += characterBody.GroundingUp * Constants.CollisionOffset;
                        characterBody.SuppressGroundSnappingUntilSteppedClear = false;
                    }
                }

                if (newIsGrounded)
                {
                    KinematicCharacterHit2D groundCharacterHit = CreateCharacterHit(
                        in newGroundHit,
                        characterBody.WasGroundedBeforeCharacterUpdate,
                        characterBody.RelativeVelocity,
                        newIsGrounded
                    );
                    velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit2D(groundCharacterHit));

                    bool tmpIsGrounded = characterBody.WasGroundedBeforeCharacterUpdate;
                    processor.ProjectVelocityOnHits(
                        ref context,
                        ref baseContext,
                        ref characterBody.RelativeVelocity,
                        ref tmpIsGrounded,
                        ref newGroundHit,
                        in velocityProjectionHitsBuffer,
                        normalizesafe(characterBody.RelativeVelocity)
                    );

                    groundCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                    characterHitsBuffer.Add(groundCharacterHit);
                }
            }

            characterBody.IsGrounded = newIsGrounded;
            characterBody.GroundHit = newGroundHit;

            // Clear the stepped-up snap suppression if grounding did not run (EvaluateGrounding off), if snapping is
            // off (the suppression only guards the snap), or if the character is no longer grounded (it left the step
            // — e.g. jumped or walked off) so a stale flag never lingers into an unrelated future step-up.
            if (
                !characterProperties.EvaluateGrounding //
                || !characterProperties.SnapToGround //
                || !newIsGrounded //
            )
            {
                characterBody.SuppressGroundSnappingUntilSteppedClear = false;
            }
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
            out float distanceToGround
        )
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
                groundProbingLength
            );

            if (
                FilterColliderCastHitsForGroundProbing(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterEntity,
                    castDirection,
                    characterProperties.ShouldIgnoreDynamicBodies(),
                    out PhysicsQueryHit2D closestHit,
                    out int closestHitIndex
                )
            )
            {
                groundHit = new BasicHit2D(closestHit);
                distanceToGround = closestHit.fraction * groundProbingLength;

                if (characterProperties.EvaluateGrounding)
                {
                    bool isGroundedOnClosestHit = processor.IsGroundedOnHit(
                        ref context,
                        ref baseContext,
                        in groundHit,
                        (int)GroundingEvaluationType2D.GroundProbing
                    );

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

                            // A sensor within tolerance of the closest hit is not solid ground — skip it (the
                            // FilterColliderCastHitsForGroundProbing closest-hit selection above already skips
                            // sensors; this tolerance walk must too, or the character grounds on a trigger).
                            if (IsSensorHit(in tmpHit))
                                continue;

                            float tmpHitDistance = tmpHit.fraction * groundProbingLength;

                            if (
                                distancesq(tmpHitDistance, distanceToGround) <= Constants.GroundedHitDistanceToleranceSq
                            )
                            {
                                BasicHit2D tmpClosestGroundedHit = new BasicHit2D(tmpHit);
                                bool isGroundedOnHit = processor.IsGroundedOnHit(
                                    ref context,
                                    ref baseContext,
                                    in tmpClosestGroundedHit,
                                    (int)GroundingEvaluationType2D.GroundProbing
                                );
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
            out int closestHitIndex
        )
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

                if (hit.entity == characterEntity || hit.entity == Entity.Null || IsSensorHit(in hit))
                    continue;

                // Ignore hits we are moving away from (normal not opposing the cast).
                float dotRatio = dot(hit.normal, castDirection);
                if (dotRatio >= -Constants.DotProductSimilarityEpsilon)
                    continue;

                // Dynamic-body skip: a non-SimulateDynamicBody character does not probe-ground on a dynamic body
                // (it would let the character stand on a body rolling at it). The dynamic flag is sourced from the
                // main-thread StoredDynamicBodyData2D snapshot (D5) — Burst-safe.
                if (ignoreDynamicBodies && IsHitDynamic(ref baseContext, hit.entity))
                    continue;

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

        // Update_PreventGroundingFromFutureSlopeChange (the C4b future-slope feature) chains between Grounding and
        // MovementAndDecollisions — its call site is in KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D; the method
        // itself is in the C4b block below.

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
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            float characterRotation,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            ref float2 characterPosition
        )
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
                in stepAndSlopeHandling,
                in bodyTransformLookup,
                ref characterPosition,
                originalVelocityDirectionBeforeMove,
                characterHitsBuffer,
                velocityProjectionHitsBuffer,
                out bool moveConfirmedThereWereNoOverlaps
            );

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
                    characterHitsBuffer
                );
            }

            // Hit dynamics — ProcessCharacterHitDynamics (REF/KinematicCharacterUtilities.cs:3166). For every recorded
            // character hit on a body ≠ parent it solves a collision impulse, applies the self-impulse to
            // RelativeVelocity, and emits a deferred impulse on the other body. The other body's velocity/mass come
            // from StoredKinematicCharacterData2D for a character (Burst-clean) or the main-thread
            // StoredDynamicBodyData2D snapshot for a regular dynamic body (the D5 read resolution — no live-handle
            // touch in the Burst job).
            if (characterHitsBuffer.Length > 0)
            {
                ProcessCharacterHitDynamics(
                    in processor,
                    ref context,
                    ref baseContext,
                    ref characterBody,
                    in characterProperties,
                    characterPosition,
                    characterHitsBuffer,
                    deferredImpulsesBuffer
                );
            }
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
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            ref float2 characterPosition,
            float2 originalVelocityDirection,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            out bool confirmedNoOverlapsOnLastMoveIteration
        )
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
                    characterBody.GroundingUp
                );
            }

            float remainingMovementLength = length(characterBody.RelativeVelocity) * baseContext.Time.DeltaTime;
            float2 remainingMovementDirection = normalizesafe(characterBody.RelativeVelocity);

            // Initial-overlap velocity pre-pass — ProjectVelocityOnInitialOverlaps (REF/KinematicCharacterUtilities.cs:2192). The 3D
            // version used CalculateDistance (no substrate 1:1); this reuses OverlapProxy + the D2 cast-back to seed
            // projection hits from the character's initial overlaps before the move iterations, mitigating tunneling
            // when a rotation changes the detected collisions.
            if (characterProperties.ProjectVelocityOnInitialOverlaps)
            {
                ProjectVelocityOnInitialOverlaps(
                    in processor,
                    ref context,
                    ref baseContext,
                    characterEntity,
                    ref characterBody,
                    in characterProperties,
                    in colliderProxy,
                    characterRotation,
                    characterPosition,
                    in bodyTransformLookup,
                    originalVelocityDirection,
                    velocityProjectionHitsBuffer
                );

                remainingMovementLength = length(characterBody.RelativeVelocity) * baseContext.Time.DeltaTime;
                remainingMovementDirection = normalizesafe(characterBody.RelativeVelocity);
            }

            if (characterProperties.DetectMovementCollisions)
            {
                int movementCastIterationsMade = 0;
                while (
                    movementCastIterationsMade < characterProperties.MaxContinuousCollisionsIterations
                    && remainingMovementLength > 0f
                )
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
                        castLength
                    );

                    bool foundMovementHit = FilterColliderCastHitsForMove(
                        in processor,
                        ref context,
                        ref baseContext,
                        ref characterEntity,
                        castDirection,
                        Entity.Null,
                        characterProperties.ShouldIgnoreDynamicBodies(),
                        out PhysicsQueryHit2D closestHit,
                        out bool foundAnyOverlaps
                    );

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
                                (int)GroundingEvaluationType2D.MovementHit
                            );
                        }

                        KinematicCharacterHit2D currentCharacterHit = CreateCharacterHit(
                            in movementHit,
                            characterBody.IsGrounded,
                            characterBody.RelativeVelocity,
                            isGroundedOnMovementHit
                        );

                        OnMovementHit(
                            in processor,
                            ref context,
                            ref baseContext,
                            ref characterEntity,
                            ref characterBody,
                            in characterProperties,
                            in colliderProxy,
                            in stepAndSlopeHandling,
                            characterRotation,
                            ref characterPosition,
                            velocityProjectionHitsBuffer,
                            ref currentCharacterHit,
                            ref remainingMovementDirection,
                            ref remainingMovementLength,
                            originalVelocityDirection,
                            movementHitDistance
                        );

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
            out bool foundAnyOverlaps
        )
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

                if (
                    hit.entity == ignoredEntity
                    || hit.entity == characterEntity
                    || hit.entity == Entity.Null
                    || IsSensorHit(in hit)
                )
                    continue;

                if (!processor.CanCollideWithHit(ref context, ref baseContext, new BasicHit2D(hit)))
                    continue;

                // A zero-fraction hit means the proxy already overlaps this body — remember it for the
                // depenetration gate (recorded regardless of dynamic-body status, as the 3D filter does).
                if (hit.fraction <= 0f)
                {
                    foundAnyOverlaps = true;
                }

                // Dynamic-body skip: a non-SimulateDynamicBody character does not treat a dynamic body as a movement
                // obstruction (it pushes through and the dynamics path handles the push). The dynamic flag comes from
                // the main-thread StoredDynamicBodyData2D snapshot (D5).
                if (ignoreDynamicBodies && IsHitDynamic(ref baseContext, hit.entity))
                    continue;

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
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            float characterRotation,
            ref float2 characterPosition,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float2 originalVelocityDirection,
            float movementHitDistance
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            bool hasSteppedUp = false;

            // Step-up. When step handling is on, the hit is non-grounded, and the character is moving roughly
            // horizontally (its velocity is not too far below the grounding-up plane), try to lift it over a step ≤
            // MaxStepHeight. CheckForSteppingUpHit sets hasSteppedUp before the projection hit is recorded (the 3D
            // order: step-up correction first, THEN add the projection hit — REF :1410-1435).
            if (
                stepAndSlopeHandling.StepHandling
                && !hit.IsGroundedOnHit
                && dot(normalizesafe(characterBody.RelativeVelocity), characterBody.GroundingUp)
                    > Constants.MinVelocityDotRatioWithGroundingUpForSteppingUpHits
            )
            {
                CheckForSteppingUpHit(
                    in processor,
                    ref context,
                    ref baseContext,
                    characterEntity,
                    ref characterBody,
                    in characterProperties,
                    in colliderProxy,
                    characterRotation,
                    ref characterPosition,
                    ref hit,
                    ref remainingMovementDirection,
                    ref remainingMovementLength,
                    movementHitDistance,
                    stepAndSlopeHandling.StepHandling,
                    stepAndSlopeHandling.MaxStepHeight,
                    stepAndSlopeHandling.CharacterWidthForStepGroundingCheck,
                    out hasSteppedUp
                );
            }

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
                    originalVelocityDirection
                );

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
                    MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, groundNormal, groundingUp)
                );
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
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer
        )
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
                    characterRotation
                );

                // The overlap list IS the context scratch list; reconstructing an overlap re-uses that same list
                // for its cast-back, so snapshot the overlapping entities first (a small fixed-cap local set).
                if (overlapCount <= 0)
                {
                    break;
                }

                // (2) Reconstruct each overlap, classify dynamic vs non-dynamic, and track the most-penetrating
                // overall / dynamic / non-dynamic — the 2D analogue of FilterDistanceHitsForSolveOverlaps (REF :3038),
                // sourcing the dynamic flag from the main-thread StoredDynamicBodyData2D snapshot (D5).
                BasicHit2D mostPenetratingHit = default;
                float mostPenetratingDepth = 0f;
                bool mostPenetratingIsDynamic = false;
                BasicHit2D mostPenetratingNonDynamicHit = default;
                float mostPenetratingNonDynamicDepth = 0f;
                bool foundNonDynamicHit = false;

                // Dynamic overlaps remembered for the kinematic-mode push-back: the reconstructed hit and its depth
                // in parallel lists (a dynamic body is pushed by normal·depth, and recorded as a character hit so
                // ProcessCharacterHitDynamics solves the velocity exchange).
                var dynamicOverlapHits = new NativeList<BasicHit2D>(overlapCount, Allocator.Temp);
                var dynamicOverlapDepths = new NativeList<float>(overlapCount, Allocator.Temp);

                // Copy overlapping entities out before the cast-back overwrites the scratch list. Sensor
                // (isTrigger) overlaps are skipped here: a sensor is non-solid to the controller, so it never
                // contributes a depenetration push — the character passes through it (the trigger-event channel
                // still reports the visit). See IsSensorHit.
                var overlappingEntities = new NativeList<Entity>(overlapCount, Allocator.Temp);
                for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
                {
                    PhysicsQueryHit2D overlapHit = baseContext.TmpQueryHits[i];
                    if (
                        overlapHit.entity != characterEntity
                        && overlapHit.entity != Entity.Null
                        && !IsSensorHit(in overlapHit)
                    )
                    {
                        overlappingEntities.Add(overlapHit.entity);
                    }
                }

                for (int i = 0; i < overlappingEntities.Length; i++)
                {
                    Entity overlapEntity = overlappingEntities[i];

                    if (
                        ReconstructOverlap(
                            ref baseContext,
                            in colliderProxy,
                            characterPosition,
                            characterRotation,
                            overlapEntity,
                            in bodyTransformLookup,
                            out float2 overlapNormal,
                            out float overlapDepth
                        )
                    )
                    {
                        BasicHit2D basicOverlapHit = new BasicHit2D(overlapEntity, characterPosition, overlapNormal);

                        if (!processor.CanCollideWithHit(ref context, ref baseContext, in basicOverlapHit))
                            continue;

                        bool hitIsDynamic =
                            baseContext.DynamicBodyDataLookup.HasComponent(overlapEntity)
                            && baseContext.DynamicBodyDataLookup[overlapEntity].IsDynamic;

                        if (overlapDepth > mostPenetratingDepth)
                        {
                            mostPenetratingDepth = overlapDepth;
                            mostPenetratingHit = basicOverlapHit;
                            mostPenetratingIsDynamic = hitIsDynamic;
                        }

                        if (hitIsDynamic)
                        {
                            dynamicOverlapHits.Add(basicOverlapHit);
                            dynamicOverlapDepths.Add(overlapDepth);
                        }
                        else if (overlapDepth > mostPenetratingNonDynamicDepth)
                        {
                            mostPenetratingNonDynamicDepth = overlapDepth;
                            mostPenetratingNonDynamicHit = basicOverlapHit;
                            foundNonDynamicHit = true;
                        }
                    }
                }

                overlappingEntities.Dispose();

                bool foundHitForDecollision = false;

                if (characterProperties.SimulateDynamicBody)
                {
                    // Dynamic mode: decollide from the most-penetrating overall hit (dynamic or not), pushing a
                    // dynamic body via the obstruction-checked DecollideFromHit branch.
                    if (mostPenetratingDepth > 0f && mostPenetratingHit.Entity != Entity.Null)
                    {
                        bool isGroundedOnHit = false;
                        if (characterProperties.EvaluateGrounding)
                        {
                            isGroundedOnHit = processor.IsGroundedOnHit(
                                ref context,
                                ref baseContext,
                                in mostPenetratingHit,
                                (int)GroundingEvaluationType2D.OverlapDecollision
                            );
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
                            isGroundedOnHit,
                            characterProperties.SimulateDynamicBody,
                            mostPenetratingIsDynamic
                        );
                        foundHitForDecollision = true;
                    }
                }
                else
                {
                    // Kinematic mode: decollide only from the closest non-dynamic hit; on the last iteration, record
                    // every dynamic overlap as a character hit (so ProcessCharacterHitDynamics solves the velocity
                    // push) and emit its displacement impulse (recorded by the deferred system — C3 note).
                    bool isLastIteration =
                        !foundNonDynamicHit
                        || decollisionIterationsMade >= characterProperties.MaxOverlapDecollisionIterations;

                    if (isLastIteration)
                    {
                        for (int i = 0; i < dynamicOverlapHits.Length; i++)
                        {
                            BasicHit2D dyn = dynamicOverlapHits[i];
                            float dynDepth = dynamicOverlapDepths[i];

                            characterHitsBuffer.Add(
                                CreateCharacterHit(
                                    in dyn,
                                    characterBody.IsGrounded,
                                    characterBody.RelativeVelocity,
                                    false
                                )
                            );

                            deferredImpulsesBuffer.Add(
                                new KinematicCharacterDeferredImpulse2D
                                {
                                    OnEntity = dyn.Entity,
                                    Displacement = dyn.Normal * dynDepth,
                                }
                            );
                        }
                    }

                    if (foundNonDynamicHit)
                    {
                        bool isGroundedOnHit = false;
                        if (characterProperties.EvaluateGrounding)
                        {
                            isGroundedOnHit = processor.IsGroundedOnHit(
                                ref context,
                                ref baseContext,
                                in mostPenetratingNonDynamicHit,
                                (int)GroundingEvaluationType2D.OverlapDecollision
                            );
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
                            in mostPenetratingNonDynamicHit,
                            mostPenetratingNonDynamicDepth,
                            originalVelocityDirection,
                            deferredImpulsesBuffer,
                            velocityProjectionHitsBuffer,
                            characterHitsBuffer,
                            isGroundedOnHit,
                            characterProperties.SimulateDynamicBody,
                            false
                        );
                        foundHitForDecollision = true;
                    }
                }

                dynamicOverlapHits.Dispose();
                dynamicOverlapDepths.Dispose();

                if (!foundHitForDecollision)
                {
                    break;
                }
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
            out float overlapDepth
        )
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

            CastProxy(ref baseContext, in colliderProxy, castOrigin, characterRotation, castDirection, castLength);

            // Find the first (nearest) hit on the target body.
            for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
            {
                PhysicsQueryHit2D hit = baseContext.TmpQueryHits[i];
                if (hit.entity != overlapEntity)
                    continue;

                overlapNormal = normalizesafe(hit.normal, dirToCharacter);

                // Distance from the cast origin to the contact along the cast. CastProxy sweeps the REAL proxy
                // shape (circle / box / capsule), so the returned fraction is where the proxy's swept SURFACE first
                // touches the body — i.e. the distance the proxy CENTER travels from the origin until the proxy is
                // just touching. The proxy is therefore just-touching when the center has travelled exactly
                // distanceToContact; the character's actual center sits `clearance` from the origin along the same
                // axis, so the penetration depth is simply how far the center is PAST the just-touching pose:
                //
                //   overlapDepth = clearance - distanceToContact
                //
                // The earlier form subtracted `boundingRadius` (overlapDepth = clearance - (distanceToContact -
                // boundingRadius)), which DOUBLE-COUNTS the proxy extent the swept cast already accounts for: for a
                // grazing wall contact the swept capsule cast returns distanceToContact ≈ clearance (barely
                // touching), so the correct depth is ≈0, but the -boundingRadius term fabricates ≈boundingRadius
                // (~1.0 for a 2-tall capsule) of phantom penetration. That phantom depth, decollided along the wall
                // normal, shoved the grounded capsule ~0.86 u BACKWARD out of a wall it was only grazing — the
                // user-reported "pushed away" / "correct Y, incorrect X" snap-back (it walked off a low step into the
                // narrow gap before a high step, grazed the high face, and the fabricated depth pushed it back). The
                // bug was masked for a CIRCLE proxy (boundingRadius == the true radius, so the cast already reaches
                // the body at distanceToContact and the surplus is small) but is gross for a CAPSULE/BOX, whose
                // bounding radius far exceeds the contacting extent against a flat face offset from the center line.
                float distanceToContact = hit.fraction * castLength;
                float penetration = max(0f, clearance - distanceToContact);

                // Project the along-cast-axis penetration onto the TRUE contact normal. The penetration above is
                // measured along dirToCharacter (the cast axis); the minimum-translation distance to separate is its
                // component along the contact normal, so scale by |dot(dirToCharacter, normal)|. For an aligned
                // contact (a normal wall / floor where the body→character axis points along the normal, dot≈1) this
                // is a no-op; for a skewed axis it correctly reduces the push to the normal component. (This is the
                // c719d90 projection, kept — but it is now a correctness refinement on a CORRECT penetration, no
                // longer a band-aid scaling down a fabricated one; the -boundingRadius double-count was the inflation.)
                penetration *= abs(dot(dirToCharacter, overlapNormal));

                // Separate to JUST-clear plus the standard CollisionOffset margin (the same small clearance
                // Update_Grounding's snap keeps above any surface). A flush contact (penetration ≈ 0 — a character
                // resting exactly on a floor, or grazing a wall) recovers a depth of ≈CollisionOffset, so the
                // decollision leaves the proxy a hair off the surface rather than dead flush. Without this margin a
                // flush-resting character sits exactly on the surface and the next tick's short down-probe re-grounds
                // only intermittently (it flickers grounded/ungrounded tick-to-tick), which on a moving platform
                // clears the auto-parent every other tick and the rider is never carried. The margin is negligible
                // for the graze case (≈0.01 push, no fling) and exactly restores the resting clearance grounding
                // expects. The OLD -boundingRadius form accidentally supplied a (gross, ~radius-sized) clearance,
                // which masked this; the correct penetration needs the explicit, small, principled margin instead.
                overlapDepth = penetration + Constants.CollisionOffset;
                return true;
            }

            return false;
        }

        private static void RecalculateDecollisionVector(
            ref float2 decollisionVector,
            float2 originalHitNormal,
            float2 newDecollisionDirection,
            float decollisionDistance
        )
        {
            float overlapDistance = max(decollisionDistance, 0f);
            if (overlapDistance > 0f)
            {
                decollisionVector = MathUtilities2D.ReverseProjectOnVector(
                    originalHitNormal * overlapDistance,
                    newDecollisionDirection,
                    overlapDistance * Constants.DefaultReverseProjectionMaxLengthRatio
                );
            }
        }

        /// <summary>
        /// Moves the character out of a single overlap by <paramref name="decollisionDistance"/> along the hit
        /// normal (reoriented to grounding-up when grounded on the hit, or onto the ground line when grounded and
        /// the hit is non-grounded), records the projection hit, projects velocity, and records a character hit. 2D
        /// port of <c>DecollideFromHit</c> (REF/KinematicCharacterUtilities.cs:2907).
        ///
        /// <para><b>The dynamic-body branch (C4b).</b> When the character simulates as a dynamic body AND the hit
        /// body is dynamic, the decollision is cast along the decollision direction first: if a THIRD body obstructs
        /// the path, the character moves only as far as the obstruction and a displacement impulse pushes the
        /// dynamic body the rest of the way (REF :2952-2984). Otherwise it fully decollides. (The 2D substrate's
        /// deferred-impulse system records but does not apply a regular body's displacement — no relative-move
        /// command exists, C3 note — so the displacement field is recorded for forward-compat while the dynamic
        /// body's velocity push rides the impulse exchange in <see cref="ProcessCharacterHitDynamics{T,C}"/> via the
        /// recorded character hit.)</para>
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
            bool isGroundedOnHit,
            bool characterSimulateDynamic = false,
            bool hitIsDynamic = false
        )
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
                    RecalculateDecollisionVector(
                        ref decollisionVector,
                        hit.Normal,
                        decollisionDirection,
                        decollisionDistance
                    );
                }
            }
            // If grounded and the hit is non-grounded (a wall while standing), decollide along the ground line so
            // pushing out of the wall does not lift the character off the ground. (Not for a dynamic hit — we push
            // that body rather than slide along it.)
            else if (characterBody.IsGrounded && !hitIsDynamic)
            {
                decollisionDirection = normalizesafe(
                    MathUtilities2D.ProjectOnPlane(decollisionDirection, characterBody.GroundHit.Normal)
                );
                RecalculateDecollisionVector(
                    ref decollisionVector,
                    hit.Normal,
                    decollisionDirection,
                    decollisionDistance
                );
            }

            // Dynamic-body decollision (C4b): before decolliding from a dynamic body, check whether the path is
            // obstructed by a THIRD (non-dynamic) body. If so, move only as far as the obstruction and push the
            // dynamic body the rest of the way with a displacement impulse.
            bool decollidedAgainstObstruction = false;
            if (characterSimulateDynamic && hitIsDynamic && decollisionDistance > 0f)
            {
                if (
                    CastProxyClosestNonCharacter(
                        ref baseContext,
                        in colliderProxy,
                        characterEntity,
                        characterPosition,
                        characterRotation,
                        decollisionDirection,
                        decollisionDistance,
                        out PhysicsQueryHit2D obstructionHit,
                        out float obstructionDistance
                    )
                    && obstructionHit.entity != hit.Entity
                )
                {
                    // Move based on how far the obstruction was, and displace the dynamic body the remainder.
                    characterPosition += decollisionDirection * obstructionDistance;
                    deferredImpulsesBuffer.Add(
                        new KinematicCharacterDeferredImpulse2D
                        {
                            OnEntity = hit.Entity,
                            Displacement = -hit.Normal * (decollisionDistance - obstructionDistance),
                        }
                    );
                    decollidedAgainstObstruction = true;
                }
            }

            if (!decollidedAgainstObstruction)
            {
                characterPosition += decollisionVector;
            }

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
                    originalVelocityDirection
                );
            }

            KinematicCharacterHit2D overlapCharacterHit = CreateCharacterHit(
                in hit,
                characterBody.IsGrounded,
                characterRelativeVelocityBeforeProjection,
                isGroundedOnHit
            );
            overlapCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
            characterHitsBuffer.Add(overlapCharacterHit);
        }

        // =====================================================================================================
        // Default processor callbacks (REF/KinematicCharacterUtilities.cs:1131,1185,1202)
        // =====================================================================================================

        /// <summary>
        /// Slope-only grounding test for a hit: the slope-angle check (<see cref="IsGroundedOnSlopeNormal"/>) plus
        /// the going-away-from-ground velocity guard (<see cref="ShouldPreventGroundingBasedOnVelocity"/>). 2D port
        /// of <c>Default_IsGroundedOnHit</c> (REF/KinematicCharacterUtilities.cs:1131) WITHOUT the step-grounding
        /// branch — a processor with step handling enabled calls the step-aware overload (which takes the step
        /// params); this overload is the step-disabled default.
        /// </summary>
        public static bool Default_IsGroundedOnHit(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in BasicHit2D hit,
            int groundingEvaluationType
        )
        {
            if (
                ShouldPreventGroundingBasedOnVelocity(
                    ref baseContext,
                    in hit,
                    characterBody.WasGroundedBeforeCharacterUpdate,
                    characterBody.RelativeVelocity
                )
            )
            {
                return false;
            }

            // Step-grounding (a round/box base grounding on a step edge that is not a grounded slope) lives in the
            // step-aware overload of Default_IsGroundedOnHit, which takes the BasicStepAndSlopeHandlingParameters2D;
            // this overload is slope-only (used when step handling is off).
            return IsGroundedOnSlopeNormal(
                characterProperties.MaxGroundedSlopeDotProduct,
                hit.Normal,
                characterBody.GroundingUp
            );
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
            float2 groundingUp
        )
        {
            return dot(groundingUp, slopeSurfaceNormal) > maxGroundedSlopeDotProduct;
        }

        /// <summary>
        /// Prevents grounding when the character is airborne and moving away from the ground normal (so it does not
        /// snap to a wall it is launching off). 2D port of <c>ShouldPreventGroundingBasedOnVelocity</c>
        /// (REF/KinematicCharacterUtilities.cs:3618).
        ///
        /// <para><b>Dynamic-ground-velocity comparison (C4b).</b> For a DYNAMIC ground body, grounding is prevented
        /// only when the character's velocity along the hit normal exceeds the ground's velocity along the same
        /// normal (the character is escaping the ground's own motion) — exactly the 3D guard (REF :3630-3648). The
        /// ground's velocity comes from the main-thread <see cref="StoredDynamicBodyData2D"/> snapshot (the D5 read
        /// resolution — never the live handle in a Burst job). A static / un-snapshotted ground is treated as
        /// stationary, so the airborne-moving-away case always prevents grounding for static geometry.</para>
        /// </summary>
        public static bool ShouldPreventGroundingBasedOnVelocity(
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit,
            bool wasGroundedBeforeCharacterUpdate,
            float2 relativeVelocity
        )
        {
            if (
                !wasGroundedBeforeCharacterUpdate
                && dot(relativeVelocity, hit.Normal) > Constants.DotProductSimilarityEpsilon
                && lengthsq(relativeVelocity) > Constants.MinVelocityLengthSqForGroundingIgnoreCheck
            )
            {
                if (
                    hit.Entity != Entity.Null
                    && baseContext.DynamicBodyDataLookup.HasComponent(hit.Entity)
                    && baseContext.DynamicBodyDataLookup[hit.Entity].IsDynamic
                )
                {
                    StoredDynamicBodyData2D groundData = baseContext.DynamicBodyDataLookup[hit.Entity];
                    float2 groundVelocityAtPoint = PhysicsUtilities2D.GetPointVelocity(
                        groundData.LinearVelocity,
                        groundData.AngularVelocity,
                        hit.Position,
                        hit.Position
                    );

                    float characterVelocityOnNormal = dot(relativeVelocity, hit.Normal);
                    float groundVelocityOnNormal = dot(groundVelocityAtPoint, hit.Normal);

                    // Only prevent grounding if the character is escaping the ground's own velocity.
                    return characterVelocityOnNormal > groundVelocityOnNormal;
                }

                // Static / un-snapshotted ground has no velocity: prevent grounding.
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
            in KinematicCharacterBody2D characterBody
        )
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
                    constrainToGroundPlane
                );
                velocityDirection = normalizesafe(velocity);

                // The original velocity direction acts as a constraint line too (index -1), preventing velocity from
                // reversing back the way it came. Its "normal" is the original direction itself (projected onto the
                // ground plane when grounded), so a re-entry test against it catches a U-turn corner.
                KinematicVelocityProjectionHit2D originalVelocityHit = default;
                originalVelocityHit.Normal = characterIsGrounded
                    ? normalizesafe(
                        MathUtilities2D.ProjectOnPlane(originalVelocityDirection, characterBody.GroundingUp)
                    )
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
            bool constrainToGroundPlane
        )
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

        // =====================================================================================================
        // C4b — advanced solve features layered on the C4a core
        // =====================================================================================================

        /// <summary>
        /// Casts a ray that skips the character itself and returns the closest non-character hit. A thin filter over
        /// <see cref="PhysicsQueries2D.RaycastClosest"/> — the substrate's <c>RaycastClosest</c> already returns the
        /// single nearest hit, but it can be the character's own body, so this re-casts the full list and walks for
        /// the nearest hit whose entity is neither the character nor null. Used by the step- and slope-handling
        /// raycasts. Note <see cref="PhysicsQueries2D.Raycast"/> writes the context scratch list, so a caller that
        /// relies on the scratch list across this call must re-cast afterward.
        /// </summary>
        static bool RaycastClosestNonCharacter(
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            float2 origin,
            float2 direction,
            float distance,
            out PhysicsQueryHit2D closestHit
        )
        {
            closestHit = default;
            // INNER query: write into the inner scratch list so it never clobbers the outer TmpQueryHits the caller
            // (GroundDetection's tolerance walk / IsGroundedOnSteps reached from a move/grounding filter loop) is
            // still iterating. See KinematicCharacterUpdateContext2D.TmpInnerQueryHits.
            var hits = baseContext.TmpInnerQueryHits;
            PhysicsQueries2D.Raycast(
                baseContext.PhysicsWorld,
                origin,
                direction,
                distance,
                Constants.CharacterHitLayerMask,
                hits
            );

            for (int i = 0; i < hits.Length; i++)
            {
                PhysicsQueryHit2D hit = hits[i];
                if (hit.entity == characterEntity || hit.entity == Entity.Null || IsSensorHit(in hit))
                    continue;

                // The list is nearest-first sorted, so the first non-character hit is the closest one.
                closestHit = hit;
                return true;
            }

            return false;
        }

        /// <summary>
        /// Casts the cast proxy and returns the closest obstructing non-character hit (the proxy analogue of
        /// <see cref="RaycastClosestNonCharacter"/>). Used by the step-up forward/down proxy casts. Reuses
        /// <see cref="FilterColliderCastHitsForMove{T,C}"/>'s accept/reject logic by walking the just-cast scratch
        /// list for the nearest non-character hit (whether or not its normal opposes the cast — the step casts need
        /// the nearest obstruction regardless of facing). Returns the hit and its un-normalized travel distance.
        /// </summary>
        static bool CastProxyClosestNonCharacter(
            ref KinematicCharacterUpdateContext2D baseContext,
            in KinematicCharacterColliderProxy2D colliderProxy,
            Entity characterEntity,
            float2 origin,
            float rotationRadians,
            float2 direction,
            float distance,
            out PhysicsQueryHit2D closestHit,
            out float hitDistance
        )
        {
            closestHit = default;
            hitDistance = distance;

            // INNER query: cast into the inner scratch list (step-up forward/down casts and the obstruction check in
            // DecollideFromHit run while an outer move/overlap loop is mid-iteration over TmpQueryHits).
            var hits = baseContext.TmpInnerQueryHits;
            CastProxyInto(ref baseContext, in colliderProxy, origin, rotationRadians, direction, distance, hits);

            for (int i = 0; i < hits.Length; i++)
            {
                PhysicsQueryHit2D hit = hits[i];
                if (hit.entity == characterEntity || hit.entity == Entity.Null || IsSensorHit(in hit))
                    continue;

                // Nearest-first sorted, so the first non-character hit is the closest.
                closestHit = hit;
                hitDistance = hit.fraction * distance;
                return true;
            }

            return false;
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 2 — Parent movement (REF/KinematicCharacterUtilities.cs:544) + moving-platform detection / momentum
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Carries the character rigidly with a <see cref="TrackedTransform2D"/> parent's pose delta over the step.
        /// 2D port of <c>Update_ParentMovement</c> (REF/KinematicCharacterUtilities.cs:544). The 3D
        /// rotation-up-correction (CreateRotationWithUpPriority / SetRotationAroundPoint) is dropped: a 2D rotation
        /// is a single z-angle, the character's grounding-up is its own up, and the parent only contributes its
        /// own z-rotation delta — there is no 3-DOF up axis to re-prioritise. The optional obstruction-cast of the
        /// parent displacement (REF :599) is ported with the proxy cast.
        /// </summary>
        public static void Update_ParentMovement<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            ref float2 characterPosition
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            // Reset parent if it no longer exists.
            if (
                characterBody.ParentEntity != Entity.Null
                && !baseContext.TrackedTransformLookup.HasComponent(characterBody.ParentEntity)
            )
            {
                characterBody.ParentEntity = Entity.Null;
            }

            // Reset parent velocity only when there is no previous parent (so a just-unparented character preserves
            // last update's parent velocity for the momentum carry-over in Update_ParentMomentum).
            if (characterBody.PreviousParentEntity == Entity.Null)
            {
                characterBody.ParentVelocity = new float2(0f, 0f);
            }

            if (characterBody.ParentEntity != Entity.Null)
            {
                TrackedTransform2D parentTracked = baseContext.TrackedTransformLookup[characterBody.ParentEntity];

                // Position: re-express the character's position under the previous parent pose, then under the
                // current parent pose. RotationFromParent is the parent's z-rotation delta over the step.
                float2 previousLocalPosition = parentTracked.PreviousFixedRateTransform.InverseTransformPoint(
                    characterPosition
                );
                float2 targetWorldPosition = parentTracked.CurrentFixedRateTransform.TransformPoint(
                    previousLocalPosition
                );

                float2 displacementFromParentMovement = targetWorldPosition - characterPosition;
                characterBody.ParentVelocity = displacementFromParentMovement / baseContext.Time.DeltaTime;
                characterBody.RotationFromParent =
                    parentTracked.CurrentFixedRateTransform.Rotation
                    - parentTracked.PreviousFixedRateTransform.Rotation;

                // Optionally cast the displacement first, to avoid pushing the character into a wall.
                if (
                    characterProperties.DetectMovementCollisions
                    && characterProperties.DetectObstructionsForParentBodyMovement
                    && lengthsq(displacementFromParentMovement) > EPSILON
                )
                {
                    float castLength = length(displacementFromParentMovement);
                    float2 castDirection = displacementFromParentMovement / castLength;

                    CastProxy(
                        ref baseContext,
                        in colliderProxy,
                        characterPosition,
                        characterRotation,
                        castDirection,
                        castLength
                    );

                    if (
                        FilterColliderCastHitsForMove(
                            in processor,
                            ref context,
                            ref baseContext,
                            ref characterEntity,
                            castDirection,
                            characterBody.ParentEntity,
                            characterProperties.ShouldIgnoreDynamicBodies(),
                            out PhysicsQueryHit2D closestHit,
                            out bool _
                        )
                    )
                    {
                        characterPosition += castDirection * (closestHit.fraction * castLength);
                    }
                    else
                    {
                        characterPosition += displacementFromParentMovement;
                    }
                }
                else
                {
                    characterPosition += displacementFromParentMovement;
                }
            }
        }

        /// <summary>
        /// Sets or clears the character's parent body. 2D port of <c>SetOrUpdateParentBody</c>
        /// (REF/KinematicCharacterUtilities.cs:1541) — a parent is accepted only when it carries a
        /// <see cref="TrackedTransform2D"/>.
        /// </summary>
        public static void SetOrUpdateParentBody(
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            Entity parentEntity,
            float2 anchorPointLocalParentSpace
        )
        {
            if (parentEntity != Entity.Null && baseContext.TrackedTransformLookup.HasComponent(parentEntity))
            {
                characterBody.ParentEntity = parentEntity;
                characterBody.ParentLocalAnchorPoint = anchorPointLocalParentSpace;
            }
            else
            {
                characterBody.ParentEntity = Entity.Null;
                characterBody.ParentLocalAnchorPoint = new float2(0f, 0f);
            }
        }

        /// <summary>
        /// Auto-parents the character to its ground when the ground is a <see cref="TrackedTransform2D"/> (a moving
        /// platform), else clears the parent. 2D port of <c>Update_MovingPlatformDetection</c>
        /// (REF/KinematicCharacterUtilities.cs:952). The 3D anchor point is the ground-hit point expressed in the
        /// platform's local space; in 2D the platform pose is read from the tracked transform (the substrate has no
        /// per-body world-transform array), so the anchor is computed via the tracked transform's inverse.
        /// </summary>
        public static void Update_MovingPlatformDetection(
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody
        )
        {
            if (
                characterBody.IsGrounded
                && baseContext.TrackedTransformLookup.HasComponent(characterBody.GroundHit.Entity)
            )
            {
                TrackedTransform2D groundTracked = baseContext.TrackedTransformLookup[characterBody.GroundHit.Entity];
                float2 anchorLocal = groundTracked.CurrentFixedRateTransform.InverseTransformPoint(
                    characterBody.GroundHit.Position
                );
                SetOrUpdateParentBody(ref baseContext, ref characterBody, characterBody.GroundHit.Entity, anchorLocal);
            }
            else
            {
                SetOrUpdateParentBody(ref baseContext, ref characterBody, Entity.Null, new float2(0f, 0f));
            }
        }

        /// <summary>
        /// Preserves / compensates velocity momentum across a parent change. 2D port of <c>Update_ParentMomentum</c>
        /// (REF/KinematicCharacterUtilities.cs:972). When the character leaves a parent it keeps the parent's
        /// velocity as momentum; when it joins one it subtracts the new parent's point velocity so its relative
        /// velocity is parent-relative, reorienting onto the ground if grounded.
        /// </summary>
        public static void Update_ParentMomentum(
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            float2 characterPosition
        )
        {
            if (
                characterBody.ParentEntity != Entity.Null
                && !baseContext.TrackedTransformLookup.HasComponent(characterBody.ParentEntity)
            )
            {
                characterBody.ParentEntity = Entity.Null;
            }

            if (characterBody.ParentEntity != characterBody.PreviousParentEntity)
            {
                // Preserve momentum from the previous parent on a parent change.
                if (characterBody.PreviousParentEntity != Entity.Null)
                {
                    characterBody.RelativeVelocity += characterBody.ParentVelocity;
                    characterBody.ParentVelocity = new float2(0f, 0f);
                }

                // Compensate for the new parent body.
                if (characterBody.ParentEntity != Entity.Null)
                {
                    TrackedTransform2D parentTracked = baseContext.TrackedTransformLookup[characterBody.ParentEntity];
                    characterBody.ParentVelocity = parentTracked.CalculatePointVelocity(
                        characterPosition,
                        baseContext.Time.DeltaTime
                    );
                    characterBody.RelativeVelocity -= characterBody.ParentVelocity;

                    if (characterBody.IsGrounded)
                    {
                        ProjectVelocityOnGrounding(
                            ref characterBody.RelativeVelocity,
                            characterBody.GroundHit.Normal,
                            characterBody.GroundingUp
                        );
                    }
                }
            }
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 4 — Prevent grounding from a future slope change (REF/KinematicCharacterUtilities.cs:817)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Predicts whether the next step lands the character on no-grounding or too-steep a downward slope and, if
        /// so, forces <see cref="KinematicCharacterBody2D.IsGrounded"/> false so it launches off a ledge cleanly
        /// instead of snapping. 2D port of <c>Update_PreventGroundingFromFutureSlopeChange</c>
        /// (REF/KinematicCharacterUtilities.cs:817).
        /// </summary>
        public static void Update_PreventGroundingFromFutureSlopeChange<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            float slopeDetectionVerticalOffset = 0.05f,
            float slopeDetectionDownDetectionDepth = 0.05f,
            float slopeDetectionSecondaryNoGroundingCheckDistance = 0.25f
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            if (
                characterBody.IsGrounded
                && (
                    stepAndSlopeHandling.PreventGroundingWhenMovingTowardsNoGrounding
                    || stepAndSlopeHandling.HasMaxDownwardSlopeChangeAngle
                )
            )
            {
                DetectFutureSlopeChange(
                    ref baseContext,
                    characterEntity,
                    ref characterBody,
                    in characterProperties,
                    slopeDetectionVerticalOffset,
                    slopeDetectionDownDetectionDepth,
                    baseContext.Time.DeltaTime,
                    slopeDetectionSecondaryNoGroundingCheckDistance,
                    stepAndSlopeHandling.StepHandling,
                    stepAndSlopeHandling.MaxStepHeight,
                    out bool isMovingTowardsNoGrounding,
                    out bool foundSlopeHit,
                    out float futureSlopeChangeAnglesRadians
                );

                if (
                    (stepAndSlopeHandling.PreventGroundingWhenMovingTowardsNoGrounding && isMovingTowardsNoGrounding)
                    || (
                        stepAndSlopeHandling.HasMaxDownwardSlopeChangeAngle
                        && foundSlopeHit
                        && degrees(futureSlopeChangeAnglesRadians) < -stepAndSlopeHandling.MaxDownwardSlopeChangeAngle
                    )
                )
                {
                    characterBody.IsGrounded = false;
                }
            }
        }

        /// <summary>
        /// Raycasts forward / down / secondary-down / back along the velocity to predict the next step's grounding.
        /// 2D port of <c>DetectFutureSlopeChange</c> (REF/KinematicCharacterUtilities.cs:3352) — the cast collider
        /// becomes a ray (the substrate's <see cref="PhysicsQueries2D"/> raycast), and the processor parameter is
        /// dropped because the 2D filter (skip-self) needs no processor for these scalar rays.
        /// </summary>
        static void DetectFutureSlopeChange(
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            float verticalOffset,
            float downDetectionDepth,
            float deltaTimeIntoFuture,
            float secondaryNoGroundingCheckDistance,
            bool stepHandling,
            float maxStepHeight,
            out bool isMovingTowardsNoGrounding,
            out bool foundSlopeHit,
            out float futureSlopeChangeAnglesRadians
        )
        {
            isMovingTowardsNoGrounding = false;
            foundSlopeHit = false;
            futureSlopeChangeAnglesRadians = 0f;

            if (
                !IsGroundedOnSlopeNormal(
                    characterProperties.MaxGroundedSlopeDotProduct,
                    characterBody.GroundHit.Normal,
                    characterBody.GroundingUp
                )
            )
            {
                return;
            }

            if (stepHandling)
            {
                downDetectionDepth = max(maxStepHeight, downDetectionDepth) + verticalOffset;
            }
            else
            {
                downDetectionDepth += verticalOffset;
            }

            float2 velocityDirection = normalizesafe(characterBody.RelativeVelocity);
            float2 rayStartPoint = characterBody.GroundHit.Position + (characterBody.GroundingUp * verticalOffset);
            float2 rayDirection = velocityDirection;
            float rayLength = length(characterBody.RelativeVelocity * deltaTimeIntoFuture);

            if (rayLength <= EPSILON)
            {
                return;
            }

            // Forward
            if (
                RaycastClosestNonCharacter(
                    ref baseContext,
                    characterEntity,
                    rayStartPoint,
                    rayDirection,
                    rayLength,
                    out PhysicsQueryHit2D forwardHit
                )
            )
            {
                foundSlopeHit = true;
                futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithGroundUp(
                    characterBody.GroundHit.Normal,
                    forwardHit.normal,
                    velocityDirection,
                    characterBody.GroundingUp
                );
                return;
            }

            rayStartPoint += rayDirection * rayLength;
            rayDirection = -characterBody.GroundingUp;
            rayLength = downDetectionDepth;

            // Down
            if (
                RaycastClosestNonCharacter(
                    ref baseContext,
                    characterEntity,
                    rayStartPoint,
                    rayDirection,
                    rayLength,
                    out PhysicsQueryHit2D downHit
                )
            )
            {
                foundSlopeHit = true;
                futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithGroundUp(
                    characterBody.GroundHit.Normal,
                    downHit.normal,
                    velocityDirection,
                    characterBody.GroundingUp
                );

                if (
                    !IsGroundedOnSlopeNormal(
                        characterProperties.MaxGroundedSlopeDotProduct,
                        downHit.normal,
                        characterBody.GroundingUp
                    )
                )
                {
                    isMovingTowardsNoGrounding = true;
                }
            }
            else
            {
                isMovingTowardsNoGrounding = true;
            }

            if (!isMovingTowardsNoGrounding)
            {
                return;
            }

            rayStartPoint += velocityDirection * secondaryNoGroundingCheckDistance;

            // Secondary down
            if (
                RaycastClosestNonCharacter(
                    ref baseContext,
                    characterEntity,
                    rayStartPoint,
                    rayDirection,
                    rayLength,
                    out PhysicsQueryHit2D secondDownHit
                )
            )
            {
                if (!foundSlopeHit)
                {
                    foundSlopeHit = true;
                    futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithGroundUp(
                        characterBody.GroundHit.Normal,
                        secondDownHit.normal,
                        velocityDirection,
                        characterBody.GroundingUp
                    );
                }

                if (
                    IsGroundedOnSlopeNormal(
                        characterProperties.MaxGroundedSlopeDotProduct,
                        secondDownHit.normal,
                        characterBody.GroundingUp
                    )
                )
                {
                    isMovingTowardsNoGrounding = false;
                }
            }
            else
            {
                rayStartPoint += rayDirection * rayLength;
                rayDirection = -velocityDirection;
                rayLength =
                    length(characterBody.RelativeVelocity * deltaTimeIntoFuture) + secondaryNoGroundingCheckDistance;

                // Back
                if (
                    RaycastClosestNonCharacter(
                        ref baseContext,
                        characterEntity,
                        rayStartPoint,
                        rayDirection,
                        rayLength,
                        out PhysicsQueryHit2D backHit
                    )
                )
                {
                    foundSlopeHit = true;
                    futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithGroundUp(
                        characterBody.GroundHit.Normal,
                        backHit.normal,
                        velocityDirection,
                        characterBody.GroundingUp
                    );

                    if (
                        IsGroundedOnSlopeNormal(
                            characterProperties.MaxGroundedSlopeDotProduct,
                            backHit.normal,
                            characterBody.GroundingUp
                        )
                    )
                    {
                        isMovingTowardsNoGrounding = false;
                    }
                }
            }
        }

        /// <summary>
        /// The signed slope-angle change of a hit relative to the current ground, in the character's movement
        /// direction (negative = downward). 2D port of <c>CalculateAngleOfHitWithGroundUp</c>
        /// (REF/KinematicCharacterUtilities.cs:3594): the 3D <c>velocityRight = cross(velocityDir, -groundingUp)</c>
        /// (the axis the slope is measured about) is a single z-axis in 2D, so the projection onto the plane
        /// perpendicular to it is just the planar vectors themselves — the angle is measured directly between the
        /// two normals, signed by which one leans more along the movement direction.
        /// </summary>
        public static float CalculateAngleOfHitWithGroundUp(
            float2 currentGroundUp,
            float2 hitNormal,
            float2 velocityDirection,
            float2 groundingUp
        )
        {
            // In 2D the "measure about velocityRight" reduces to measuring directly in the movement plane (which is
            // the whole 2D plane), so the projected normals are the normals themselves.
            float slopeChangeAnglesRadians = MathUtilities2D.AngleRadians(currentGroundUp, hitNormal);

            // Invert the sign if it's a downward slope change (the hit normal leans more along the movement
            // direction than the current ground normal does).
            if (dot(currentGroundUp, velocityDirection) < dot(hitNormal, velocityDirection))
            {
                slopeChangeAnglesRadians *= -1f;
            }

            return slopeChangeAnglesRadians;
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 5 (movement) — step handling: IsGroundedOnSteps + CheckForSteppingUpHit
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Whether a character with a rounded/box base can be grounded on a step edge that is not a grounded slope.
        /// 2D port of <c>IsGroundedOnSteps</c> (REF/KinematicCharacterUtilities.cs:3680): a "back step" raycast
        /// behind the hit confirms a grounded surface within step height, and a "forward step" raycast confirms the
        /// step rises within the horizontal offset — both within <see cref="BasicStepAndSlopeHandlingParameters2D.MaxStepHeight"/>.
        /// The collider casts of the 3D forward-obstruction checks become rays here (the substrate offers only
        /// circle/box casts and the step probes are point rays in the reference).
        /// </summary>
        public static bool IsGroundedOnSteps(
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            in KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in BasicHit2D hit,
            float maxStepHeight,
            float extraStepChecksDistance
        )
        {
            if (maxStepHeight <= 0f)
            {
                return false;
            }

            bool isGroundedOnBackStep = false;
            bool isGroundedOnForwardStep = false;
            float2 backCheckDirection = normalizesafe(
                MathUtilities2D.ProjectOnPlane(hit.Normal, characterBody.GroundingUp)
            );

            // Close back step hit.
            float backHitDistance = 0f;
            if (
                RaycastClosestNonCharacter(
                    ref baseContext,
                    characterEntity,
                    hit.Position + (backCheckDirection * Constants.StepGroundingDetectionHorizontalOffset),
                    -characterBody.GroundingUp,
                    maxStepHeight,
                    out PhysicsQueryHit2D backStepHit
                )
            )
            {
                backHitDistance = backStepHit.fraction * maxStepHeight;
                if (backHitDistance > 0f)
                {
                    isGroundedOnBackStep = IsGroundedOnSlopeNormal(
                        characterProperties.MaxGroundedSlopeDotProduct,
                        backStepHit.normal,
                        characterBody.GroundingUp
                    );
                }
            }

            if (
                !isGroundedOnBackStep //
                && extraStepChecksDistance > Constants.StepGroundingDetectionHorizontalOffset //
            )
            {
                if (
                    RaycastClosestNonCharacter(
                        ref baseContext,
                        characterEntity,
                        hit.Position + (backCheckDirection * extraStepChecksDistance),
                        -characterBody.GroundingUp,
                        maxStepHeight,
                        out backStepHit
                    )
                )
                {
                    backHitDistance = backStepHit.fraction * maxStepHeight;
                    if (backHitDistance > 0f)
                    {
                        isGroundedOnBackStep = IsGroundedOnSlopeNormal(
                            characterProperties.MaxGroundedSlopeDotProduct,
                            backStepHit.normal,
                            characterBody.GroundingUp
                        );
                    }
                }
            }

            if (isGroundedOnBackStep)
            {
                float forwardCheckHeight = maxStepHeight - backHitDistance;

                // F1 — forward obstruction (a short horizontal ray at step height, into the step face). Verbatim
                // 2D mirror of REF/KinematicCharacterUtilities.cs:3742-3757, including the `forwardHitDistance > 0`
                // guard the reference applies to every forward cast.
                bool forwardStepHitFound = RaycastClosestNonCharacter(
                    ref baseContext,
                    characterEntity,
                    hit.Position + (characterBody.GroundingUp * forwardCheckHeight),
                    -backCheckDirection,
                    Constants.StepGroundingDetectionHorizontalOffset,
                    out PhysicsQueryHit2D forwardStepHit
                );
                float forwardHitDistance = forwardStepHit.fraction * Constants.StepGroundingDetectionHorizontalOffset;
                if (forwardStepHitFound && forwardHitDistance > 0f)
                {
                    isGroundedOnForwardStep = IsGroundedOnSlopeNormal(
                        characterProperties.MaxGroundedSlopeDotProduct,
                        forwardStepHit.normal,
                        characterBody.GroundingUp
                    );
                }

                if (!forwardStepHitFound)
                {
                    // F2 — close forward step hit (down ray just in front of the step). REF :3759-3777.
                    forwardStepHitFound = RaycastClosestNonCharacter(
                        ref baseContext,
                        characterEntity,
                        hit.Position
                            + (characterBody.GroundingUp * forwardCheckHeight)
                            + (-backCheckDirection * Constants.StepGroundingDetectionHorizontalOffset),
                        -characterBody.GroundingUp,
                        maxStepHeight,
                        out forwardStepHit
                    );
                    forwardHitDistance = forwardStepHit.fraction * maxStepHeight;
                    if (forwardStepHitFound && forwardHitDistance > 0f)
                    {
                        isGroundedOnForwardStep = IsGroundedOnSlopeNormal(
                            characterProperties.MaxGroundedSlopeDotProduct,
                            forwardStepHit.normal,
                            characterBody.GroundingUp
                        );
                    }

                    // The extra-reach forward checks: a step slightly angled or slightly farther than the close
                    // offset (a step+slope corner) needs a longer forward reach to find the step rise. REF
                    // :3779-3819 — the `extraStepChecksDistance > StepGroundingDetectionHorizontalOffset` block,
                    // active under the stock params (ExtraStepChecksDistance 0.1 > offset 0.01). Omitting it made
                    // IsGroundedOnSteps direction-asymmetric: a corner that grounds on one approach's close
                    // geometry but needs the extra reach on the mirrored approach was judged not-grounded on that
                    // one side only.
                    if (
                        !isGroundedOnForwardStep
                        && extraStepChecksDistance > Constants.StepGroundingDetectionHorizontalOffset
                    )
                    {
                        // F3 — extra forward obstruction (horizontal, the wider reach). REF :3782-3797.
                        forwardStepHitFound = RaycastClosestNonCharacter(
                            ref baseContext,
                            characterEntity,
                            hit.Position + (characterBody.GroundingUp * forwardCheckHeight),
                            -backCheckDirection,
                            extraStepChecksDistance,
                            out forwardStepHit
                        );
                        forwardHitDistance = forwardStepHit.fraction * extraStepChecksDistance;
                        if (forwardStepHitFound && forwardHitDistance > 0f)
                        {
                            isGroundedOnForwardStep = IsGroundedOnSlopeNormal(
                                characterProperties.MaxGroundedSlopeDotProduct,
                                forwardStepHit.normal,
                                characterBody.GroundingUp
                            );
                        }

                        if (!forwardStepHitFound)
                        {
                            // F4 — extra forward step hit (down ray at the wider reach). REF :3799-3818.
                            forwardStepHitFound = RaycastClosestNonCharacter(
                                ref baseContext,
                                characterEntity,
                                hit.Position
                                    + (characterBody.GroundingUp * forwardCheckHeight)
                                    + (-backCheckDirection * extraStepChecksDistance),
                                -characterBody.GroundingUp,
                                maxStepHeight,
                                out forwardStepHit
                            );
                            forwardHitDistance = forwardStepHit.fraction * maxStepHeight;
                            if (forwardStepHitFound && forwardHitDistance > 0f)
                            {
                                isGroundedOnForwardStep = IsGroundedOnSlopeNormal(
                                    characterProperties.MaxGroundedSlopeDotProduct,
                                    forwardStepHit.normal,
                                    characterBody.GroundingUp
                                );
                            }
                        }
                    }
                }
            }

            return isGroundedOnBackStep && isGroundedOnForwardStep;
        }

        /// <summary>
        /// Tries to lift the character over a step ≤ <paramref name="maxStepHeight"/> on a non-grounded movement
        /// hit. 2D port of <c>CheckForSteppingUpHit</c> (REF/KinematicCharacterUtilities.cs:3851): an up-cast finds
        /// headroom, a forward-cast clears the step lip, a down-cast lands on the step top; if that top grounds the
        /// character, the character is moved up-and-over and <paramref name="hasSteppedUp"/> is set so the movement
        /// loop skips the slide. The 3D collider casts become proxy casts; the character-width slope refinement
        /// (REF :3994) is kept with the 2D forward-slope ray.
        /// </summary>
        public static void CheckForSteppingUpHit<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            ref float2 characterPosition,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float hitDistance,
            bool stepHandling,
            float maxStepHeight,
            float characterWidthForStepGroundingCheck,
            out bool hasSteppedUp
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            hasSteppedUp = false;

            if (
                !characterProperties.EvaluateGrounding //
                || !stepHandling //
                || hit.IsGroundedOnHit //
                || maxStepHeight <= 0f //
            )
            {
                return;
            }

            float2 upCheckDirection = characterBody.GroundingUp;
            float upCheckDistance = maxStepHeight;

            // Up cast: how much headroom is there to lift the character?
            float upStepHitDistance;
            if (
                CastProxyClosestNonCharacter(
                    ref baseContext,
                    in colliderProxy,
                    characterEntity,
                    characterPosition,
                    characterRotation,
                    upCheckDirection,
                    upCheckDistance,
                    out PhysicsQueryHit2D _,
                    out float upHitDist
                )
            )
            {
                upStepHitDistance = max(0f, upHitDist - Constants.CollisionOffset);
            }
            else
            {
                upStepHitDistance = upCheckDistance;
            }

            if (upStepHitDistance <= 0f)
            {
                return;
            }

            float2 startPositionOfForwardCheck = characterPosition + (upCheckDirection * upStepHitDistance);
            float distanceOverStep = length(
                MathUtilities2D.ProjectOnPlane(
                    remainingMovementDirection * (remainingMovementLength - hitDistance),
                    hit.Normal
                )
            );
            float2 endPositionOfForwardCheck =
                startPositionOfForwardCheck
                + (remainingMovementDirection * (remainingMovementLength + Constants.CollisionOffset));
            float minimumDistanceOverStep = Constants.CollisionOffset * 3f;
            if (distanceOverStep < minimumDistanceOverStep)
            {
                endPositionOfForwardCheck += -hit.Normal * (minimumDistanceOverStep - distanceOverStep);
            }

            float2 forwardDelta = endPositionOfForwardCheck - startPositionOfForwardCheck;
            float forwardCheckDistance = length(forwardDelta);
            if (forwardCheckDistance <= EPSILON)
            {
                return;
            }

            float2 forwardCheckDirection = forwardDelta / forwardCheckDistance;

            // Forward cast: clear the step lip.
            float forwardStepHitDistance;
            if (
                CastProxyClosestNonCharacter(
                    ref baseContext,
                    in colliderProxy,
                    characterEntity,
                    startPositionOfForwardCheck,
                    characterRotation,
                    forwardCheckDirection,
                    forwardCheckDistance,
                    out PhysicsQueryHit2D _,
                    out float fwdHitDist
                )
            )
            {
                forwardStepHitDistance = max(0f, fwdHitDist - Constants.CollisionOffset);
            }
            else
            {
                forwardStepHitDistance = forwardCheckDistance;
            }

            if (forwardStepHitDistance <= 0f)
            {
                return;
            }

            float2 startPositionOfDownCheck =
                startPositionOfForwardCheck + (forwardCheckDirection * forwardStepHitDistance);
            float2 downCheckDirection = -characterBody.GroundingUp;
            float downCheckDistance = upStepHitDistance;

            // Down cast: land on the step top.
            if (
                !CastProxyClosestNonCharacter(
                    ref baseContext,
                    in colliderProxy,
                    characterEntity,
                    startPositionOfDownCheck,
                    characterRotation,
                    downCheckDirection,
                    downCheckDistance,
                    out PhysicsQueryHit2D downStepHit,
                    out float downStepHitDistance
                )
                || downStepHitDistance <= 0f
            )
            {
                return;
            }

            BasicHit2D stepHit = new BasicHit2D(downStepHit);
            bool isGroundedOnStepHit = false;
            if (characterProperties.EvaluateGrounding)
            {
                isGroundedOnStepHit = processor.IsGroundedOnHit(
                    ref context,
                    ref baseContext,
                    in stepHit,
                    (int)GroundingEvaluationType2D.StepUpHit
                );
            }

            if (!isGroundedOnStepHit)
            {
                return;
            }

            float hitHeight = upStepHitDistance - downStepHitDistance;
            float steppedHeight = max(0f, hitHeight + Constants.CollisionOffset);

            // Slope + character-width consideration: a rounded base over an angled step top can rise more than the
            // measured hit height, so add the extra height a forward slope of the step top would impose. This is the
            // gate that REJECTS a step-up onto a surface too steep to stand on: an over-limit slope produces a large
            // tan(slopeRadians)·width extra height that pushes steppedHeight past maxStepHeight, so the step-up below
            // does not engage (a character must not "step up" a 75° ramp it cannot climb — doing so lets the grounded
            // magnitude-preserving reorient pump gravity into a lateral fling / propulsion).
            //
            // The down-probe samples the surface a short offset along the step top's UP-SLOPE tangent
            // (MathUtilities2D.StepTopUpSlopeTangent2D — the 2D reduction of the 3D's
            // -normalize(cross(cross(GroundingUp, stepHit.Normal), stepHit.Normal)), REF/KinematicCharacterUtilities.cs:3998),
            // so it reads the slope AHEAD of the step lip. The helper's doc records the two earlier mis-ports this
            // replaces; the degenerate one (a zero direction sampling straight down at the lip) over-rejected an
            // in-range step at a step+slope corner.
            if (characterWidthForStepGroundingCheck > 0f)
            {
                float2 forwardSlopeCheckDirection = MathUtilities2D.StepTopUpSlopeTangent2D(
                    stepHit.Normal,
                    characterBody.GroundingUp
                );
                if (
                    RaycastClosestNonCharacter(
                        ref baseContext,
                        characterEntity,
                        stepHit.Position
                            + (characterBody.GroundingUp * Constants.CollisionOffset)
                            + (forwardSlopeCheckDirection * Constants.CollisionOffset),
                        -characterBody.GroundingUp,
                        maxStepHeight,
                        out PhysicsQueryHit2D forwardSlopeCheckHit
                    )
                )
                {
                    float slopeRadians = MathUtilities2D.AngleRadians(
                        characterBody.GroundingUp,
                        forwardSlopeCheckHit.normal
                    );
                    float extraHeight = tan(slopeRadians) * characterWidthForStepGroundingCheck * 0.5f;
                    steppedHeight += extraHeight;
                }
            }

            if (steppedHeight < maxStepHeight)
            {
                // Step up: lift onto the step top and advance to the forward-clear position. Lift by hitHeight PLUS
                // CollisionOffset so the proxy bottom lands the same small clearance above the step top that normal
                // ground-snapping keeps above any surface. The 3D reference lifts by the bare hitHeight (its
                // collider-cast returns a clean top-surface hit even from a flush rest), but in 2D a box proxy whose
                // bottom is flush with the step top registers the step only as a zero-fraction overlap with no
                // opposing normal — so without the clearance the next frame's down-probe can never re-ground on the
                // step and grounding would fall through to the lower floor. The clearance lets normal grounding take
                // over on the step top cleanly once the centre clears the edge.
                characterPosition += characterBody.GroundingUp * (hitHeight + Constants.CollisionOffset);
                characterPosition += forwardCheckDirection * forwardStepHitDistance;

                characterBody.IsGrounded = true;
                characterBody.GroundHit = stepHit;

                // Suppress next frame's downward ground-snap until the centre clears the step edge. The swept
                // MovePosition delivers this lifted pose next frame (D3 latency); a box proxy resting with its edge
                // flush against the step's vertical face makes the step a zero-fraction overlap in the down-probe (no
                // opposing top normal), so the grounding step would otherwise snap the character down onto the lower
                // floor it just climbed from. The flag holds the snap until the step top is cleanly grounded
                // (Update_Grounding clears it). See KinematicCharacterBody2D.SuppressGroundSnappingUntilSteppedClear.
                characterBody.SuppressGroundSnappingUntilSteppedClear = true;

                // Kill the velocity component along grounding-up (we are now standing on the step top).
                float2 characterVelocityBeforeHit = characterBody.RelativeVelocity;
                characterBody.RelativeVelocity = MathUtilities2D.ProjectOnPlane(
                    characterBody.RelativeVelocity,
                    characterBody.GroundingUp
                );
                remainingMovementDirection = normalizesafe(characterBody.RelativeVelocity);
                remainingMovementLength -= forwardStepHitDistance;

                hit = CreateCharacterHit(
                    in stepHit,
                    characterBody.IsGrounded,
                    characterVelocityBeforeHit,
                    isGroundedOnStepHit
                );
                hit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;

                hasSteppedUp = true;
            }
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 5 (movement) — initial-overlap velocity pre-pass (REF/KinematicCharacterUtilities.cs:2192)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Seeds velocity-projection hits from the character's initial overlaps, before the move iterations, so a
        /// character starting the step inside a body projects its velocity off that body's surface first. 2D port of
        /// <c>ProjectVelocityOnInitialOverlaps</c> (REF/KinematicCharacterUtilities.cs:2192): the 3D
        /// <c>CalculateDistance</c> has no substrate analogue, so the overlap normals are recovered via the same D2
        /// cast-back <see cref="SolveOverlaps{T,C}"/> uses.
        /// </summary>
        public static void ProjectVelocityOnInitialOverlaps<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in KinematicCharacterColliderProxy2D colliderProxy,
            float characterRotation,
            float2 characterPosition,
            in ComponentLookup<Unity.Transforms.LocalToWorld> bodyTransformLookup,
            float2 originalVelocityDirection,
            DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHitsBuffer
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            int overlapCount = OverlapProxy(ref baseContext, in colliderProxy, characterPosition, characterRotation);
            if (overlapCount <= 0)
            {
                return;
            }

            // Sensor (isTrigger) overlaps are skipped: a sensor is non-solid to the controller, so it seeds no
            // initial velocity-projection hit (the character slides through it). See IsSensorHit.
            var overlappingEntities = new NativeList<Entity>(overlapCount, Allocator.Temp);
            for (int i = 0; i < baseContext.TmpQueryHits.Length; i++)
            {
                PhysicsQueryHit2D overlapHit = baseContext.TmpQueryHits[i];
                if (
                    overlapHit.entity != characterEntity
                    && overlapHit.entity != Entity.Null
                    && !IsSensorHit(in overlapHit)
                )
                {
                    overlappingEntities.Add(overlapHit.entity);
                }
            }

            for (int i = 0; i < overlappingEntities.Length; i++)
            {
                Entity overlapEntity = overlappingEntities[i];
                if (
                    ReconstructOverlap(
                        ref baseContext,
                        in colliderProxy,
                        characterPosition,
                        characterRotation,
                        overlapEntity,
                        in bodyTransformLookup,
                        out float2 overlapNormal,
                        out float _
                    )
                )
                {
                    BasicHit2D basicOverlapHit = new BasicHit2D(overlapEntity, characterPosition, overlapNormal);
                    if (!processor.CanCollideWithHit(ref context, ref baseContext, in basicOverlapHit))
                        continue;

                    bool isGroundedOnHit = false;
                    if (characterProperties.EvaluateGrounding)
                    {
                        isGroundedOnHit = processor.IsGroundedOnHit(
                            ref context,
                            ref baseContext,
                            in basicOverlapHit,
                            (int)GroundingEvaluationType2D.InitialOverlaps
                        );
                    }

                    if (dot(characterBody.RelativeVelocity, overlapNormal) < 0f)
                    {
                        velocityProjectionHitsBuffer.Add(
                            new KinematicVelocityProjectionHit2D(basicOverlapHit, isGroundedOnHit)
                        );
                        processor.ProjectVelocityOnHits(
                            ref context,
                            ref baseContext,
                            ref characterBody.RelativeVelocity,
                            ref characterBody.IsGrounded,
                            ref characterBody.GroundHit,
                            in velocityProjectionHitsBuffer,
                            originalVelocityDirection
                        );
                    }
                }
            }

            overlappingEntities.Dispose();
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 5 (dynamics) — character ↔ character + character ↔ regular-dynamic-body impulse exchange
        // (REF/KinematicCharacterUtilities.cs:3166)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Solves the impulse exchange for every recorded character hit on a body with a real rigidbody (≠ parent),
        /// applying the self-impulse to the character's own velocity and emitting a deferred impulse on the other
        /// body. 2D port of <c>ProcessCharacterHitDynamics</c> (REF/KinematicCharacterUtilities.cs:3166).
        ///
        /// <para><b>The D5 read resolution.</b> The other body's velocity/mass come from: a
        /// <see cref="StoredKinematicCharacterData2D"/> if the hit is another CHARACTER (Burst-clean — a
        /// <see cref="ComponentLookup{T}"/> read), or a <see cref="StoredDynamicBodyData2D"/> snapshot if the hit is
        /// a regular DYNAMIC body. That snapshot is written once per step on the main thread by
        /// <see cref="StoreDynamicBodyDataSystem2D"/> (the body-velocity read is a managed
        /// <c>Unity.U2D.Physics.PhysicsBody</c> property, not Burst-callable — C2 D5), so this method — and the whole
        /// solve job — reads only Burst-safe component lookups and never touches the live handle, keeping the solve
        /// <c>ScheduleParallel</c>.</para>
        ///
        /// <para>The deferred impulse OUT carries the raw impulse the solver produced (the substrate's
        /// <c>AddForce(Impulse)</c> mass-scales it during the step — C2 coupling note), and is skipped for a
        /// character that is itself moving toward us (it will solve the pair in its own update).</para>
        /// </summary>
        public static void ProcessCharacterHitDynamics<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            float2 characterPosition,
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            for (int b = 0; b < characterHitsBuffer.Length; b++)
            {
                KinematicCharacterHit2D characterHit = characterHitsBuffer[b];
                Entity hitBodyEntity = characterHit.Entity;

                if (hitBodyEntity == Entity.Null || hitBodyEntity == characterBody.ParentEntity)
                {
                    continue;
                }

                // Self motion/mass.
                float2 selfLinearVelocity = characterBody.RelativeVelocity + characterBody.ParentVelocity;
                KinematicCharacterMass2D selfMass = PhysicsUtilities2D.GetKinematicCharacterMass(
                    in characterProperties
                );
                float selfAngularVelocity = 0f;

                // Other body's motion/mass — character via StoredKinematicCharacterData2D, else a regular dynamic
                // body via the main-thread StoredDynamicBodyData2D snapshot. A body that is neither is skipped (no
                // velocity/mass to exchange against — a static body, infinite mass, no response).
                bool otherIsCharacter = false;
                bool otherIsDynamic = false;
                float2 otherLinearVelocity = new float2(0f, 0f);
                float otherAngularVelocity = 0f;
                KinematicCharacterMass2D otherMass = default;
                float2 otherCenterOfMass = characterHit.Position;

                if (baseContext.StoredCharacterBodyPropertiesLookup.HasComponent(hitBodyEntity))
                {
                    StoredKinematicCharacterData2D data = baseContext.StoredCharacterBodyPropertiesLookup[
                        hitBodyEntity
                    ];
                    otherIsCharacter = true;
                    otherIsDynamic = data.SimulateDynamicBody;
                    otherLinearVelocity = data.RelativeVelocity + data.ParentVelocity;
                    otherMass = PhysicsUtilities2D.GetKinematicCharacterMass(in data);
                }
                else if (baseContext.DynamicBodyDataLookup.HasComponent(hitBodyEntity))
                {
                    StoredDynamicBodyData2D data = baseContext.DynamicBodyDataLookup[hitBodyEntity];
                    otherIsDynamic = data.IsDynamic && data.Mass > 0f;
                    otherLinearVelocity = data.LinearVelocity;
                    otherAngularVelocity = data.AngularVelocity;
                    otherMass = new KinematicCharacterMass2D
                    {
                        CenterOfMass = new float2(0f, 0f),
                        InverseMass = otherIsDynamic ? (1f / data.Mass) : 0f,
                        InverseInertia =
                            (otherIsDynamic && data.RotationalInertia > 0f) ? (1f / data.RotationalInertia) : 0f,
                    };
                }
                else
                {
                    // Neither a character nor a snapshotted dynamic body — a static body. No impulse to exchange.
                    continue;
                }

                // Correct the effective hit normal for grounding considerations.
                float2 effectiveHitNormalFromOtherToSelf = characterHit.Normal;
                if (characterHit.WasCharacterGroundedOnHitEnter && !characterHit.IsGroundedOnHit)
                {
                    effectiveHitNormalFromOtherToSelf = normalizesafe(
                        MathUtilities2D.ProjectOnPlane(characterHit.Normal, characterBody.GroundingUp)
                    );
                }
                else if (characterHit.IsGroundedOnHit)
                {
                    effectiveHitNormalFromOtherToSelf = characterBody.GroundingUp;
                }

                // Don't reorient the normal to grounding-up for a dynamic body we're not grounded on.
                if (otherIsDynamic && !characterHit.IsGroundedOnHit)
                {
                    effectiveHitNormalFromOtherToSelf = characterHit.Normal;
                }

                // Mass overrides (only for a regular dynamic body, both dynamic).
                if (characterProperties.SimulateDynamicBody && otherIsDynamic && !otherIsCharacter)
                {
                    if (selfMass.InverseMass > 0f && otherMass.InverseMass > 0f)
                    {
                        processor.OverrideDynamicHitMasses(
                            ref context,
                            ref baseContext,
                            ref selfMass,
                            ref otherMass,
                            new BasicHit2D(characterHit.Entity, characterHit.Position, characterHit.Normal)
                        );
                    }
                }

                // Kinematic-vs-kinematic special case: pretend a mass of 1 against another kinematic body, and
                // cancel another kinematic character's velocity toward us (prevents the pair bumping each other).
                if (!characterProperties.SimulateDynamicBody && !otherIsDynamic)
                {
                    selfMass.InverseMass = 1f;

                    if (otherIsCharacter && dot(otherLinearVelocity, effectiveHitNormalFromOtherToSelf) > 0f)
                    {
                        otherLinearVelocity = MathUtilities2D.ProjectOnPlane(
                            otherLinearVelocity,
                            effectiveHitNormalFromOtherToSelf
                        );
                    }
                }

                // Restore the velocity lost during the original hit projection (so dynamics re-solves it).
                float2 velocityLostInOriginalProjection = projectsafe(
                    characterHit.CharacterVelocityBeforeHit - characterHit.CharacterVelocityAfterHit,
                    effectiveHitNormalFromOtherToSelf
                );
                selfLinearVelocity += velocityLostInOriginalProjection;

                // Solve the impulse.
                PhysicsUtilities2D.SolveCollisionImpulses(
                    selfLinearVelocity,
                    selfAngularVelocity,
                    otherLinearVelocity,
                    otherAngularVelocity,
                    in selfMass,
                    in otherMass,
                    characterPosition,
                    otherCenterOfMass,
                    characterHit.Position,
                    effectiveHitNormalFromOtherToSelf,
                    out float2 impulseOnSelf,
                    out float2 impulseOnOther
                );

                // Apply the self-impulse to the character's own velocity.
                float2 previousSelfLinearVel = selfLinearVelocity;
                PhysicsUtilities2D.ApplyLinearImpulse(ref selfLinearVelocity, in selfMass, impulseOnSelf);
                float2 characterLinearVelocityChange =
                    velocityLostInOriginalProjection + (selfLinearVelocity - previousSelfLinearVel);
                characterBody.RelativeVelocity += characterLinearVelocityChange;

                // Trim velocity going toward the ground (avoids the reoriented-velocity issue on a grounded hit).
                if (
                    characterHit.IsGroundedOnHit
                    && dot(characterBody.RelativeVelocity, characterHit.Normal) < -Constants.DotProductSimilarityEpsilon
                )
                {
                    characterBody.RelativeVelocity = MathUtilities2D.ProjectOnPlane(
                        characterBody.RelativeVelocity,
                        characterBody.GroundingUp
                    );
                    characterBody.RelativeVelocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                        characterBody.RelativeVelocity,
                        characterHit.Normal,
                        characterBody.GroundingUp
                    );
                }

                // If the other is a character moving toward us, it solves the pair in its own update — don't double-apply.
                bool otherIsCharacterMovingTowardsUs =
                    otherIsCharacter
                    && dot(otherLinearVelocity, effectiveHitNormalFromOtherToSelf)
                        > Constants.DotProductSimilarityEpsilon;

                // Emit the deferred impulse on the other body (only a dynamic, non-character-moving-toward-us body).
                if (!otherIsCharacterMovingTowardsUs && otherIsDynamic && lengthsq(impulseOnOther) > 0f)
                {
                    deferredImpulsesBuffer.Add(
                        new KinematicCharacterDeferredImpulse2D
                        {
                            OnEntity = hitBodyEntity,
                            // The deferred system applies this to a regular body as AddForce(Impulse), which Box2D
                            // mass-scales — so the OUT value is the raw impulse, not a pre-divided Δv (C2 coupling note).
                            // For a character target the deferred system adds it as a RelativeVelocity Δv, so a
                            // character's deferred impulse must be a velocity change; the solver's impulseOnOther is a
                            // raw impulse, divided by the other character's mass here. otherIsCharacterMovingTowardsUs
                            // already gates a character that solves itself; a character NOT moving toward us still
                            // receives the velocity change.
                            LinearVelocityChange = otherIsCharacter
                                ? (impulseOnOther * otherMass.InverseMass)
                                : impulseOnOther,
                            AngularVelocityChange = 0f,
                        }
                    );
                }
            }
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 6 — Ground pushing (REF/KinematicCharacterUtilities.cs:873)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Pushes the ground body down with the character's weight (gravity·mass) as a deferred impulse, when the
        /// character is grounded on a regular dynamic body and itself simulates as a dynamic body. 2D port of
        /// <c>Update_GroundPushing</c> (REF/KinematicCharacterUtilities.cs:873). The ground's velocity/mass come
        /// from the main-thread <see cref="StoredDynamicBodyData2D"/> snapshot (D5 — never the live handle in a
        /// Burst job). The deferred OUT carries the raw impulse the solver produces.
        /// </summary>
        public static void Update_GroundPushing<T, C>(
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            float2 characterPosition,
            DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
            float2 gravity,
            float forceMultiplier = 1f
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            if (!characterBody.IsGrounded || !characterProperties.SimulateDynamicBody)
            {
                return;
            }

            Entity groundEntity = characterBody.GroundHit.Entity;
            if (groundEntity == Entity.Null || !baseContext.DynamicBodyDataLookup.HasComponent(groundEntity))
            {
                return;
            }

            StoredDynamicBodyData2D groundData = baseContext.DynamicBodyDataLookup[groundEntity];
            if (!groundData.IsDynamic || groundData.Mass <= 0f)
            {
                return;
            }

            KinematicCharacterMass2D selfMass = PhysicsUtilities2D.GetKinematicCharacterMass(in characterProperties);
            selfMass.InverseMass = 1f / characterProperties.Mass;
            KinematicCharacterMass2D groundMass = new KinematicCharacterMass2D
            {
                CenterOfMass = new float2(0f, 0f),
                InverseMass = 1f / groundData.Mass,
                InverseInertia = groundData.RotationalInertia > 0f ? (1f / groundData.RotationalInertia) : 0f,
            };

            processor.OverrideDynamicHitMasses(
                ref context,
                ref baseContext,
                ref selfMass,
                ref groundMass,
                new BasicHit2D(
                    characterBody.GroundHit.Entity,
                    characterBody.GroundHit.Position,
                    characterBody.GroundHit.Normal
                )
            );

            float2 groundNormalUp = -normalizesafe(gravity);
            // Self "velocity" for the push solve = the ground point velocity + the gravity step (the weight pressing
            // the ground), exactly as the 3D ground-push builds its self velocity.
            float2 groundPointVelocity = PhysicsUtilities2D.GetPointVelocity(
                groundData.LinearVelocity,
                groundData.AngularVelocity,
                characterPosition,
                characterBody.GroundHit.Position
            );
            float2 selfPushVelocity = groundPointVelocity + (gravity * baseContext.Time.DeltaTime);

            PhysicsUtilities2D.SolveCollisionImpulses(
                selfPushVelocity,
                0f,
                groundData.LinearVelocity,
                groundData.AngularVelocity,
                in selfMass,
                in groundMass,
                characterPosition,
                characterPosition,
                characterBody.GroundHit.Position,
                groundNormalUp,
                out float2 _,
                out float2 impulseOnOther
            );

            if (lengthsq(impulseOnOther) > 0f)
            {
                deferredImpulsesBuffer.Add(
                    new KinematicCharacterDeferredImpulse2D
                    {
                        OnEntity = groundEntity,
                        LinearVelocityChange = impulseOnOther * forceMultiplier,
                        AngularVelocityChange = 0f,
                    }
                );
            }
        }

        // -----------------------------------------------------------------------------------------------------
        // Step 8 — Stateful-hit Enter/Stay/Exit diff (REF/KinematicCharacterUtilities.cs:1013)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// Diffs this step's character hits against last step's stateful hits to assign Enter / Stay / Exit states —
        /// the <c>OnCollisionEnter/Stay/Exit</c> analogue. 2D port of <c>Update_ProcessStatefulCharacterHits</c>
        /// (REF/KinematicCharacterUtilities.cs:1013): the old stateful hits are at the front of the buffer, new ones
        /// are appended, then the old range is removed — verbatim, no dimensional content.
        /// </summary>
        public static void Update_ProcessStatefulCharacterHits(
            DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
            DynamicBuffer<StatefulKinematicCharacterHit2D> statefulHitsBuffer
        )
        {
            int lastIndexOfOldStatefulHits = statefulHitsBuffer.Length - 1;

            // Add new stateful hits.
            for (int hitIndex = 0; hitIndex < characterHitsBuffer.Length; hitIndex++)
            {
                KinematicCharacterHit2D characterHit = characterHitsBuffer[hitIndex];
                if (
                    !StatefulHitsContainEntity(
                        in statefulHitsBuffer,
                        characterHit.Entity,
                        lastIndexOfOldStatefulHits + 1,
                        statefulHitsBuffer.Length
                    )
                )
                {
                    StatefulKinematicCharacterHit2D newStatefulHit = new StatefulKinematicCharacterHit2D(characterHit);
                    bool wasInBefore = OldStatefulHitsContainEntity(
                        in statefulHitsBuffer,
                        characterHit.Entity,
                        lastIndexOfOldStatefulHits,
                        out CharacterHitState2D oldState
                    );

                    if (wasInBefore)
                    {
                        newStatefulHit.State = oldState switch
                        {
                            CharacterHitState2D.Enter => CharacterHitState2D.Stay,
                            CharacterHitState2D.Stay => CharacterHitState2D.Stay,
                            CharacterHitState2D.Exit => CharacterHitState2D.Enter,
                            _ => CharacterHitState2D.Enter,
                        };
                    }
                    else
                    {
                        newStatefulHit.State = CharacterHitState2D.Enter;
                    }

                    statefulHitsBuffer.Add(newStatefulHit);
                }
            }

            // Detect Exit states: an old hit entity not present in the new hits is appended as Exit.
            for (int i = 0; i <= lastIndexOfOldStatefulHits; i++)
            {
                StatefulKinematicCharacterHit2D oldStatefulHit = statefulHitsBuffer[i];
                if (
                    oldStatefulHit.State != CharacterHitState2D.Exit
                    && !StatefulHitsContainEntity(
                        in statefulHitsBuffer,
                        oldStatefulHit.Hit.Entity,
                        lastIndexOfOldStatefulHits + 1,
                        statefulHitsBuffer.Length
                    )
                )
                {
                    oldStatefulHit.State = CharacterHitState2D.Exit;
                    statefulHitsBuffer.Add(oldStatefulHit);
                }
            }

            // Remove all old stateful hits (the front range).
            if (lastIndexOfOldStatefulHits >= 0)
            {
                statefulHitsBuffer.RemoveRange(0, lastIndexOfOldStatefulHits + 1);
            }
        }

        /// <summary>Whether the OLD stateful hits (indices [0, lastIndexOfOldStatefulHits]) contain an entity.</summary>
        static bool OldStatefulHitsContainEntity(
            in DynamicBuffer<StatefulKinematicCharacterHit2D> statefulHitsBuffer,
            Entity entity,
            int lastIndexOfOldStatefulHits,
            out CharacterHitState2D oldState
        )
        {
            oldState = default;
            if (lastIndexOfOldStatefulHits < 0)
            {
                return false;
            }

            for (int i = 0; i <= lastIndexOfOldStatefulHits; i++)
            {
                StatefulKinematicCharacterHit2D oldStatefulHit = statefulHitsBuffer[i];
                if (oldStatefulHit.Hit.Entity == entity)
                {
                    oldState = oldStatefulHit.State;
                    return true;
                }
            }

            return false;
        }

        /// <summary>Whether the NEW stateful hits (indices [firstIndexOfNewStatefulHits, length)) contain an entity.</summary>
        static bool StatefulHitsContainEntity(
            in DynamicBuffer<StatefulKinematicCharacterHit2D> statefulHitsBuffer,
            Entity entity,
            int firstIndexOfNewStatefulHits,
            int length
        )
        {
            if (firstIndexOfNewStatefulHits >= length)
            {
                return false;
            }

            for (int i = firstIndexOfNewStatefulHits; i < length; i++)
            {
                StatefulKinematicCharacterHit2D newStatefulHit = statefulHitsBuffer[i];
                if (newStatefulHit.Hit.Entity == entity)
                {
                    return true;
                }
            }

            return false;
        }

        // -----------------------------------------------------------------------------------------------------
        // Step-aware grounding (REF/KinematicCharacterUtilities.cs:1131 — full signature with step handling)
        // -----------------------------------------------------------------------------------------------------

        /// <summary>
        /// The step-aware grounding test: the C4a slope-only <see cref="Default_IsGroundedOnHit"/> ORed with the
        /// step-grounding result (<see cref="IsGroundedOnSteps"/>) when step handling is on and the slope check
        /// failed. 2D port of the full <c>Default_IsGroundedOnHit</c> (REF/KinematicCharacterUtilities.cs:1131)
        /// step branch (REF :1151). A processor with step handling enabled calls THIS overload; the slope-only
        /// overload remains for the step-disabled default. Step-grounding is only evaluated during the ground-probe
        /// and step-up phases (matching the 3D guard) and is skipped for a dynamic ground body.
        /// </summary>
        public static bool Default_IsGroundedOnHit(
            ref KinematicCharacterUpdateContext2D baseContext,
            Entity characterEntity,
            in KinematicCharacterBody2D characterBody,
            in KinematicCharacterProperties2D characterProperties,
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            in BasicHit2D hit,
            int groundingEvaluationType
        )
        {
            if (
                ShouldPreventGroundingBasedOnVelocity(
                    ref baseContext,
                    in hit,
                    characterBody.WasGroundedBeforeCharacterUpdate,
                    characterBody.RelativeVelocity
                )
            )
            {
                return false;
            }

            bool isGroundedOnSlope = IsGroundedOnSlopeNormal(
                characterProperties.MaxGroundedSlopeDotProduct,
                hit.Normal,
                characterBody.GroundingUp
            );

            bool isGroundedOnSteps = false;
            if (!isGroundedOnSlope && stepAndSlopeHandling.StepHandling && stepAndSlopeHandling.MaxStepHeight > 0f)
            {
                bool hitIsOnCharacterBottom =
                    dot(characterBody.GroundingUp, hit.Normal) > Constants.DotProductSimilarityEpsilon;
                if (
                    hitIsOnCharacterBottom
                    && (
                        groundingEvaluationType == (int)GroundingEvaluationType2D.GroundProbing
                        || groundingEvaluationType == (int)GroundingEvaluationType2D.StepUpHit
                    )
                )
                {
                    // Don't step-ground onto a dynamic body (avoids a character stepping onto a body rolling at it).
                    bool hitIsDynamic =
                        baseContext.DynamicBodyDataLookup.HasComponent(hit.Entity)
                        && baseContext.DynamicBodyDataLookup[hit.Entity].IsDynamic;
                    if (!hitIsDynamic)
                    {
                        isGroundedOnSteps = IsGroundedOnSteps(
                            ref baseContext,
                            characterEntity,
                            in characterBody,
                            in characterProperties,
                            in hit,
                            stepAndSlopeHandling.MaxStepHeight,
                            stepAndSlopeHandling.ExtraStepChecksDistance
                        );
                    }
                }
            }

            return isGroundedOnSlope || isGroundedOnSteps;
        }
    }
}
