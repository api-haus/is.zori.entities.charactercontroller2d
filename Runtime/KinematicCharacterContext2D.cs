using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The per-entity context for one character's solve — the non-Aspect replacement for the 3D
    /// <c>KinematicCharacterAspect</c>. The 3D Aspect bundled the character's <c>RefRW&lt;&gt;</c> /
    /// <c>DynamicBuffer&lt;&gt;</c> handles for ergonomic call sites and carried NO state; every one of its methods
    /// forwarded to a static <c>KinematicCharacterUtilities</c> method. Entities 6.5 dropped <c>IAspect</c>, so this
    /// is a plain blittable <c>struct</c> built fresh inside the solve <c>IJobEntity.Execute</c> from the entity's
    /// own component values + the job's buffers. <see cref="KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D{T,C}"/>
    /// chains the core static steps over it and delivers the final pose with a single
    /// <see cref="PhysicsBody2DCommands.MovePosition"/>.
    ///
    /// <para><b>Why a plain struct, not a <c>ref struct</c> with <c>ref</c> fields.</b> Holding the mutated
    /// <see cref="KinematicCharacterBody2D"/> by a <c>ref</c> field would need C# 11 <c>ref</c>-field support, which
    /// the editor's Burst/IL2CPP C# language level does not guarantee. So the context holds the read-mostly data by
    /// VALUE (entity, proxy, properties, the current pose) and the buffers by value (a <see cref="DynamicBuffer{T}"/>
    /// is itself a small struct aliasing chunk memory, so writes through it land on the entity); the one component
    /// the solve MUTATES — the character body — is threaded as a <c>ref</c> PARAMETER through
    /// <see cref="KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D{T,C}"/>, not stored in the struct.</para>
    ///
    /// <para><b>Pose I/O — the deepest 3D→2D divergence.</b> The 3D controller owned <c>LocalTransform</c> and the
    /// move was a write to <c>LocalTransform.Position</c>. The 2D substrate owns <see cref="LocalToWorld"/> (written
    /// by <c>PhysicsBody2DWriteBackSystem</c>) and a body moves only by a command on its
    /// <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c>. So the context READS the current pose from
    /// <see cref="LocalToWorld"/> (position = the matrix translation, z-angle = the local +X basis angle) and WRITES
    /// the solved pose by enqueuing one <c>MovePosition</c> the substrate drains and applies next step (one-step
    /// pipeline latency).</para>
    /// </summary>
    public struct KinematicCharacterContext2D
    {
        /// <summary>The character entity.</summary>
        public Entity Entity;

        /// <summary>The character's static properties (grounding/collision/dynamics toggles, mass), by value.</summary>
        public KinematicCharacterProperties2D CharacterProperties;

        /// <summary>The circle-or-box cast proxy the solve sweeps, by value.</summary>
        public KinematicCharacterColliderProxy2D ColliderProxy;

        /// <summary>The detected-character-hits buffer (cleared at Initialize, filled through the solve).</summary>
        public DynamicBuffer<KinematicCharacterHit2D> CharacterHitsBuffer;

        /// <summary>The stateful Enter/Stay/Exit hit buffer (diffed by the stateful-hit pass; untouched here).</summary>
        public DynamicBuffer<StatefulKinematicCharacterHit2D> StatefulHitsBuffer;

        /// <summary>The deferred-impulse buffer (recorded during the solve, drained by the deferred-impulses system).</summary>
        public DynamicBuffer<KinematicCharacterDeferredImpulse2D> DeferredImpulsesBuffer;

        /// <summary>The running set of surface lines velocity is projected against this step.</summary>
        public DynamicBuffer<KinematicVelocityProjectionHit2D> VelocityProjectionHits;

        /// <summary>
        /// The character body's move queue. NEW vs the 3D Aspect (which wrote <c>LocalTransform.Position</c>): the
        /// solve enqueues one <see cref="PhysicsBody2DCommands.MovePosition"/> per step here, drained by
        /// <c>PhysicsWorld2DSystem</c> before the next step.
        /// </summary>
        public DynamicBuffer<PhysicsBody2DCommand> CommandBuffer;

        /// <summary>
        /// The current world pose, read from the entity's <see cref="LocalToWorld"/>. Position is the matrix
        /// translation; the z-angle is the local +X basis angle (<c>atan2(c0.y, c0.x)</c>) of the flat, unscaled
        /// physics-body matrix — the same pose source <c>TrackedTransformSystem2D</c> uses.
        /// </summary>
        public float2 CurrentPosition;

        /// <summary>The current z-rotation (radians), read from <see cref="LocalToWorld"/>.</summary>
        public float CurrentRotation;
    }

    /// <summary>
    /// The convenience that chains the core solve steps for one character per fixed step — the 2D analogue of the
    /// 3D Standard Characters sample's <c>aspect.PhysicsUpdate(...)</c> (the package ships the steps; the sample
    /// chains them). It runs Initialize → Grounding → MovementAndDecollisions over a
    /// <see cref="KinematicCharacterContext2D"/> + the live character body, then delivers the final pose with one
    /// <see cref="PhysicsBody2DCommands.MovePosition"/>.
    ///
    /// <para>The omitted steps (parent movement, prevent-grounding-from-future-slope-change, hit dynamics, ground
    /// pushing, moving-platform momentum, stateful-hit diff) are advanced features; they slot into this chain at the
    /// seams marked in <see cref="KinematicCharacterUtilities2D"/>. A consumer that wants a fuller chain can call the
    /// static steps directly instead of this convenience.</para>
    /// </summary>
    public static class KinematicCharacterPhysicsUpdate2D
    {
        /// <summary>
        /// Reads <see cref="KinematicCharacterContext2D.CurrentPosition"/> /
        /// <see cref="KinematicCharacterContext2D.CurrentRotation"/> off a character's <see cref="LocalToWorld"/>
        /// into the per-entity context. Position from the matrix translation (<c>c3.xy</c>), z-angle from the local
        /// +X basis (<c>atan2(c0.y, c0.x)</c>) — the substrate body matrix is flat and unscaled in the physics
        /// plane, so the +X column is the rotated right axis.
        /// </summary>
        public static void ReadPoseFromLocalToWorld(
            ref KinematicCharacterContext2D characterContext,
            in LocalToWorld localToWorld
        )
        {
            float4x4 m = localToWorld.Value;
            characterContext.CurrentPosition = m.c3.xy;
            characterContext.CurrentRotation = atan2(m.c0.y, m.c0.x);
        }

        /// <summary>
        /// Runs the core solve chain over <paramref name="characterContext"/> + the live
        /// <paramref name="characterBody"/> and enqueues the resulting pose as a single
        /// <see cref="PhysicsBody2DCommands.MovePosition"/>. The base context <paramref name="baseContext"/> carries
        /// the per-system global data (world handle, lookups, scratch list); <paramref name="bodyTransformLookup"/>
        /// is the <see cref="LocalToWorld"/> lookup the depenetration cast-back uses to read an overlapping
        /// body's center.
        ///
        /// <para>The solve computes a final <c>characterPosition</c> starting from the context's
        /// <see cref="KinematicCharacterContext2D.CurrentPosition"/>; the move command carries that absolute target.
        /// Rotation is NOT moved here — a platformer character's rotation is fixed or driven by the processor on the
        /// regular update, not the fixed solve; a consumer that rotates the character enqueues its own
        /// <c>MoveRotation</c>.</para>
        /// </summary>
        public static void PhysicsUpdate2D<T, C>(
            ref KinematicCharacterContext2D characterContext,
            ref KinematicCharacterBody2D characterBody,
            in T processor,
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in ComponentLookup<LocalToWorld> bodyTransformLookup,
            in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
            float2 gravity,
            float deltaTime
        )
            where T : unmanaged, IKinematicCharacterProcessor2D<C>
            where C : unmanaged
        {
            float2 characterPosition = characterContext.CurrentPosition;
            float characterRotation = characterContext.CurrentRotation;

            // Step 1 — Initialize (clears buffers, snapshots state, calls UpdateGroundingUp).
            KinematicCharacterUtilities2D.Update_Initialize(
                in processor,
                ref context,
                ref baseContext,
                ref characterBody,
                characterContext.CharacterHitsBuffer,
                characterContext.DeferredImpulsesBuffer,
                characterContext.VelocityProjectionHits,
                deltaTime
            );

            // Step 2 — Parent movement (advanced feature): carry the pose rigidly with a TrackedTransform2D parent's delta.
            KinematicCharacterUtilities2D.Update_ParentMovement(
                in processor,
                ref context,
                ref baseContext,
                characterContext.Entity,
                ref characterBody,
                in characterContext.CharacterProperties,
                in characterContext.ColliderProxy,
                characterRotation,
                ref characterPosition
            );

            // Step 3 — Grounding (detect ground, snap, project velocity onto it).
            KinematicCharacterUtilities2D.Update_Grounding(
                in processor,
                ref context,
                ref baseContext,
                ref characterBody,
                characterContext.Entity,
                in characterContext.CharacterProperties,
                in characterContext.ColliderProxy,
                characterRotation,
                characterContext.VelocityProjectionHits,
                characterContext.CharacterHitsBuffer,
                ref characterPosition
            );

            // Step 4 — Prevent grounding from a future slope change (advanced feature): launch off a ledge cleanly.
            KinematicCharacterUtilities2D.Update_PreventGroundingFromFutureSlopeChange(
                in processor,
                ref context,
                ref baseContext,
                characterContext.Entity,
                ref characterBody,
                in characterContext.CharacterProperties,
                in stepAndSlopeHandling
            );

            // Step 5 — Movement and decollisions (collide-and-slide + depenetration + step-up + hit dynamics).
            KinematicCharacterUtilities2D.Update_MovementAndDecollisions(
                in processor,
                ref context,
                ref baseContext,
                characterContext.Entity,
                ref characterBody,
                in characterContext.CharacterProperties,
                in characterContext.ColliderProxy,
                in stepAndSlopeHandling,
                characterRotation,
                in bodyTransformLookup,
                characterContext.VelocityProjectionHits,
                characterContext.CharacterHitsBuffer,
                characterContext.DeferredImpulsesBuffer,
                ref characterPosition
            );

            // Step 6 — Ground pushing (advanced feature): press the dynamic ground down with the character's weight.
            KinematicCharacterUtilities2D.Update_GroundPushing(
                in processor,
                ref context,
                ref baseContext,
                ref characterBody,
                in characterContext.CharacterProperties,
                characterPosition,
                characterContext.DeferredImpulsesBuffer,
                gravity
            );

            // Step 7 — Moving-platform detection + parent momentum (advanced feature): auto-parent to a tracked ground and carry
            // momentum across a parent change.
            KinematicCharacterUtilities2D.Update_MovingPlatformDetection(ref baseContext, ref characterBody);
            KinematicCharacterUtilities2D.Update_ParentMomentum(ref baseContext, ref characterBody, characterPosition);

            // Step 8 — Stateful-hit Enter/Stay/Exit diff (advanced feature).
            KinematicCharacterUtilities2D.Update_ProcessStatefulCharacterHits(
                characterContext.CharacterHitsBuffer,
                characterContext.StatefulHitsBuffer
            );

            // Deliver the solved pose: one swept MovePosition the substrate drains before the next step.
            // The solve already resolved collisions, so this move is short and obstruction-free by construction.
            PhysicsBody2DCommands.MovePosition(characterContext.CommandBuffer, characterPosition);
        }
    }
}
