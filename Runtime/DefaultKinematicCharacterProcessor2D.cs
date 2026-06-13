using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The minimal user-context (the <c>C</c> in <see cref="IKinematicCharacterProcessor2D{C}"/>) for the default
    /// processor — the 2D analogue of the 3D sample's <c>…CharacterUpdateContext</c>
    /// (REF/Samples~/…/FirstPersonCharacterProcessor.cs:8). It carries no extra global data; the default solve needs
    /// only the base context. A richer consumer adds its own context with <c>ComponentLookup</c>s / singletons.
    /// </summary>
    public struct DefaultCharacterUpdateContext2D { }

    /// <summary>
    /// The concrete default processor (design D4) — a faithful 2D port of the Standard Characters sample's
    /// <c>IKinematicCharacterProcessor</c> implementation (REF/Samples~/…/FirstPersonCharacterProcessor.cs:24),
    /// minus the camera/look concerns that have no 2D meaning. It wires the six callbacks to the
    /// <c>Default_*</c> core behaviours so the package has a runnable out-of-box character.
    ///
    /// <para>The 3D sample processor held <c>RefRW&lt;&gt;</c> handles to the character's components
    /// (<c>CharacterDataAccess</c>) and read them inside the callbacks. In Entities 6.5 the solve runs inside an
    /// <c>IJobEntity</c> that already holds the components by <c>ref</c>/<c>in</c>, so this processor instead carries
    /// the READ-ONLY data the non-mutating callbacks need — a snapshot of the body's grounding state, the
    /// properties, and the constrain-to-ground flag — taken once at the start of the solve. The mutating
    /// movement-hit behaviour is NOT routed through this processor's <see cref="OnMovementHit"/> during the core
    /// loop: <see cref="KinematicCharacterUtilities2D.MoveWithCollisions{T,C}"/> calls the static
    /// <see cref="KinematicCharacterUtilities2D.OnMovementHit{T,C}"/> directly (it mutates the live position/body
    /// the job holds by <c>ref</c>, which the <c>unmanaged</c> processor cannot carry without
    /// <c>ComponentLookup</c> indirection). This processor's <see cref="OnMovementHit"/> still forwards to the same
    /// static for a consumer that drives the steps itself.</para>
    ///
    /// <para>All callbacks are HPC#-clean and carry no <c>[BurstCompile]</c> (the solve job is the entry point).</para>
    /// </summary>
    public struct DefaultKinematicCharacterProcessor2D : IKinematicCharacterProcessor2D<DefaultCharacterUpdateContext2D>
    {
        /// <summary>
        /// A read-only snapshot of the character's grounding state, taken at the start of the solve. The
        /// non-mutating callbacks (<see cref="IsGroundedOnHit"/>, <see cref="ProjectVelocityOnHits"/>) read
        /// grounding-up / was-grounded / relative-velocity off it, exactly as the 3D callbacks read
        /// <c>CharacterBody.ValueRO</c>. It is the body as it was when the processor was built; the live body the
        /// solve mutates is held by the job and threaded through the static steps by <c>ref</c>.
        /// </summary>
        public KinematicCharacterBody2D CharacterBodySnapshot;

        /// <summary>The character's static properties.</summary>
        public KinematicCharacterProperties2D CharacterProperties;

        /// <summary>
        /// The character's step- and slope-handling parameters, so the grounding callback can evaluate step
        /// grounding (<see cref="KinematicCharacterUtilities2D.IsGroundedOnSteps"/>) and the constrain-to-ground flag.
        /// </summary>
        public BasicStepAndSlopeHandlingParameters2D StepAndSlopeHandling;

        /// <summary>
        /// The character entity, needed by the step-grounding raycasts so they can skip the character's own body.
        /// </summary>
        public Entity CharacterEntity;

        /// <inheritdoc/>
        public void UpdateGroundingUp(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext
        )
        {
            // 2D port of Default_UpdateGroundingUp (REF/KinematicCharacterUtilities.cs:1185): grounding-up is the
            // character transform's up. Built into the snapshot's GroundingUp by the solve system from the body's
            // current rotation (MathUtilities2D.UpFromAngle), so this is a no-op for the default processor — the
            // solve sets GroundingUp on the live body before Initialize and the snapshot mirrors it.
        }

        /// <inheritdoc/>
        public bool CanCollideWithHit(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit
        )
        {
            // 2D port of the sample's CanCollideWithHit (REF/Samples~/…:220), which gated on
            // PhysicsUtilities.IsCollidable(hit.Material). The 2D substrate carries no per-hit material (BasicHit2D
            // drops it), so every real hit (a resolved package entity) is collidable; non-package hits resolve to
            // Entity.Null and are already skipped by the filter loops.
            return hit.Entity != Unity.Entities.Entity.Null;
        }

        /// <inheritdoc/>
        public bool IsGroundedOnHit(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit,
            int groundingEvaluationType
        )
        {
            return KinematicCharacterUtilities2D.Default_IsGroundedOnHit(
                ref baseContext,
                CharacterEntity,
                in CharacterBodySnapshot,
                in CharacterProperties,
                in StepAndSlopeHandling,
                in hit,
                groundingEvaluationType
            );
        }

        /// <inheritdoc/>
        public void OnMovementHit(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float2 originalVelocityDirection,
            float hitDistance
        )
        {
            // The core MoveWithCollisions calls KinematicCharacterUtilities2D.OnMovementHit directly (it needs the
            // live position/body refs). This callback is the override surface for a consumer driving the steps
            // itself; the default leaves it empty because the core already applied the default behaviour. A consumer
            // that wants custom movement-hit handling implements its own processor and drives MoveWithCollisions
            // through a chain that routes this callback.
        }

        /// <inheritdoc/>
        public void ProjectVelocityOnHits(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref float2 velocity,
            ref bool characterIsGrounded,
            ref BasicHit2D characterGroundHit,
            in Unity.Entities.DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHits,
            float2 originalVelocityDirection
        )
        {
            KinematicCharacterUtilities2D.Default_ProjectVelocityOnHits(
                ref velocity,
                ref characterIsGrounded,
                ref characterGroundHit,
                in velocityProjectionHits,
                originalVelocityDirection,
                StepAndSlopeHandling.ConstrainVelocityToGroundPlane,
                in CharacterBodySnapshot
            );
        }

        /// <inheritdoc/>
        public void OverrideDynamicHitMasses(
            ref DefaultCharacterUpdateContext2D context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterMass2D characterMass,
            ref KinematicCharacterMass2D otherMass,
            BasicHit2D hit
        )
        {
            // No mass override by default (matches the 3D sample's empty OverrideDynamicHitMasses,
            // REF/Samples~/…:283). The dynamics path that consumes the masses is chunk C4b.
        }
    }
}
