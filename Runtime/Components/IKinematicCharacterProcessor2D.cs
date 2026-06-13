using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Interface implemented by user structs passed to the controller's update steps to customize internal solve
    /// logic. 2D port of <c>Unity.CharacterController.IKinematicCharacterProcessor&lt;C&gt;</c>
    /// (REF/IKinematicCharacterProcessor.cs:12) — the same six callbacks, with <c>float3</c> reduced to
    /// <c>float2</c>, the 3D <c>BasicHit</c>/<c>KinematicCharacterHit</c>/<c>KinematicVelocityProjectionHit</c>
    /// replaced by their 2D ports, and the 3D <c>PhysicsMass</c> replaced by <see cref="KinematicCharacterMass2D"/>
    /// (scalar inertia). The 3D <c>KinematicCharacterUpdateContext</c> base-context parameter becomes
    /// <see cref="KinematicCharacterUpdateContext2D"/>, which this interface forward-references.
    /// </summary>
    /// <typeparam name="C"> The type of the user "context" struct created by the consumer </typeparam>
    public interface IKinematicCharacterProcessor2D<C>
        where C : unmanaged
    {
        /// <summary>
        /// Requests that the grounding-up direction be updated.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        void UpdateGroundingUp(ref C context, ref KinematicCharacterUpdateContext2D baseContext);

        /// <summary>
        /// Determines whether a hit can be collided with.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <returns> True if the hit can be collided with </returns>
        bool CanCollideWithHit(ref C context, ref KinematicCharacterUpdateContext2D baseContext, in BasicHit2D hit);

        /// <summary>
        /// Determines whether the character can be grounded on the hit.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <param name="groundingEvaluationType"> Which solve phase is asking (a <see cref="GroundingEvaluationType2D"/> cast to int) </param>
        /// <returns> True if the character is grounded on the hit </returns>
        bool IsGroundedOnHit(
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit,
            int groundingEvaluationType
        );

        /// <summary>
        /// Determines what happens when the character detects a hit during its movement phase.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        /// <param name="hit"> The evaluated hit </param>
        /// <param name="remainingMovementDirection"> The direction of the movement vector that remains to be processed </param>
        /// <param name="remainingMovementLength"> The magnitude of the movement vector that remains to be processed </param>
        /// <param name="originalVelocityDirection"> The original movement direction before any projection </param>
        /// <param name="hitDistance"> The distance of the detected hit </param>
        void OnMovementHit(
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float2 originalVelocityDirection,
            float hitDistance
        );

        /// <summary>
        /// Requests that the character velocity be projected on the hits detected so far in the update.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        /// <param name="velocity"> The character velocity to project </param>
        /// <param name="characterIsGrounded"> Whether the character is grounded </param>
        /// <param name="characterGroundHit"> The character's current effective ground hit </param>
        /// <param name="velocityProjectionHits"> The hits detected so far during the update </param>
        /// <param name="originalVelocityDirection"> The original velocity direction before any projection </param>
        void ProjectVelocityOnHits(
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref float2 velocity,
            ref bool characterIsGrounded,
            ref BasicHit2D characterGroundHit,
            in DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHits,
            float2 originalVelocityDirection
        );

        /// <summary>
        /// Provides an opportunity to modify the masses used to solve impulses between the character and a hit body.
        /// </summary>
        /// <param name="context"> The user context struct </param>
        /// <param name="baseContext"> The built-in context struct </param>
        /// <param name="characterMass"> The mass of the character </param>
        /// <param name="otherMass"> The mass of the other body that was hit </param>
        /// <param name="hit"> The evaluated hit with the dynamic body </param>
        void OverrideDynamicHitMasses(
            ref C context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterMass2D characterMass,
            ref KinematicCharacterMass2D otherMass,
            BasicHit2D hit
        );
    }
}
