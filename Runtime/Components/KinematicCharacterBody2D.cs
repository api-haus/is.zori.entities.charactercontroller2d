using System;
using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// A component holding the transient data (data modified during the character update) of a 2D character.
    /// 2D port of <c>Unity.CharacterController.KinematicCharacterBody</c>
    /// (REF/KinematicCharacterComponents.cs:262). Velocities, the grounding-up axis, the parent anchor, and the
    /// parent velocity reduce from <c>float3</c> to <c>float2</c>; the parent-relative rotation reduces from a
    /// <c>quaternion</c> to a single <c>float</c> z-angle in radians (the package angular convention, default 0
    /// = identity); the ground hit becomes a <see cref="BasicHit2D"/>. It is enableable so the solve can skip a
    /// character cheaply, exactly as the 3D component is.
    /// </summary>
    [Serializable]
    public struct KinematicCharacterBody2D : IComponentData, IEnableableComponent
    {
        /// <summary>
        /// Whether the character is currently grounded or not
        /// </summary>
        public bool IsGrounded;

        /// <summary>
        /// The character's velocity relative to its assigned parent's velocity (if any)
        /// </summary>
        public float2 RelativeVelocity;

        /// <summary>
        /// The character's parent entity
        /// </summary>
        public Entity ParentEntity;

        /// <summary>
        /// The character's anchor point to its parent, expressed in the parent's local space
        /// </summary>
        public float2 ParentLocalAnchorPoint;

        // The following data is fully reset at the beginning of the character update, or recalculated during
        // the update. It typically needs no network sync unless accessed before the character update.

        /// <summary>
        /// The character's grounding up direction
        /// </summary>
        public float2 GroundingUp;

        /// <summary>
        /// The character's detected ground hit
        /// </summary>
        public BasicHit2D GroundHit;

        /// <summary>
        /// The calculated velocity of the character's parent
        /// </summary>
        public float2 ParentVelocity;

        /// <summary>
        /// The previous parent entity
        /// </summary>
        public Entity PreviousParentEntity;

        /// <summary>
        /// The rotation (z-angle, radians) resulting from the parent's movement over the latest update.
        /// Default 0 is the identity rotation (the 2D analogue of the 3D <c>quaternion.identity</c>).
        /// </summary>
        public float RotationFromParent;

        /// <summary>
        /// The last known delta time of the character update
        /// </summary>
        public float LastPhysicsUpdateDeltaTime;

        /// <summary>
        /// Whether the character was considered grounded at the beginning of the update, before ground is detected
        /// </summary>
        public bool WasGroundedBeforeCharacterUpdate;

        /// <summary>
        /// Set true when step-up lifts the character onto a step, and consumed by the next update's grounding step
        /// to suppress the downward ground-snap while the character's centre still overhangs the lower surface it
        /// stepped up from. In 3D the step-up writes the over-the-step position instantaneously and the next frame's
        /// collider-cast down returns the step top as the nearest opposing hit, so the character stays on the step.
        /// In 2D the move is the substrate's swept <c>MovePosition</c> applied next frame, and a box
        /// proxy whose edge is flush against the step's vertical face registers the step only as a zero-fraction
        /// overlap (no opposing top-surface normal) — so the next frame's grounding finds the lower floor as the
        /// closest opposing hit and the snap would yank the character back down onto it. This flag is the localized
        /// 2D bridge: it holds the snap until the centre clears the step edge and the step top is cleanly grounded
        /// (the grounding step clears it then). It is persisted state (not reset in <c>Update_Initialize</c>).
        /// </summary>
        public bool SuppressGroundSnappingUntilSteppedClear;

        /// <summary>
        /// Returns a sensible default for this component, grounding-up pointing along world +Y.
        /// </summary>
        /// <returns> The default KinematicCharacterBody2D </returns>
        public static KinematicCharacterBody2D GetDefault()
        {
            return new KinematicCharacterBody2D
            {
                IsGrounded = default,
                RelativeVelocity = default,
                ParentEntity = default,
                ParentLocalAnchorPoint = default,

                GroundingUp = up().xy,
                GroundHit = default,
                ParentVelocity = default,
                PreviousParentEntity = default,
                RotationFromParent = 0f,
                LastPhysicsUpdateDeltaTime = 0f,
                WasGroundedBeforeCharacterUpdate = default,
                SuppressGroundSnappingUntilSteppedClear = default,
            };
        }

        /// <summary>
        /// Whether the character has become grounded on this frame
        /// </summary>
        /// <returns> True if the character has become grounded </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeGrounded()
        {
            return !WasGroundedBeforeCharacterUpdate && IsGrounded;
        }

        /// <summary>
        /// Whether the character has become ungrounded on this frame
        /// </summary>
        /// <returns> True if the character has become ungrounded </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeUngrounded()
        {
            return WasGroundedBeforeCharacterUpdate && !IsGrounded;
        }
    }
}
