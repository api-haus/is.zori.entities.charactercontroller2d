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
