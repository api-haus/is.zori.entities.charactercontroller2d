using System;
using System.Runtime.CompilerServices;
using Unity.Entities;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Component holding general (mostly static) properties for a kinematic 2D character. 2D port of
    /// <c>Unity.CharacterController.KinematicCharacterProperties</c> (REF/KinematicCharacterComponents.cs:157).
    /// Every field is scalar or bool, so the port is verbatim — there is no <c>float3</c>/<c>quaternion</c> to
    /// reduce. <c>MaxGroundedSlopeDotProduct</c> stays a dot-product threshold (the authored degrees are
    /// converted at bake by <c>MathUtilities2D.AngleRadiansToDotRatio</c>, the 2D analogue of
    /// REF/KinematicCharacterComponents.cs:232); the authoring → runtime constructor lives in the baking layer.
    /// </summary>
    [Serializable]
    public struct KinematicCharacterProperties2D : IComponentData
    {
        /// <summary>
        /// Enables detecting ground and evaluating grounding for each hit
        /// </summary>
        public bool EvaluateGrounding;

        /// <summary>
        /// Enables snapping to the ground surface below the character
        /// </summary>
        public bool SnapToGround;

        /// <summary>
        /// Distance to snap to ground, if SnapToGround is enabled
        /// </summary>
        public float GroundSnappingDistance;

        /// <summary>
        /// Computes a more precise distance to ground hits when the original query returned a distance of 0f due
        /// to imprecisions. Helps reduce jitter when moving against angled walls, at an extra query cost.
        /// </summary>
        public bool EnhancedGroundPrecision;

        /// <summary>
        /// The max slope (as a dot product of grounding-up against the surface normal) the character can be
        /// considered grounded on. Authored as a degrees angle and converted at bake.
        /// </summary>
        public float MaxGroundedSlopeDotProduct;

        /// <summary>
        /// Enables detecting and solving movement collisions with a collider cast, based on the character's velocity
        /// </summary>
        public bool DetectMovementCollisions;

        /// <summary>
        /// Enables detecting and solving overlaps
        /// </summary>
        public bool DecollideFromOverlaps;

        /// <summary>
        /// Enables doing an extra physics check to project velocity on initial overlaps before the character
        /// moves, at a performance cost. Helps with tunneling when a rotation changes the detected collisions.
        /// </summary>
        public bool ProjectVelocityOnInitialOverlaps;

        /// <summary>
        /// The maximum amount of times per frame that the character should try to cast its collider for detecting hits
        /// </summary>
        public byte MaxContinuousCollisionsIterations;

        /// <summary>
        /// The maximum amount of times per frame that the character should try to decollide itself from overlaps
        /// </summary>
        public byte MaxOverlapDecollisionIterations;

        /// <summary>
        /// Whether the remaining move distance should be reset to zero when the character exceeds the maximum
        /// collision iterations
        /// </summary>
        public bool DiscardMovementWhenExceedMaxIterations;

        /// <summary>
        /// Whether the velocity should be reset to zero when the character exceeds the maximum collision iterations
        /// </summary>
        public bool KillVelocityWhenExceedMaxIterations;

        /// <summary>
        /// Enables a collider cast to detect obstructions when being moved by a parent body, instead of simply
        /// carrying the character transform along
        /// </summary>
        public bool DetectObstructionsForParentBodyMovement;

        /// <summary>
        /// Enables physics interactions (push and be pushed) with other dynamic bodies
        /// </summary>
        public bool SimulateDynamicBody;

        /// <summary>
        /// The mass used to simulate dynamic body interactions
        /// </summary>
        public float Mass;

        /// <summary>
        /// Whether dynamic rigidbody collisions should be ignored with the current character properties
        /// </summary>
        /// <returns> True if the character should ignore dynamic bodies based on these properties </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ShouldIgnoreDynamicBodies()
        {
            return !SimulateDynamicBody;
        }
    }
}
