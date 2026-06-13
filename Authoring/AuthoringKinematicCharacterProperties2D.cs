using System;
using UnityEngine;

namespace Zori.Entities.CharacterController2D.Authoring
{
    /// <summary>
    /// The authoring-side character properties, exposed in the inspector on
    /// <see cref="CharacterController2DAuthoring"/> and read by <c>CharacterController2DBaker</c> to build the
    /// runtime <see cref="KinematicCharacterProperties2D"/>. 2D port of
    /// <c>Unity.CharacterController.AuthoringKinematicCharacterProperties</c>
    /// (REF/KinematicCharacterComponents.cs:15). It lives in the Authoring assembly (not the Runtime one) because
    /// it is authoring-only data: the runtime component carries the baked <c>MaxGroundedSlopeDotProduct</c>, while
    /// this struct carries the author-facing <see cref="MaxGroundedSlopeAngle"/> in DEGREES (the baker converts
    /// with <c>cos(radians(angle))</c>, the 2D analogue of REF/KinematicCharacterComponents.cs:232).
    /// </summary>
    /// <remarks>
    /// The 3D origin also carries <c>CustomPhysicsBodyTags</c> and <c>InterpolatePosition</c>/<c>InterpolateRotation</c>.
    /// Those are dropped here: 2D has no per-body custom-tags consumer (the substrate defers it — its bake contract
    /// records <c>CustomPhysicsBodyTags</c> as a deferred, no-consumer component), and render-rate smoothing is the
    /// substrate's own <c>PhysicsBody2DInterpolation</c> on the body, not a per-character interpolation flag
    /// (the substrate's <c>PhysicsBody2DSmoothing</c> replaces the 3D <c>CharacterInterpolation</c>).
    /// The interpolation choice is therefore an authoring field on this component (<see cref="Interpolation"/>),
    /// baked into the body's <c>PhysicsBody2DDefinition.interpolation</c>.
    /// </remarks>
    [Serializable]
    public struct AuthoringKinematicCharacterProperties2D
    {
        /// <summary>
        /// Enables detecting ground and evaluating grounding for each hit
        /// </summary>
        [Header("Grounding")]
        [Tooltip("Enables detecting ground and evaluating grounding for each hit")]
        public bool EvaluateGrounding;

        /// <summary>
        /// Enables snapping to the ground surface below the character
        /// </summary>
        [Tooltip("Enables snapping to the ground surface below the character")]
        public bool SnapToGround;

        /// <summary>
        /// Distance to snap to ground, if SnapToGround is enabled
        /// </summary>
        [Tooltip("Distance to snap to ground, if SnapToGround is enabled")]
        public float GroundSnappingDistance;

        /// <summary>
        /// Computes a more precise distance to ground hits when the original query returned a distance of 0f due
        /// to imprecisions. Helps reduce jitter when moving against angled walls, at an extra query cost.
        /// </summary>
        [Tooltip(
            "Computes a more precise distance to ground hits when the original query returned a distance of 0f due to imprecisions. Helps reduce jitter when moving against angled walls, at an extra query cost."
        )]
        public bool EnhancedGroundPrecision;

        /// <summary>
        /// The max slope ANGLE (degrees) the character can be considered grounded on. The baker converts this to
        /// the runtime dot-product threshold <c>KinematicCharacterProperties2D.MaxGroundedSlopeDotProduct</c> via
        /// <c>cos(radians(MaxGroundedSlopeAngle))</c>.
        /// </summary>
        [Tooltip("The max slope angle (degrees) that the character can be considered grounded on")]
        [Range(0f, 90f)]
        public float MaxGroundedSlopeAngle;

        /// <summary>
        /// Enables detecting and solving movement collisions with a collider cast, based on the character's velocity
        /// </summary>
        [Header("Collisions")]
        [Tooltip(
            "Enables detecting and solving movement collisions with a collider cast, based on the character's velocity"
        )]
        public bool DetectMovementCollisions;

        /// <summary>
        /// Enables detecting and solving overlaps
        /// </summary>
        [Tooltip("Enables detecting and solving overlaps")]
        public bool DecollideFromOverlaps;

        /// <summary>
        /// Enables an extra physics check to project velocity on initial overlaps before the character moves, at a
        /// performance cost. Helps with tunneling when a rotation changes the detected collisions.
        /// </summary>
        [Tooltip(
            "Enables an extra physics check to project velocity on initial overlaps before the character moves, at a performance cost. Helps with tunneling when a rotation changes the detected collisions."
        )]
        public bool ProjectVelocityOnInitialOverlaps;

        /// <summary>
        /// The maximum amount of times per frame the character should try to cast its collider for detecting hits
        /// </summary>
        [Tooltip(
            "The maximum amount of times per frame that the character should try to cast its collider for detecting hits"
        )]
        public byte MaxContinuousCollisionsIterations;

        /// <summary>
        /// The maximum amount of times per frame the character should try to decollide itself from overlaps
        /// </summary>
        [Tooltip(
            "The maximum amount of times per frame that the character should try to decollide itself from overlaps"
        )]
        public byte MaxOverlapDecollisionIterations;

        /// <summary>
        /// Whether the remaining move distance should be reset to zero when the character exceeds the maximum
        /// collision iterations
        /// </summary>
        [Tooltip(
            "Whether the remaining move distance should be reset to zero when the character exceeds the maximum collision iterations"
        )]
        public bool DiscardMovementWhenExceedMaxIterations;

        /// <summary>
        /// Whether the velocity should be reset to zero when the character exceeds the maximum collision iterations
        /// </summary>
        [Tooltip(
            "Whether the velocity should be reset to zero when the character exceeds the maximum collision iterations"
        )]
        public bool KillVelocityWhenExceedMaxIterations;

        /// <summary>
        /// Enables a collider cast to detect obstructions when being moved by a parent body, instead of carrying
        /// the character transform along
        /// </summary>
        [Tooltip(
            "Enables a collider cast to detect obstructions when being moved by a parent body, instead of carrying the character transform along"
        )]
        public bool DetectObstructionsForParentBodyMovement;

        /// <summary>
        /// Enables physics interactions (push and be pushed) with other dynamic bodies
        /// </summary>
        [Header("Dynamics")]
        [Tooltip("Enables physics interactions (push and be pushed) with other dynamic bodies")]
        public bool SimulateDynamicBody;

        /// <summary>
        /// The mass used to simulate dynamic body interactions
        /// </summary>
        [Tooltip("The mass used to simulate dynamic body interactions")]
        public float Mass;

        /// <summary>
        /// Gets a sensible default set of authoring parameters (matching the 3D reference's defaults,
        /// REF/KinematicCharacterComponents.cs:119, minus the dropped tags/interpolation fields).
        /// </summary>
        /// <returns> The default authoring character properties </returns>
        public static AuthoringKinematicCharacterProperties2D GetDefault()
        {
            return new AuthoringKinematicCharacterProperties2D
            {
                // Grounding
                EvaluateGrounding = true,
                SnapToGround = true,
                GroundSnappingDistance = 0.5f,
                EnhancedGroundPrecision = false,
                MaxGroundedSlopeAngle = 60f,

                // Collisions
                DetectMovementCollisions = true,
                DecollideFromOverlaps = true,
                ProjectVelocityOnInitialOverlaps = false,
                MaxContinuousCollisionsIterations = 8,
                MaxOverlapDecollisionIterations = 2,
                DiscardMovementWhenExceedMaxIterations = true,
                KillVelocityWhenExceedMaxIterations = true,
                DetectObstructionsForParentBodyMovement = false,

                // Dynamics
                SimulateDynamicBody = true,
                Mass = 1f,
            };
        }
    }
}
