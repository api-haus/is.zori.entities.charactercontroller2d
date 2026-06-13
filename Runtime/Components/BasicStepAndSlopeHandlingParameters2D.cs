using System;
using Unity.Entities;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Parameters for the controller's step- and slope-handling behaviour. 2D port of
    /// <c>Unity.CharacterController.BasicStepAndSlopeHandlingParameters</c>
    /// (REF/BasicStepAndSlopeHandlingParameters.cs:9): every field is scalar or bool, so the port is verbatim
    /// with no dimensional reduction. The 3D reference embeds this as a plain serializable struct inside the
    /// sample's component; the 2D port makes it an <see cref="IComponentData"/> so the baker can add it directly
    /// to the character entity. The <c>[Header]</c>/<c>[Tooltip]</c> attributes are kept for
    /// the inspector when an authoring component exposes the struct.
    /// </summary>
    [Serializable]
    public struct BasicStepAndSlopeHandlingParameters2D : IComponentData
    {
        /// <summary>
        /// Whether step handling logic is enabled
        /// </summary>
        [UnityEngine.Header("Step Handling")]
        [UnityEngine.Tooltip("Whether step handling logic is enabled")]
        public bool StepHandling;

        /// <summary>
        /// Max height that the character can step on
        /// </summary>
        [UnityEngine.Tooltip("Max height that the character can step on")]
        public float MaxStepHeight;

        /// <summary>
        /// Horizontal offset distance of the extra downward raycasts used to detect grounding around a step
        /// </summary>
        [UnityEngine.Tooltip(
            "Horizontal offset distance of extra downwards raycasts used to detect grounding around a step"
        )]
        public float ExtraStepChecksDistance;

        /// <summary>
        /// Character width used to determine grounding for steps. For a box this should be the maximum box width;
        /// for a circle, twice the radius. It guards against a round-based shape being grounded on a step one
        /// frame and pushed past its max step height the next.
        /// </summary>
        [UnityEngine.Tooltip(
            "Character width used to determine grounding for steps. For a circle this should be 2x radius, and for a box it should be maximum box width."
        )]
        public float CharacterWidthForStepGroundingCheck;

        /// <summary>
        /// Whether to cancel grounding when the character is moving off a ledge, so it does not snap back onto
        /// the ledge as it moves off
        /// </summary>
        [UnityEngine.Header("Slope Changes")]
        [UnityEngine.Tooltip("Whether or not to cancel grounding when the character is moving off a ledge.")]
        public bool PreventGroundingWhenMovingTowardsNoGrounding;

        /// <summary>
        /// Whether the character has a max slope change it can stay grounded on
        /// </summary>
        [UnityEngine.Tooltip("Whether or not the character has a max slope change that it can stay grounded on")]
        public bool HasMaxDownwardSlopeChangeAngle;

        /// <summary>
        /// Max slope change (degrees) the character can stay grounded on
        /// </summary>
        [UnityEngine.Tooltip("Max slope change that the character can stay grounded on (degrees)")]
        [UnityEngine.Range(0f, 180f)]
        public float MaxDownwardSlopeChangeAngle;

        /// <summary>
        /// Whether to constrain the character velocity to the ground plane when it hits a non-grounded slope
        /// </summary>
        [UnityEngine.Header("Misc")]
        [UnityEngine.Tooltip(
            "Whether or not to constrain the character velocity to ground plane when it hits a non-grounded slope"
        )]
        public bool ConstrainVelocityToGroundPlane;

        /// <summary>
        /// Gets a default-initialized version of the step- and slope-handling parameters
        /// </summary>
        /// <returns> Default parameters struct </returns>
        public static BasicStepAndSlopeHandlingParameters2D GetDefault()
        {
            return new BasicStepAndSlopeHandlingParameters2D
            {
                StepHandling = false,
                MaxStepHeight = 0.5f,
                ExtraStepChecksDistance = 0.1f,
                CharacterWidthForStepGroundingCheck = 1f,

                PreventGroundingWhenMovingTowardsNoGrounding = true,
                HasMaxDownwardSlopeChangeAngle = false,
                MaxDownwardSlopeChangeAngle = 90f,

                ConstrainVelocityToGroundPlane = true,
            };
        }
    }
}
