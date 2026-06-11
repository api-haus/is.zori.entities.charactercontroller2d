namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The state of a character hit (enter, exit, stay). 2D port of <c>Unity.CharacterController.CharacterHitState</c>
    /// (REF/KinematicCharacterUtilities.cs:19) — identical, no dimensional content.
    /// </summary>
    public enum CharacterHitState2D
    {
        /// <summary>
        /// The hit has been entered
        /// </summary>
        Enter,

        /// <summary>
        /// The hit is being detected
        /// </summary>
        Stay,

        /// <summary>
        /// The hit has been exited
        /// </summary>
        Exit,
    }

    /// <summary>
    /// Identifier for a type of grounding evaluation, passed to <see cref="IKinematicCharacterProcessor2D{C}.IsGroundedOnHit"/>
    /// so a processor can branch on which solve phase asked. 2D port of
    /// <c>Unity.CharacterController.GroundingEvaluationType</c> (REF/KinematicCharacterUtilities.cs:38) — identical.
    /// </summary>
    public enum GroundingEvaluationType2D
    {
        /// <summary>
        /// General-purpose grounding evaluation
        /// </summary>
        Default,

        /// <summary>
        /// Grounding evaluation for the ground probing phase
        /// </summary>
        GroundProbing,

        /// <summary>
        /// Grounding evaluation for the overlap decollision phase
        /// </summary>
        OverlapDecollision,

        /// <summary>
        /// Grounding evaluation for the initial overlaps detection phase
        /// </summary>
        InitialOverlaps,

        /// <summary>
        /// Grounding evaluation for movement hits phase
        /// </summary>
        MovementHit,

        /// <summary>
        /// Grounding evaluation for stepping up hits phase
        /// </summary>
        StepUpHit,
    }
}
