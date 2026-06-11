using System;
using Unity.Entities;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// A stateful version of a character hit, carrying the hit's enter/stay/exit state — the
    /// <c>OnCollisionEnter/Stay/Exit</c> analogue, produced by diffing this step's character hits against last
    /// step's. 2D port of <c>Unity.CharacterController.StatefulKinematicCharacterHit</c>
    /// (REF/KinematicCharacterComponents.cs:559): <see cref="State"/> is the 2D <see cref="CharacterHitState2D"/>
    /// enum, <see cref="Hit"/> is the 2D <see cref="KinematicCharacterHit2D"/>.
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct StatefulKinematicCharacterHit2D : IBufferElementData
    {
        /// <summary>
        /// State of the hit (enter/stay/exit)
        /// </summary>
        public CharacterHitState2D State;

        /// <summary>
        /// The character hit
        /// </summary>
        public KinematicCharacterHit2D Hit;

        /// <summary>
        /// Constructs a stateful character hit from a character hit
        /// </summary>
        /// <param name="characterHit"> The character hit </param>
        public StatefulKinematicCharacterHit2D(KinematicCharacterHit2D characterHit)
        {
            State = default;
            Hit = characterHit;
        }
    }
}
