using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Stores the data a character might need to read off OTHER characters during its update, snapshotted once
    /// before the solve so character ↔ character impulse exchange reads a consistent pre-step snapshot and the
    /// parallel update stays deterministic. 2D port of
    /// <c>Unity.CharacterController.StoredKinematicCharacterData</c> (REF/KinematicCharacterComponents.cs:362):
    /// the two velocities reduce from <c>float3</c> to <c>float2</c>; <see cref="SetFrom"/> is verbatim.
    /// </summary>
    [Serializable]
    public struct StoredKinematicCharacterData2D : IComponentData
    {
        /// <summary>
        /// Enables physics interactions (push and be pushed) with other dynamic bodies
        /// </summary>
        public bool SimulateDynamicBody;

        /// <summary>
        /// The mass used to simulate dynamic body interactions
        /// </summary>
        public float Mass;

        /// <summary>
        /// The character's velocity relative to its assigned parent's velocity (if any)
        /// </summary>
        public float2 RelativeVelocity;

        /// <summary>
        /// The calculated velocity of the character's parent
        /// </summary>
        public float2 ParentVelocity;

        /// <summary>
        /// Sets the data in this component from a character body and character properties component
        /// </summary>
        /// <param name="characterProperties"> A character properties component to read from </param>
        /// <param name="characterBody"> A character body component to read from </param>
        public void SetFrom(in KinematicCharacterProperties2D characterProperties, in KinematicCharacterBody2D characterBody)
        {
            SimulateDynamicBody = characterProperties.SimulateDynamicBody;
            Mass = characterProperties.Mass;
            RelativeVelocity = characterBody.RelativeVelocity;
            ParentVelocity = characterBody.ParentVelocity;
        }
    }
}
