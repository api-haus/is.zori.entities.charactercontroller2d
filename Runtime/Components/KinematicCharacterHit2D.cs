using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Data representing a detected character hit, recorded each step for the stateful-hit diff and the dynamics
    /// path. 2D port of <c>Unity.CharacterController.KinematicCharacterHit</c>
    /// (REF/KinematicCharacterComponents.cs:425): <see cref="Position"/>, <see cref="Normal"/>, and the two
    /// before/after velocities reduce from <c>float3</c> to <c>float2</c>; the 3D <c>ColliderKey</c> and
    /// <c>Physics.Material</c> are dropped (the 2D substrate gives one shape per hit and no per-hit material).
    /// <see cref="RigidBodyIndex"/> is retained for parity, left at <c>-1</c> when the substrate supplies none —
    /// the <see cref="Entity"/> is the 2D body identity.
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicCharacterHit2D : IBufferElementData
    {
        /// <summary>
        /// Hit entity
        /// </summary>
        public Entity Entity;

        /// <summary>
        /// Hit rigidbody index (retained for parity; <c>-1</c> when the substrate supplies none)
        /// </summary>
        public int RigidBodyIndex;

        /// <summary>
        /// Hit point (world space)
        /// </summary>
        public float2 Position;

        /// <summary>
        /// Hit normal (world space)
        /// </summary>
        public float2 Normal;

        /// <summary>
        /// Whether the character was grounded when the hit was detected
        /// </summary>
        public bool WasCharacterGroundedOnHitEnter;

        /// <summary>
        /// Whether the character would consider itself grounded on this hit
        /// </summary>
        public bool IsGroundedOnHit;

        /// <summary>
        /// The character's velocity before velocity projection on this hit
        /// </summary>
        public float2 CharacterVelocityBeforeHit;

        /// <summary>
        /// The character's velocity after velocity projection on this hit
        /// </summary>
        public float2 CharacterVelocityAfterHit;
    }
}
