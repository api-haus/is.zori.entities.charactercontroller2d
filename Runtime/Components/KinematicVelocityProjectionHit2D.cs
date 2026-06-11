using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Holds the data for a hit that participates in velocity projection — the running set of surface lines the
    /// solve projects velocity against this step. 2D port of
    /// <c>Unity.CharacterController.KinematicVelocityProjectionHit</c>
    /// (REF/KinematicCharacterComponents.cs:474): <see cref="Position"/> / <see cref="Normal"/> reduce from
    /// <c>float3</c> to <c>float2</c>; the 3D <c>ColliderKey</c> and <c>Physics.Material</c> are dropped. The
    /// three constructors mirror the reference (REF:509, REF:525, REF:542).
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicVelocityProjectionHit2D : IBufferElementData
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
        /// Whether the character would consider itself grounded on this hit
        /// </summary>
        public bool IsGroundedOnHit;

        /// <summary>
        /// Constructs the velocity-projection hit from a character hit
        /// </summary>
        /// <param name="hit"> A character hit </param>
        public KinematicVelocityProjectionHit2D(KinematicCharacterHit2D hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            Position = hit.Position;
            Normal = hit.Normal;
            IsGroundedOnHit = hit.IsGroundedOnHit;
        }

        /// <summary>
        /// Constructs the velocity-projection hit from a basic hit and grounding status
        /// </summary>
        /// <param name="hit"> A basic hit </param>
        /// <param name="isGroundedOnHit"> Whether the character is grounded on this hit </param>
        public KinematicVelocityProjectionHit2D(BasicHit2D hit, bool isGroundedOnHit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            Position = hit.Position;
            Normal = hit.Normal;
            IsGroundedOnHit = isGroundedOnHit;
        }

        /// <summary>
        /// Constructs the velocity-projection hit from a raw normal/position and grounding status
        /// </summary>
        /// <param name="normal"> The hit normal (world space) </param>
        /// <param name="position"> The hit point (world space) </param>
        /// <param name="isGroundedOnHit"> Whether the character is grounded on this hit </param>
        public KinematicVelocityProjectionHit2D(float2 normal, float2 position, bool isGroundedOnHit)
        {
            Entity = Entity.Null;
            RigidBodyIndex = -1;
            Position = position;
            Normal = normal;
            IsGroundedOnHit = isGroundedOnHit;
        }
    }
}
