using System;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// A common hit struct for the controller's cast and overlap-reconstruction hits, the 2D port of
    /// <c>Unity.CharacterController.BasicHit</c> (REF/KinematicCharacterUtilities.cs:98). The 3D struct carried
    /// <c>float3</c> position/normal, a <c>ColliderKey</c>, and a <c>Physics.Material</c>; the 2D substrate's
    /// <see cref="PhysicsQueryHit2D"/> surfaces one shape per hit and no material or sub-collider key, so this
    /// reduction drops both — surface friction/bounce live on <see cref="PhysicsShape2D"/> and are read there if
    /// ever needed. The owning <see cref="Entity"/> is the body identity in 2D (Box2D has no stable rigidbody
    /// index across a step); <see cref="RigidBodyIndex"/> is retained for signature parity with the reference's
    /// dynamics path and is left at <c>-1</c> when the substrate does not supply one.
    /// </summary>
    [Serializable]
    public struct BasicHit2D
    {
        /// <summary>
        /// Hit entity — the owner of the hit shape's body, and the body identity used for the dynamics path.
        /// </summary>
        public Entity Entity;

        /// <summary>
        /// Hit rigidbody index, retained for parity with the reference's dynamics path. The 2D substrate
        /// identifies bodies by <see cref="Entity"/>, not an index, so this is <c>-1</c> unless a later layer
        /// fills it.
        /// </summary>
        public int RigidBodyIndex;

        /// <summary>
        /// Hit point (world space).
        /// </summary>
        public float2 Position;

        /// <summary>
        /// Hit normal (world space).
        /// </summary>
        public float2 Normal;

        /// <summary>
        /// Constructs a basic hit from a substrate query hit. A cast hit (raycast / circle-cast / box-cast)
        /// fills <see cref="PhysicsQueryHit2D.point"/> and <see cref="PhysicsQueryHit2D.normal"/>; an overlap hit
        /// leaves them zero (the substrate documents that overlaps carry no contact geometry), so a caller using
        /// this constructor for an overlap result must reconstruct the normal separately (design D2).
        /// </summary>
        /// <param name="hit"> A substrate query hit </param>
        public BasicHit2D(PhysicsQueryHit2D hit)
        {
            Entity = hit.entity;
            RigidBodyIndex = -1;
            Position = hit.point;
            Normal = hit.normal;
        }

        /// <summary>
        /// Constructs a basic hit from explicit fields, for callers that reconstruct a hit's geometry (e.g. the
        /// overlap-depenetration cast-back of design D2, where the normal comes from a separate cast).
        /// </summary>
        /// <param name="entity"> The hit entity </param>
        /// <param name="position"> The hit point (world space) </param>
        /// <param name="normal"> The hit normal (world space) </param>
        public BasicHit2D(Entity entity, float2 position, float2 normal)
        {
            Entity = entity;
            RigidBodyIndex = -1;
            Position = position;
            Normal = normal;
        }
    }
}
