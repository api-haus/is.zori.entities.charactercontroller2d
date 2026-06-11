using System;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The circle/box/capsule cast proxy the solve sweeps against the world. NEW in 2D — it has no 3D origin
    /// (design D1). The 3D controller casts the character's actual collider; the substrate's cast surface offers
    /// <c>PhysicsQueries2D.CircleCast</c>, <c>BoxCast</c>, and <c>CapsuleCast</c> (no arbitrary-shape cast), so
    /// the controller's <em>sensing</em> shape is one of those three even when the body's <em>world</em> collider
    /// is a polygon or edge. <see cref="Kind"/> must be <see cref="PhysicsShape2DKind.Circle"/>,
    /// <see cref="PhysicsShape2DKind.Box"/>, or <see cref="PhysicsShape2DKind.Capsule"/>; the variable-length
    /// kinds (Polygon, Edge) of the shared substrate enum are unsupported as a cast proxy (they have no
    /// fixed-geometry cast surface).
    /// </summary>
    [Serializable]
    public struct KinematicCharacterColliderProxy2D : IComponentData
    {
        /// <summary>
        /// Which cast shape the solve sweeps. Only <see cref="PhysicsShape2DKind.Circle"/>,
        /// <see cref="PhysicsShape2DKind.Box"/>, and <see cref="PhysicsShape2DKind.Capsule"/> are supported (the
        /// substrate exposes circle/box/capsule casts).
        /// </summary>
        public PhysicsShape2DKind Kind;

        /// <summary>
        /// Circle proxy radius, OR the capsule end-cap radius (used when <see cref="Kind"/> is
        /// <see cref="PhysicsShape2DKind.Circle"/> or <see cref="PhysicsShape2DKind.Capsule"/>).
        /// </summary>
        public float Radius;

        /// <summary>
        /// Box proxy full extents (used when <see cref="Kind"/> is <see cref="PhysicsShape2DKind.Box"/>).
        /// </summary>
        public float2 BoxSize;

        /// <summary>
        /// Capsule proxy end-cap centers in the body's local frame (used when <see cref="Kind"/> is
        /// <see cref="PhysicsShape2DKind.Capsule"/>). These are the SAME two local centers the baked world
        /// <c>PhysicsShape2D</c> capsule carries (<c>capsuleCenter1</c>/<c>capsuleCenter2</c>), so the cast
        /// geometry the solve sweeps and the world body's shape describe one capsule. The solve translates both
        /// by the character's centre before casting; rotation is not applied (the side-scroller character does
        /// not rotate, matching the box proxy's axis-aligned sweep).
        /// </summary>
        public float2 CapsuleCenter1;
        public float2 CapsuleCenter2;
    }
}
