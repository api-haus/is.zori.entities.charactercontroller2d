using System;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The circle-or-box cast proxy the solve sweeps against the world. NEW in 2D — it has no 3D origin
    /// (design D1). The 3D controller casts the character's actual collider (any shape, including capsule), but
    /// the substrate's cast surface offers only <c>PhysicsQueries2D.CircleCast</c> and <c>BoxCast</c> — no
    /// capsule or arbitrary-shape cast — so the controller's <em>sensing</em> shape is restricted to a circle or
    /// an axis-aligned box even when the body's <em>world</em> collider is a capsule or polygon.
    /// <see cref="Kind"/> must be <see cref="PhysicsShape2DKind.Circle"/> or <see cref="PhysicsShape2DKind.Box"/>;
    /// the other kinds of the shared substrate enum are unsupported as a cast proxy (they have no cast surface).
    /// </summary>
    [Serializable]
    public struct KinematicCharacterColliderProxy2D : IComponentData
    {
        /// <summary>
        /// Which cast shape the solve sweeps. Only <see cref="PhysicsShape2DKind.Circle"/> and
        /// <see cref="PhysicsShape2DKind.Box"/> are supported (the substrate exposes only circle/box casts).
        /// </summary>
        public PhysicsShape2DKind Kind;

        /// <summary>
        /// Circle proxy radius (used when <see cref="Kind"/> is <see cref="PhysicsShape2DKind.Circle"/>).
        /// </summary>
        public float Radius;

        /// <summary>
        /// Box proxy full extents (used when <see cref="Kind"/> is <see cref="PhysicsShape2DKind.Box"/>).
        /// </summary>
        public float2 BoxSize;
    }
}
