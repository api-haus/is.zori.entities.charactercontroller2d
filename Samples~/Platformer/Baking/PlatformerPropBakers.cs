using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.CharacterController2D.Samples.Platformer;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Baking
{
    /// <summary>
    /// Bakes the Platformer sample's FEATURE-PROP authoring components into their runtime markers/components —
    /// moving platforms, pushable crates, wind zones, friction surfaces, teleporters, and rope anchors. Editor-only
    /// (<c>includePlatforms: ["Editor"]</c>, like every baking assembly in the family), so the authoring references
    /// never reach a player build.
    ///
    /// <para>These bakers are deliberately tiny: each adds only its prop's own marker/data. The substrate body / shape
    /// (kinematic platform body, sensor shape, dynamic crate body, static surface collider) is emitted by the substrate
    /// bakers from the <c>PhysicsBody2DAuthoring</c> / <c>PhysicsShape2DAuthoring</c> on the SAME GameObject — each
    /// authoring component bakes independently onto one entity, so the entity converges on the union archetype. The
    /// CHARACTER baker is a separate file (<c>PlatformerCharacterBaker.cs</c>, P3); these prop bakers live here to
    /// avoid a file collision.</para>
    /// </summary>
    public sealed class MovingPlatform2DBaker : Baker<MovingPlatform2DAuthoring>
    {
        public override void Bake(MovingPlatform2DAuthoring authoring)
        {
            // Dynamic: a kinematic platform body is moved each step; the same TransformUsageFlags the substrate body
            // baker requests, so the requests merge to one entity.
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new MovingPlatform2D
                {
                    HomeCaptured = false,
                    TravelHalfExtent = new float2(authoring.TravelHalfExtent.x, authoring.TravelHalfExtent.y),
                    Speed = authoring.Speed,
                    Phase = 0f,
                }
            );
        }
    }

    /// <summary>Bakes <see cref="Pushable2DAuthoring"/> into the <see cref="Pushable2D"/> tag.</summary>
    public sealed class Pushable2DBaker : Baker<Pushable2DAuthoring>
    {
        public override void Bake(Pushable2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<Pushable2D>(entity);
        }
    }

    /// <summary>Bakes <see cref="WindZone2DAuthoring"/> into a <see cref="WindZone2D"/> carrying the authored force.</summary>
    public sealed class WindZone2DBaker : Baker<WindZone2DAuthoring>
    {
        public override void Bake(WindZone2DAuthoring authoring)
        {
            // A sensor body is moved/positioned in the world but never solved against; Dynamic matches the substrate
            // shape baker's request so both author onto one entity.
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new WindZone2D { Force = new float2(authoring.Force.x, authoring.Force.y) });
        }
    }

    /// <summary>Bakes <see cref="FrictionModifier2DAuthoring"/> into a <see cref="FrictionModifier2D"/>.</summary>
    public sealed class FrictionModifier2DBaker : Baker<FrictionModifier2DAuthoring>
    {
        public override void Bake(FrictionModifier2DAuthoring authoring)
        {
            // A surface segment is a static collider; Renderable is enough (no per-step transform write).
            var entity = GetEntity(TransformUsageFlags.Renderable);
            AddComponent(entity, new FrictionModifier2D { Friction = authoring.Friction });
        }
    }

    /// <summary>
    /// Bakes <see cref="Teleporter2DAuthoring"/> into a <see cref="Teleporter2D"/>, resolving the authored destination
    /// GameObject to its entity. A null destination bakes <see cref="Entity.Null"/>, which
    /// <c>TeleporterSystem2D</c> treats as a no-op (it never teleports to the origin).
    /// </summary>
    public sealed class Teleporter2DBaker : Baker<Teleporter2DAuthoring>
    {
        public override void Bake(Teleporter2DAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            // The destination is a position marker; Renderable captures its LocalToWorld without making it dynamic.
            Entity destination =
                authoring.Destination != null
                    ? GetEntity(authoring.Destination, TransformUsageFlags.Renderable)
                    : Entity.Null;
            AddComponent(entity, new Teleporter2D { Destination = destination });
        }
    }

    /// <summary>Bakes <see cref="RopeAnchor2DAuthoring"/> into the <see cref="RopeAnchor2D"/> tag.</summary>
    public sealed class RopeAnchor2DBaker : Baker<RopeAnchor2DAuthoring>
    {
        public override void Bake(RopeAnchor2DAuthoring authoring)
        {
            // A static anchor whose world pivot the grab query reads from LocalToWorld; Renderable captures it.
            var entity = GetEntity(TransformUsageFlags.Renderable);
            AddComponent<RopeAnchor2D>(entity);
        }
    }
}
