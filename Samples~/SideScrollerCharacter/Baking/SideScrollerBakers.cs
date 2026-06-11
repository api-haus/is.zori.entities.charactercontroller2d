using Unity.Entities;
using Zori.Entities.CharacterController2D.Samples;

namespace Zori.Entities.CharacterController2D.Samples.Baking
{
    /// <summary>
    /// Bakes the side-scroller sample's marker authoring components into their runtime tags/components. Editor-only
    /// (<c>includePlatforms: ["Editor"]</c>, like every baking assembly in the family), so the authoring references
    /// never reach a player build. These are deliberately tiny — they add only the sample's own tags; the heavy
    /// character archetype is emitted by the package's <c>CharacterController2DBaker</c> from the
    /// <c>CharacterController2DAuthoring</c> on the same GameObject, and the body/shape by the substrate bakers from
    /// the <c>PhysicsBody2DAuthoring</c>/<c>PhysicsShape2DAuthoring</c>. The sample bakers run alongside those on the
    /// same entity (each authoring component bakes independently), so the entity converges on the union archetype.
    /// </summary>
    public sealed class SideScrollerSampleConfigBaker : Baker<SideScrollerSampleConfigAuthoring>
    {
        public override void Bake(SideScrollerSampleConfigAuthoring authoring)
        {
            // A pure data singleton — no transform usage needed.
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent<SideScrollerSampleConfig>(entity);
        }
    }

    /// <summary>
    /// Bakes <see cref="SideScrollerCharacterAuthoring"/> into the sample's character tag + an empty control
    /// component. Runs on the same entity as the package's <c>CharacterController2DBaker</c> (both author components
    /// off the same GameObject), so the character ends up with the full controller archetype PLUS the sample's
    /// control/tag. It does NOT add the package's <c>DefaultCharacterController2DTag</c>, so only the sample's solve
    /// drives this character.
    /// </summary>
    public sealed class SideScrollerCharacterBaker : Baker<SideScrollerCharacterAuthoring>
    {
        public override void Bake(SideScrollerCharacterAuthoring authoring)
        {
            // Dynamic: the same TransformUsageFlags the controller baker requests, so the requests merge to one entity.
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<SideScrollerCharacterTag>(entity);
            AddComponent(entity, new CharacterControl2D());
        }
    }

    /// <summary>Bakes <see cref="SideScrollerPushableAuthoring"/> into the <see cref="SideScrollerPushable"/> tag.</summary>
    public sealed class SideScrollerPushableBaker : Baker<SideScrollerPushableAuthoring>
    {
        public override void Bake(SideScrollerPushableAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<SideScrollerPushable>(entity);
        }
    }

    /// <summary>
    /// Bakes <see cref="SideScrollerMovingPlatformAuthoring"/> into the <see cref="SideScrollerMovingPlatform"/>
    /// component carrying the authored travel half-extent + speed. The <see cref="TrackedTransform2D"/> and the
    /// command buffer are added at runtime (no tracked-transform baker in the package yet — the C4b-flagged addition);
    /// the home X / fixed Y are captured at first update from the baked pose.
    /// </summary>
    public sealed class SideScrollerMovingPlatformBaker : Baker<SideScrollerMovingPlatformAuthoring>
    {
        public override void Bake(SideScrollerMovingPlatformAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(
                entity,
                new SideScrollerMovingPlatform
                {
                    HomeCaptured = false,
                    TravelHalfExtentX = authoring.TravelHalfExtentX,
                    SpeedX = authoring.SpeedX,
                });
        }
    }
}
