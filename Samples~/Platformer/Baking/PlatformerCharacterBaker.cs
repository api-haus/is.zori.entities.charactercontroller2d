using Unity.Entities;
using Zori.Entities.CharacterController2D.Samples.Platformer;

namespace Zori.Entities.CharacterController2D.Samples.Platformer.Baking
{
    /// <summary>
    /// Bakes <see cref="PlatformerCharacterAuthoring"/> into the Platformer character's sample-specific components,
    /// running on the SAME entity as the package's <c>CharacterController2DBaker</c> (both author off the same
    /// GameObject — the <c>[RequireComponent(typeof(CharacterController2DAuthoring))]</c> guarantees the sibling), so
    /// the character converges on the full controller archetype (the character components + buffers, the CAPSULE cast
    /// proxy, the kinematic <c>PhysicsBody2DDefinition</c>, the capsule <c>PhysicsShape2D</c> world shape, and the
    /// <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> move queue — all emitted by the controller baker from the
    /// capsule proxy choice on the sibling authoring) PLUS the Platformer surface this baker adds. It does NOT add the
    /// package's <c>DefaultCharacterController2DTag</c>, so only the Platformer sample's solve drives this character.
    /// </summary>
    /// <remarks>
    /// Editor-only assembly (<c>includePlatforms: ["Editor"]</c>, like every baking assembly in the family), so the
    /// <c>UnityEngine.*</c> authoring reference never reaches a player build. This file is character-only on purpose:
    /// the feature-prop bakers (platform, pushable, wind zone, friction, teleporter, rope anchor) live in their own
    /// baker file(s), so naming this <c>PlatformerCharacterBaker.cs</c> keeps the two apart.
    /// </remarks>
    public sealed class PlatformerCharacterBaker : Baker<PlatformerCharacterAuthoring>
    {
        public override void Bake(PlatformerCharacterAuthoring authoring)
        {
            // Dynamic: the same TransformUsageFlags the controller baker requests off the same GameObject, so the
            // two bake requests merge to one entity (the SideScroller pattern).
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            // Identity + intent. The tag is the inert-on-import gate the sample systems RequireForUpdate on; the
            // control component is the per-frame intent the control system fills and the solve consumes (no intent
            // until the control system writes one), mirroring the SideScroller baking an empty control. The jump
            // buffer stamp starts at NegativeInfinity (NOT the struct-default 0) so the very first grounded step at
            // simulation time ≈ 0 does NOT read a phantom buffered jump (0 − 0 ≤ 0.15 would be a spurious window hit).
            AddComponent<PlatformerCharacterTag>(entity);
            AddComponent(entity, new PlatformerCharacterControl2D { JumpBufferElapsedTime = float.NegativeInfinity });

            // Per-character movement + rope tuning, read per-entity in the solve (the coordinator correction — NOT a
            // scene config singleton, NOT const-on-the-system). The LayerMask authors a 1<<layer bitfield; the
            // substrate sets a shape's categoryBits to 1<<gameObject.layer, so casting the mask value to the 64-bit
            // category mask matches an anchor on layer N to a mask with bit N set. Cast through uint first so a mask
            // with the high (sign) bit set does not sign-extend into the upper 32 bits.
            AddComponent(
                entity,
                new PlatformerCharacterTuning2D
                {
                    GravityMagnitude = authoring.GravityMagnitude,
                    GroundMoveSpeed = authoring.GroundMoveSpeed,
                    GroundedMovementSharpness = authoring.GroundedMovementSharpness,
                    AirMoveSpeed = authoring.AirMoveSpeed,
                    AirAcceleration = authoring.AirAcceleration,
                    JumpSpeed = authoring.JumpSpeed,
                    JumpBufferTime = authoring.JumpBufferTime,
                    RopeLength = authoring.RopeLength,
                    RopeAnchorSearchRadius = authoring.RopeAnchorSearchRadius,
                    RopeSwingMaxSpeed = authoring.RopeSwingMaxSpeed,
                    RopeSwingAcceleration = authoring.RopeSwingAcceleration,
                    RopeSwingDrag = authoring.RopeSwingDrag,
                    RopeAnchorLayerMask = (ulong)(uint)authoring.RopeAnchorLayerMask.value,
                    FallRespawnThresholdY = authoring.FallRespawnThresholdY,
                    RespawnHeightOffset = authoring.RespawnHeightOffset,
                }
            );

            // The persistent stance state, initialized to GroundMove — the character starts on the ground line; the
            // solve flips it to AirMove when it leaves the ground and to RopeSwing on a grab near an anchor.
            AddComponent(entity, new PlatformerCharacterState2D { Stance = PlatformerStance2D.GroundMove });

            // The active-rope params, defaulted (Anchor = Entity.Null, no rope) — valid only once the AirMove ->
            // RopeSwing transition writes a grabbed anchor into it. Baked once so the RopeSwing block reads/writes an
            // existing component, with no runtime structural change on first grab.
            AddComponent(entity, new RopeSwingState2D());

            // The last-safe-point state, defaulted (HasPoint = false — nowhere to respawn to until the character
            // stands somewhere safe). PlatformerRespawnSystem records the safe pose every stable grounded step and
            // teleports the character back to it on a fall.
            AddComponent(entity, new LastSafePoint2D());
        }
    }
}
