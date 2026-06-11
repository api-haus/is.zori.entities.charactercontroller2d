using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// The Platformer sample's user-context (the <c>C</c> in <see cref="IKinematicCharacterProcessor2D{C}"/>). Unlike
    /// the SideScroller's empty <c>SideScrollerCharacterUpdateContext</c>, the Platformer carries a
    /// <see cref="ComponentLookup{T}"/> over <see cref="FrictionModifier2D"/> — the documented place a richer Phase-B
    /// platformer adds its own lookups. The GroundMove stance reads the modifier off the character's ground hit entity
    /// through this lookup to scale its move sharpness (the "different physics materials" feature), because the
    /// kinematic controller is material-blind to Box2D friction and computes its own velocity.
    ///
    /// <para>The lookup is populated from the solve system (<c>GetComponentLookup&lt;FrictionModifier2D&gt;(true)</c> in
    /// <c>OnCreate</c>, <c>.Update(ref state)</c> in <c>OnUpdate</c>) and read inside the Bursted solve job; it lives in
    /// the context, not on the processor, because the friction read happens in the control block before
    /// <c>PhysicsUpdate2D</c> rather than inside a processor callback.</para>
    /// </summary>
    public struct PlatformerCharacterUpdateContext
    {
        /// <summary>Read-only lookup over per-surface friction modifiers, keyed on a ground hit's owning entity. The
        /// GroundMove block reads it off <see cref="KinematicCharacterBody2D.GroundHit"/>'s <see cref="BasicHit2D.Entity"/>
        /// to scale the ground-move acceleration sharpness.</summary>
        public ComponentLookup<FrictionModifier2D> FrictionModifierLookup;
    }

    /// <summary>
    /// The concrete <see cref="IKinematicCharacterProcessor2D{C}"/> for the Platformer character. Its six callbacks are
    /// identical to <c>SideScrollerCharacterProcessor</c> — the proven side-scroller solve choices — differing only in
    /// the user-context type (<see cref="PlatformerCharacterUpdateContext"/>, which carries the
    /// <see cref="FrictionModifier2D"/> lookup). The three platformer stances (GroundMove / AirMove / RopeSwing) are
    /// dispatched ABOVE the processor, in the solve's velocity-control block (the <c>switch</c> in
    /// <c>PlatformerCharacterPhysicsSystem</c>); they change the velocity computed before <c>PhysicsUpdate2D</c>, not
    /// the processor's grounding / projection / mass callbacks, so one processor serves all three stances.
    ///
    /// <para><b>The six choices</b> (forwarding to the package <c>Default_*</c> core methods exactly as the SideScroller does).
    /// <list type="bullet">
    /// <item><see cref="UpdateGroundingUp"/> — no-op: the physics system sets <c>GroundingUp = +Y</c> on the live body
    /// before Initialize (a platformer's up never changes), so the callback has nothing to do.</item>
    /// <item><see cref="CanCollideWithHit"/> — accept every resolved-entity hit: the substrate carries no per-hit
    /// material to gate on, and non-package hits already resolve to <see cref="Entity.Null"/> and are filtered out.</item>
    /// <item><see cref="IsGroundedOnHit"/> — forward to the step-aware <c>Default_IsGroundedOnHit</c> so a hit grounds
    /// the character when it is within the slope limit OR is a valid step.</item>
    /// <item><see cref="OnMovementHit"/> — empty: the core <c>MoveWithCollisions</c> applies the default movement-hit
    /// behaviour directly (it needs the live position/body refs); this callback is the override seam for a consumer
    /// driving the steps itself.</item>
    /// <item><see cref="ProjectVelocityOnHits"/> — forward to <c>Default_ProjectVelocityOnHits</c>: slide along a single
    /// surface, kill velocity in a corner wedge.</item>
    /// <item><see cref="OverrideDynamicHitMasses"/> — no override: the box-push uses the authored masses as-is.</item>
    /// </list>
    /// </para>
    /// </summary>
    public struct PlatformerCharacterProcessor : IKinematicCharacterProcessor2D<PlatformerCharacterUpdateContext>
    {
        /// <summary>Read-only snapshot of the character body, taken at the start of the solve (the C#-9-safe, ref-field-free 2D equivalent of the 3D live <c>RefRW</c>).</summary>
        public KinematicCharacterBody2D CharacterBodySnapshot;

        /// <summary>The character's static properties.</summary>
        public KinematicCharacterProperties2D CharacterProperties;

        /// <summary>The character's step- and slope-handling parameters (read by the step-aware grounding callback).</summary>
        public BasicStepAndSlopeHandlingParameters2D StepAndSlopeHandling;

        /// <summary>The character entity (so the step-grounding raycasts skip the character's own body).</summary>
        public Entity CharacterEntity;

        /// <inheritdoc/>
        public void UpdateGroundingUp(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext)
        {
            // A platformer's grounding-up is fixed at world +Y; the physics system sets it on the live body before
            // Initialize and the snapshot mirrors it, so this is a no-op.
        }

        /// <inheritdoc/>
        public bool CanCollideWithHit(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit)
        {
            return hit.Entity != Entity.Null;
        }

        /// <inheritdoc/>
        public bool IsGroundedOnHit(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit,
            int groundingEvaluationType)
        {
            return KinematicCharacterUtilities2D.Default_IsGroundedOnHit(
                ref baseContext,
                CharacterEntity,
                in CharacterBodySnapshot,
                in CharacterProperties,
                in StepAndSlopeHandling,
                in hit,
                groundingEvaluationType);
        }

        /// <inheritdoc/>
        public void OnMovementHit(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterHit2D hit,
            ref float2 remainingMovementDirection,
            ref float remainingMovementLength,
            float2 originalVelocityDirection,
            float hitDistance)
        {
            // The core MoveWithCollisions applies the default movement-hit behaviour directly (it needs the live
            // position/body refs). This is the override seam for a consumer driving the steps themselves; empty here.
        }

        /// <inheritdoc/>
        public void ProjectVelocityOnHits(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref float2 velocity,
            ref bool characterIsGrounded,
            ref BasicHit2D characterGroundHit,
            in DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHits,
            float2 originalVelocityDirection)
        {
            KinematicCharacterUtilities2D.Default_ProjectVelocityOnHits(
                ref velocity,
                ref characterIsGrounded,
                ref characterGroundHit,
                in velocityProjectionHits,
                originalVelocityDirection,
                StepAndSlopeHandling.ConstrainVelocityToGroundPlane,
                in CharacterBodySnapshot);
        }

        /// <inheritdoc/>
        public void OverrideDynamicHitMasses(
            ref PlatformerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterMass2D characterMass,
            ref KinematicCharacterMass2D otherMass,
            BasicHit2D hit)
        {
            // No mass override — the pushable-box demo uses the authored masses directly.
        }
    }
}
