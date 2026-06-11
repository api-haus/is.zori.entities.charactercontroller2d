using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// The sample's user-context (the <c>C</c> in <see cref="IKinematicCharacterProcessor2D{C}"/>) — the 2D analogue
    /// of the 3D Standard Characters sample's <c>…CharacterUpdateContext</c>
    /// (REF/Samples~/…/FirstPersonCharacterProcessor.cs:8). It carries no extra global data; the side-scroller solve
    /// needs only the base context, exactly like the default processor's context. A richer Phase-B platformer adds
    /// its own <c>ComponentLookup</c>s / singletons here.
    /// </summary>
    public struct SideScrollerCharacterUpdateContext { }

    /// <summary>
    /// The concrete <see cref="IKinematicCharacterProcessor2D{C}"/> for the side-scroller character — a worked example
    /// of the six callbacks, the 2D analogue of the 3D sample's <c>FirstPersonCharacterProcessor</c>
    /// (REF/Samples~/…/FirstPersonCharacterProcessor.cs:24) minus the camera/look concerns. It wires the standard
    /// grounding / velocity-projection / step-grounding behaviours by forwarding to the package's <c>Default_*</c>
    /// core methods, which is what a platformer character wants: stand on slopes within the limit, slide along walls,
    /// kill velocity in corners, walk up steps.
    ///
    /// <para><b>The six choices.</b>
    /// <list type="bullet">
    /// <item><see cref="UpdateGroundingUp"/> — no-op: the physics system sets <c>GroundingUp = +Y</c> on the live body
    /// before Initialize (a side-scroller's up never changes), so the callback has nothing to do.</item>
    /// <item><see cref="CanCollideWithHit"/> — accept every resolved-entity hit: the substrate carries no per-hit
    /// material to gate on, and non-package hits already resolve to <see cref="Entity.Null"/> and are filtered out.</item>
    /// <item><see cref="IsGroundedOnHit"/> — forward to the step-aware <c>Default_IsGroundedOnHit</c> so a hit grounds
    /// the character when it is within the slope limit OR is a valid step (the platformer wants both).</item>
    /// <item><see cref="OnMovementHit"/> — empty: the core <c>MoveWithCollisions</c> calls the static
    /// <c>OnMovementHit</c> directly (it mutates the live position/body refs an <c>unmanaged</c> processor cannot
    /// carry); this callback is the override seam for a consumer that drives the steps itself.</item>
    /// <item><see cref="ProjectVelocityOnHits"/> — forward to <c>Default_ProjectVelocityOnHits</c> (the D6 corner-only
    /// 2D collapse): slide along a single surface, kill velocity in a wedge.</item>
    /// <item><see cref="OverrideDynamicHitMasses"/> — no override: the box-push uses the authored masses as-is.</item>
    /// </list>
    /// </para>
    ///
    /// <para>It carries the SAME read-only snapshot the default processor does (the body's grounding state taken at the
    /// start of the solve, the properties, the step params, the entity) because the non-mutating callbacks read those
    /// — the Entities-6.5 non-Aspect shape, where the solve runs inside an <c>IJobEntity</c> holding the live body by
    /// <c>ref</c> and the processor reads a value snapshot rather than a live <c>RefRW</c>. All callbacks are HPC#-clean
    /// (no <c>[BurstCompile]</c>; the solve job is the entry point).</para>
    /// </summary>
    public struct SideScrollerCharacterProcessor : IKinematicCharacterProcessor2D<SideScrollerCharacterUpdateContext>
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
            ref SideScrollerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext)
        {
            // A side-scroller's grounding-up is fixed at world +Y; the physics system sets it on the live body before
            // Initialize and the snapshot mirrors it, so this is a no-op.
        }

        /// <inheritdoc/>
        public bool CanCollideWithHit(
            ref SideScrollerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            in BasicHit2D hit)
        {
            return hit.Entity != Entity.Null;
        }

        /// <inheritdoc/>
        public bool IsGroundedOnHit(
            ref SideScrollerCharacterUpdateContext context,
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
            ref SideScrollerCharacterUpdateContext context,
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
            ref SideScrollerCharacterUpdateContext context,
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
            ref SideScrollerCharacterUpdateContext context,
            ref KinematicCharacterUpdateContext2D baseContext,
            ref KinematicCharacterMass2D characterMass,
            ref KinematicCharacterMass2D otherMass,
            BasicHit2D hit)
        {
            // No mass override — the pushable-box demo uses the authored masses directly.
        }
    }
}
