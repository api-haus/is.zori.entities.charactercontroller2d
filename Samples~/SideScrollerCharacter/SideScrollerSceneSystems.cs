using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// Drives the sample's moving platform back and forth in X each fixed step via a swept
    /// <c>PhysicsBody2DCommands.MovePosition</c>. Runs <c>[UpdateBefore(PhysicsWorld2DSystem)]</c> so the substrate
    /// drains the move and steps the platform THIS frame (a platform is a DRIVEN kinematic body, not a solved one —
    /// unlike the character, whose move applies next frame via the read-after-step pipeline). The substrate's
    /// <c>TrackedTransformSystem2D</c> then records the platform's just-stepped pose
    /// <c>[UpdateAfter(PhysicsWorld2DSystem)]</c>, and the character's solve (also after the step) carries itself with
    /// that one-fixed-step platform delta (<c>Update_ParentMovement</c>) — the moving-platform feature the C4b gate
    /// verified.
    ///
    /// <para>The platform's <see cref="TrackedTransform2D"/> and its <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c>
    /// are added at runtime by <see cref="SideScrollerPlatformInitSystem"/> — a substrate kinematic body baked from
    /// <c>PhysicsBody2DAuthoring</c> has neither (no baker authors a tracked-transform, and the command buffer is the
    /// body owner's responsibility). The init split keeps the structural change off the Burst mover's hot path.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(PhysicsWorld2DSystem))]
    [BurstCompile]
    public partial struct SideScrollerMovingPlatformSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<SideScrollerSampleConfig>();
            state.RequireForUpdate<SideScrollerMovingPlatform>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            float dt = SystemAPI.Time.DeltaTime;

            foreach (var (platform, commands, ltw) in
                     SystemAPI.Query<RefRW<SideScrollerMovingPlatform>, DynamicBuffer<PhysicsBody2DCommand>, RefRO<LocalToWorld>>()
                         .WithAll<Simulate>())
            {
                // Capture the home X from the baked pose on the first update (so the oscillation centres on where the
                // platform was authored, not on world origin).
                if (!platform.ValueRO.HomeCaptured)
                {
                    platform.ValueRW.HomeX = ltw.ValueRO.Value.c3.x;
                    platform.ValueRW.FixedY = ltw.ValueRO.Value.c3.y;
                    platform.ValueRW.HomeCaptured = true;
                }

                platform.ValueRW.Phase += dt;

                // A smooth triangle/sine oscillation in X within ±TravelHalfExtentX around HomeX. Sine keeps the
                // velocity continuous (no instantaneous reversal jerk a triangle wave would give a rider).
                float omega = platform.ValueRO.TravelHalfExtentX > 0f
                    ? platform.ValueRO.SpeedX / platform.ValueRO.TravelHalfExtentX
                    : 0f;
                float targetX = platform.ValueRO.HomeX
                    + (platform.ValueRO.TravelHalfExtentX * sin(platform.ValueRO.Phase * omega));

                PhysicsBody2DCommands.MovePosition(commands, new float2(targetX, platform.ValueRO.FixedY));
            }
        }
    }

    /// <summary>
    /// One-shot structural setup for the moving platform: adds the <see cref="TrackedTransform2D"/> (so the controller
    /// treats it as a moving platform) and the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> (so
    /// <see cref="SideScrollerMovingPlatformSystem"/> can drive it) to any <see cref="SideScrollerMovingPlatform"/>
    /// that lacks them. This is the "<c>TrackedTransform2D</c> authoring path" the validation gate flagged a moving
    /// platform needs — provided here as a tiny runtime add rather than a baker, since the controller package ships no
    /// tracked-transform baker (a <c>TrackedTransform2DAuthoring</c>/baker is the obvious package addition the C4b gate
    /// already named; until then this is the consumer's responsibility and the sample shows the pattern).
    ///
    /// <para>Runs in the <see cref="InitializationSystemGroup"/> (a structural change, off the fixed-step hot path) and
    /// uses an <see cref="EntityCommandBuffer"/> so the add happens at a sync point, never mid-query.</para>
    /// </summary>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct SideScrollerPlatformInitSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<SideScrollerSampleConfig>();
            state.RequireForUpdate<SideScrollerMovingPlatform>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<SideScrollerMovingPlatform>>().WithNone<TrackedTransform2D>().WithEntityAccess())
            {
                ecb.AddComponent(entity, new TrackedTransform2D());
            }

            // The command buffer is added in a separate pass (WithNone keyed on the buffer type) so a platform missing
            // both gets both, and a platform missing only one gets only the one.
            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<SideScrollerMovingPlatform>>().WithNone<PhysicsBody2DCommand>().WithEntityAccess())
            {
                ecb.AddBuffer<PhysicsBody2DCommand>(entity);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }

    /// <summary>
    /// One-shot structural setup for the pushable crates: adds the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> to
    /// any <see cref="SideScrollerPushable"/> body that lacks it. The controller's deferred-impulse system pushes a
    /// regular dynamic body only <c>if HasBuffer(target)</c> — and a <c>PhysicsBody2DAuthoring</c> crate is baked
    /// WITHOUT a command buffer (only the controller's own baker adds one, to the character). So out of the box the
    /// character's mass-scaled push of a crate is silently dropped; this system closes that gap for the tagged crates.
    /// This is the single most important integration fact from the C4b gate, surfaced as the sample's pattern (the
    /// clean long-term fix is a substrate/baking change so any impulse-receiving dynamic body carries the buffer).
    /// </summary>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct SideScrollerPushableInitSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<SideScrollerSampleConfig>();
            state.RequireForUpdate<SideScrollerPushable>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<SideScrollerPushable>>().WithNone<PhysicsBody2DCommand>().WithEntityAccess())
            {
                ecb.AddBuffer<PhysicsBody2DCommand>(entity);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
