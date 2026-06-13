using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Records the previous and current fixed-rate poses of every entity carrying a
    /// <see cref="TrackedTransform2D"/> — the moving-platform bodies a character can be parented to. 2D port of
    /// <c>Unity.CharacterController.TrackedTransformFixedSimulationSystem</c> (REF/TrackedTransformSystem.cs).
    /// </summary>
    /// <remarks>
    /// The 3D system reads the body's pose from <c>LocalTransform</c>. The 2D substrate's physics body owns
    /// <see cref="LocalToWorld"/> (not <c>LocalTransform</c>) — its pose is moved by the substrate and landed in
    /// <see cref="LocalToWorld"/> by <c>PhysicsBody2DWriteBackSystem</c> — so the tracked pose is read from
    /// <see cref="LocalToWorld"/>: the position from its translation, and the z-angle from its local +X basis
    /// vector (a flat, unscaled physics-body matrix). Running
    /// <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> captures the pose after the whole physics group,
    /// including the write-back that lands it in <see cref="LocalToWorld"/>, matching
    /// <see cref="StoreKinematicCharacterBodyPropertiesSystem2D"/>, so the solve reads a current/previous pair from
    /// the step it queries (design section 5).
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    [BurstCompile]
    public partial struct TrackedTransformSystem2D : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<TrackedTransform2D>();
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            TrackedTransformSystem2DJob job = new TrackedTransformSystem2DJob();
            job.ScheduleParallel();
        }

        /// <summary>
        /// Shifts the current pose into the previous slot and captures the body's just-stepped
        /// <see cref="LocalToWorld"/> pose as the new current.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct TrackedTransformSystem2DJob : IJobEntity
        {
            void Execute(ref TrackedTransform2D trackedTransform, in LocalToWorld localToWorld)
            {
                trackedTransform.PreviousFixedRateTransform = trackedTransform.CurrentFixedRateTransform;

                float4x4 m = localToWorld.Value;
                float2 position = m.c3.xy;
                // The z-rotation angle of a flat 2D physics-body matrix: the angle of its local +X basis vector.
                float angle = atan2(m.c0.y, m.c0.x);

                trackedTransform.CurrentFixedRateTransform = new RigidTransform2D
                {
                    Position = position,
                    Rotation = angle,
                };
            }
        }
    }
}
