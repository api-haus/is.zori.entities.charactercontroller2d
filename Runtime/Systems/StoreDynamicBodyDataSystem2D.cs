using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The main-thread pre-pass that resolves the substrate's Burst gap: it snapshots every regular (non-character)
    /// dynamic body's velocity and mass into a <see cref="StoredDynamicBodyData2D"/> component, so the Burst
    /// character solve can read another body's motion for the hit-dynamics impulse exchange through a
    /// <see cref="ComponentLookup{T}"/> instead of touching the managed <c>PhysicsBody</c> handle (whose
    /// <c>linearVelocity</c>/<c>angularVelocity</c> are not Burst-callable).
    ///
    /// <para><b>Why a main-thread snapshot and not the live handle in the solve.</b> The velocity read is a managed
    /// property on the raw <c>Unity.U2D.Physics.PhysicsBody</c> handle and CANNOT run inside a <c>[BurstCompile]</c>
    /// job — exactly as the substrate's own write-back reads it on the main thread
    /// (<c>PhysicsBody2DWriteBackSystem.CaptureSmoothing</c>, "reads the body's managed velocity via the
    /// Unity.U2D.Physics handle, which is not Burst"). Snapshotting it here, on the main thread, before the solve,
    /// lets the solve stay <c>ScheduleParallel</c> and HPC#-clean (the character ↔ character path was already
    /// Burst-clean via <see cref="StoredKinematicCharacterData2D"/>; this extends the same store-then-read pattern
    /// to regular bodies). The chosen resolution snapshots into a lookup-readable component rather than serialize
    /// the whole hit-dynamics phase onto the main thread.</para>
    ///
    /// <para><b>Burst.</b> NO <c>[BurstCompile]</c> on <c>OnUpdate</c>: the body-velocity read is managed, so the
    /// snapshot loop runs as plain main-thread C# (a <c>SystemAPI.Query</c> <c>foreach</c>). The component-add for a
    /// newly-seen dynamic body is a structural change, applied via the
    /// <c>EndSimulationEntityCommandBufferSystem</c>.</para>
    ///
    /// <para><b>Ordering.</b> <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> so the read sees the just-stepped
    /// velocities, and <c>[UpdateBefore(KinematicCharacterPhysicsSolveSystem2D)]</c> so the snapshot exists before
    /// the solve reads it.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    [UpdateBefore(typeof(KinematicCharacterPhysicsSolveSystem2D))]
    public partial struct StoreDynamicBodyDataSystem2D : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            // Only run when there is at least one character that could read dynamic-body data — the snapshot is
            // pointless without a character solve that consumes it.
            state.RequireForUpdate<KinematicCharacterProperties2D>();
        }

        public void OnDestroy(ref SystemState state) { }

        // No [BurstCompile]: PhysicsBody.linearVelocity/angularVelocity are managed reads.
        public void OnUpdate(ref SystemState state)
        {
            // (1) Add the snapshot component to any dynamic body that has a live handle + a body definition but is
            // neither a character nor already snapshotted. A character body is excluded — its motion is read from
            // StoredKinematicCharacterData2D, not from a regular-body snapshot. The adds are collected first and
            // applied AFTER the iteration (a structural change mid-Query invalidates the iterator); the body's
            // velocity is captured next step, once the component exists.
            var toAdd = new NativeList<Entity>(Allocator.Temp);
            foreach (
                var (def, entity) in SystemAPI
                    .Query<RefRO<PhysicsBody2DDefinition>>()
                    .WithAll<PhysicsBody2D>()
                    .WithNone<StoredDynamicBodyData2D, KinematicCharacterProperties2D>()
                    .WithEntityAccess()
            )
            {
                if (def.ValueRO.bodyType == PhysicsBody.BodyType.Dynamic)
                {
                    toAdd.Add(entity);
                }
            }

            for (int i = 0; i < toAdd.Length; i++)
            {
                state.EntityManager.AddComponent<StoredDynamicBodyData2D>(toAdd[i]);
            }
            toAdd.Dispose();

            // (2) Fill the snapshot for every dynamic body that already carries the component. The velocity is read
            // off the live handle on the main thread; the mass/inertia are sourced from the authored definition
            // (the substrate exposes no read for a regular body's solved mass — the authored value is the faithful
            // source, processor-overridable at solve time).
            foreach (
                var (stored, body, def) in SystemAPI
                    .Query<RefRW<StoredDynamicBodyData2D>, RefRO<PhysicsBody2D>, RefRO<PhysicsBody2DDefinition>>()
                    .WithNone<KinematicCharacterProperties2D>()
            )
            {
                bool isDynamic = def.ValueRO.bodyType == PhysicsBody.BodyType.Dynamic;
                float2 linearVelocity = new float2(0f, 0f);
                float angularVelocity = 0f;

                if (
                    isDynamic
                    && PhysicsUtilities2D.TryGetDynamicBodyMotion(body.ValueRO.body, out float2 v, out float w)
                )
                {
                    linearVelocity = v;
                    angularVelocity = w;
                }

                stored.ValueRW = new StoredDynamicBodyData2D
                {
                    IsDynamic = isDynamic,
                    LinearVelocity = linearVelocity,
                    AngularVelocity = angularVelocity,
                    Mass = def.ValueRO.mass,
                    RotationalInertia = def.ValueRO.overrideMassDistribution ? def.ValueRO.rotationalInertia : 0f,
                };
            }
        }
    }
}
