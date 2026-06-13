using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Applies the impulses recorded on every character's deferred-impulse buffer during the solve, after the
    /// solve has run. 2D port of <c>Unity.CharacterController.KinematicCharacterDeferredImpulsesSystem</c>
    /// (REF/KinematicCharacterDeferredImpulsesSystem.cs), which sits <c>OrderLast</c> in the 3D character physics
    /// group.
    /// </summary>
    /// <remarks>
    /// The structural difference from the 3D system is the write-in path for a NON-character body. The 3D system
    /// owns the other body's <c>PhysicsVelocity</c> and writes <c>Linear</c>/<c>Angular</c> directly, plus nudges
    /// <c>LocalTransform.Position</c> for the displacement. The 2D substrate has no <c>PhysicsVelocity</c> for a
    /// kinematic-or-dynamic body and owns <see cref="Unity.Transforms.LocalToWorld"/> (not
    /// <c>LocalTransform</c>) through <c>PhysicsBody2DWriteBackSystem</c>, so a body's velocity is changed by
    /// enqueuing a write-in command onto its <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c>
    /// (<see cref="PhysicsBody2DCommands.AddForce(Unity.Entities.DynamicBuffer{PhysicsBody2DCommand},float2,PhysicsForceMode2D)"/>
    /// in <see cref="PhysicsForceMode2D.Impulse"/> mode), drained by <c>PhysicsWorld2DSystem</c> before the next
    /// step (P2D/Runtime/PhysicsBody2DCommands.cs). A character target keeps the 3D path verbatim: its
    /// <see cref="KinematicCharacterBody2D.RelativeVelocity"/> is written directly through a
    /// <see cref="ComponentLookup{T}"/>.
    ///
    /// <para>Ordering: <c>[UpdateInGroup(FixedStepSimulationSystemGroup)]</c> after this system's sibling store
    /// system; the solve system (<c>KinematicCharacterPhysicsSolveSystem2D</c>) declares
    /// <c>[UpdateBefore(KinematicCharacterDeferredImpulsesSystem2D)]</c> so the drain runs after the solve. This
    /// system owns no forward reference to the solve type — the drain-after-solve order is the solve system's to
    /// declare against this type.</para>
    ///
    /// <para>The buffer helpers are HPC#-clean buffer appends (no <c>[BurstCompile]</c>, per the entry-point-only
    /// rule), so this <c>[BurstCompile]</c> job calls them and they auto-compile from the Burst context
    /// (docs/unity/burst/compilation-context.md).</para>
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StoreKinematicCharacterBodyPropertiesSystem2D))]
    [BurstCompile]
    public partial struct KinematicCharacterDeferredImpulsesSystem2D : ISystem
    {
        EntityQuery _characterQuery;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            _characterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<KinematicCharacterDeferredImpulse2D>()
                .Build(ref state);
            state.RequireForUpdate(_characterQuery);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            KinematicCharacterDeferredImpulsesJob job = new KinematicCharacterDeferredImpulsesJob
            {
                CharacterBodyLookup = SystemAPI.GetComponentLookup<KinematicCharacterBody2D>(false),
                CharacterPropertiesLookup = SystemAPI.GetComponentLookup<KinematicCharacterProperties2D>(true),
                CommandBufferLookup = SystemAPI.GetBufferLookup<PhysicsBody2DCommand>(false),
            };
            job.Schedule();
        }

        /// <summary>
        /// Drains each character's deferred-impulse buffer, applying the linear velocity change to the target —
        /// to another character's <see cref="KinematicCharacterBody2D.RelativeVelocity"/> directly, or to a
        /// regular body via an <see cref="PhysicsForceMode2D.Impulse"/> command on its
        /// <see cref="PhysicsBody2DCommand"/> buffer.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct KinematicCharacterDeferredImpulsesJob : IJobEntity
        {
            /// <summary>Lookup for the character body, written for a character target.</summary>
            public ComponentLookup<KinematicCharacterBody2D> CharacterBodyLookup;

            /// <summary>Lookup identifying whether a target entity is itself a character.</summary>
            [ReadOnly]
            public ComponentLookup<KinematicCharacterProperties2D> CharacterPropertiesLookup;

            /// <summary>Lookup for a regular body's write-in command buffer, appended for a non-character target.</summary>
            public BufferLookup<PhysicsBody2DCommand> CommandBufferLookup;

            void Execute(in DynamicBuffer<KinematicCharacterDeferredImpulse2D> characterDeferredImpulsesBuffer)
            {
                for (int i = 0; i < characterDeferredImpulsesBuffer.Length; i++)
                {
                    KinematicCharacterDeferredImpulse2D deferredImpulse = characterDeferredImpulsesBuffer[i];

                    // Linear velocity change.
                    bool isImpulseOnCharacter = CharacterPropertiesLookup.HasComponent(deferredImpulse.OnEntity);
                    if (isImpulseOnCharacter)
                    {
                        KinematicCharacterProperties2D hitCharacterProperties = CharacterPropertiesLookup[
                            deferredImpulse.OnEntity
                        ];
                        if (hitCharacterProperties.SimulateDynamicBody)
                        {
                            KinematicCharacterBody2D hitCharacterBody = CharacterBodyLookup[deferredImpulse.OnEntity];
                            hitCharacterBody.RelativeVelocity += deferredImpulse.LinearVelocityChange;
                            CharacterBodyLookup[deferredImpulse.OnEntity] = hitCharacterBody;
                        }
                    }
                    else if (
                        lengthsq(deferredImpulse.LinearVelocityChange) > 0f
                        && CommandBufferLookup.HasBuffer(deferredImpulse.OnEntity)
                    )
                    {
                        // A regular body has no PhysicsVelocity in the 2D substrate; its velocity is changed by
                        // an Impulse write-in command that PhysicsWorld2DSystem drains before the next step. The
                        // helper appends the raw Δv and Box2D mass-scales it during the step — the substrate's
                        // documented impulse contract (P2D/Runtime/PhysicsBody2DCommands.cs).
                        DynamicBuffer<PhysicsBody2DCommand> targetCommands = CommandBufferLookup[
                            deferredImpulse.OnEntity
                        ];
                        PhysicsBody2DCommands.AddForce(
                            targetCommands,
                            deferredImpulse.LinearVelocityChange,
                            PhysicsForceMode2D.Impulse
                        );
                    }
                }
            }
        }
    }
}
