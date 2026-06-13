using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The package's built-in solve system: runs the core solve chain
    /// (<see cref="KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D{T,C}"/>) once per fixed step for every
    /// character tagged <see cref="DefaultCharacterController2DTag"/>, using the
    /// <see cref="DefaultKinematicCharacterProcessor2D"/>. A character WITHOUT the tag is never touched here, so a
    /// consumer driving a custom processor from their own system is not double-stepped — a concrete tag-gated
    /// default, not a generic auto-running system.
    ///
    /// <para><b>Ordering.</b> The controller systems run
    /// <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> — the substrate's queries are valid only against the
    /// just-stepped world, so the solve reads the world stepped on frame N, computes the target pose, and enqueues the
    /// <see cref="PhysicsBody2DCommands.MovePosition"/> that <c>PhysicsWorld2DSystem</c> drains and applies on frame
    /// N+1 (a one-step pipeline latency, the natural fit for the substrate's command-drain-before-step cycle). The
    /// two ordering edges, declared verbatim: <c>[UpdateAfter(StoreKinematicCharacterBodyPropertiesSystem2D)]</c>
    /// (read the pre-solve snapshot) and <c>[UpdateBefore(KinematicCharacterDeferredImpulsesSystem2D)]</c> (the
    /// drain runs after the solve records impulses). Resolved order:
    /// <c>Physics2DSimulationSystemGroup → Store… → SolveSystem → DeferredImpulses…</c>.</para>
    ///
    /// <para><b>Default gravity.</b> The 3D sample carries gravity on the character's own component;
    /// <see cref="KinematicCharacterProperties2D"/> has no gravity field, so the default path applies a fixed
    /// <see cref="DefaultGravityMagnitude"/> to <see cref="KinematicCharacterBody2D.RelativeVelocity"/> while airborne so a
    /// default character falls and lands. A richer consumer drives velocity from its own control component and
    /// processor instead of the default tag.</para>
    ///
    /// <para><b>Burst.</b> <c>[BurstCompile]</c> on the <c>ISystem</c> entry points and the nested
    /// <see cref="KinematicCharacterPhysicsSolveJob"/> only (entry-point rule). The whole core solve is HPC#-clean:
    /// it never reads a regular dynamic body's managed velocity (that read is the hit-dynamics path, which would
    /// run main-thread), so this job Bursts and parallelizes.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StoreKinematicCharacterBodyPropertiesSystem2D))]
    [UpdateBefore(typeof(KinematicCharacterDeferredImpulsesSystem2D))]
    [BurstCompile]
    public partial struct KinematicCharacterPhysicsSolveSystem2D : ISystem
    {
        /// <summary>
        /// The default downward gravity magnitude (m/s²) the default solve applies to an airborne tagged character,
        /// along <c>-world-Y</c> (the platformer convention). A <c>const</c> scalar (not a <c>static readonly</c>
        /// <c>float2</c>) so Burst reads it as a compile-time constant rather than a managed static-field read; the
        /// job builds the <c>float2(0, -DefaultGravityMagnitude)</c> locally. Only used by the built-in default path.
        /// </summary>
        public const float DefaultGravityMagnitude = 9.81f;

        EntityQuery _characterQuery;
        KinematicCharacterUpdateContext2D _baseContext;
        ComponentLookup<LocalToWorld> _localToWorldLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // The query MUST contain every component the solve job's Execute accesses, or scheduling an IJobEntity
            // with a custom query throws "the query must (at the very minimum) contain all the components required
            // for …Execute()". That includes the five DynamicBuffers the Execute takes and the Simulate tag the job
            // is [WithAll(typeof(Simulate))]-gated on.
            _characterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<DefaultCharacterController2DTag>()
                .WithAll<KinematicCharacterProperties2D, KinematicCharacterColliderProxy2D>()
                .WithAllRW<KinematicCharacterBody2D>()
                .WithAll<BasicStepAndSlopeHandlingParameters2D, LocalToWorld>()
                .WithAll<KinematicCharacterHit2D, StatefulKinematicCharacterHit2D>()
                .WithAll<KinematicCharacterDeferredImpulse2D, KinematicVelocityProjectionHit2D>()
                .WithAll<PhysicsBody2DCommand, Simulate>()
                .Build(ref state);

            state.RequireForUpdate(_characterQuery);
            state.RequireForUpdate<PhysicsWorldSingleton2D>();

            _baseContext = default;
            _baseContext.OnSystemCreate(ref state);
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(true);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            PhysicsWorld physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton2D>().world;
            _baseContext.OnSystemUpdate(ref state, SystemAPI.Time, physicsWorld);
            _localToWorldLookup.Update(ref state);

            KinematicCharacterPhysicsSolveJob job = new KinematicCharacterPhysicsSolveJob
            {
                BaseContext = _baseContext,
                BodyTransformLookup = _localToWorldLookup,
                DeltaTime = SystemAPI.Time.DeltaTime,
            };

            // ScheduleParallel: each character solves independently (the store system already snapshotted the
            // pre-solve state for deterministic char↔char exchange, which the core solve does not yet use). The
            // cast-back reads LocalToWorld through a [ReadOnly] lookup, so no write aliasing with the iterated
            // LocalToWorld (also read-only here).
            state.Dependency = job.ScheduleParallel(_characterQuery, state.Dependency);
        }

        /// <summary>
        /// Per-character solve: read the current pose off <see cref="LocalToWorld"/>, set grounding-up from the
        /// body's current rotation, apply default gravity while airborne, then run the core chain and enqueue the
        /// move. Built fresh per entity from the entity's own <c>ref</c>/<c>in</c> components.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct KinematicCharacterPhysicsSolveJob : IJobEntity
        {
            public KinematicCharacterUpdateContext2D BaseContext;

            [Unity.Collections.ReadOnly]
            public ComponentLookup<LocalToWorld> BodyTransformLookup;

            public float DeltaTime;

            void Execute(
                Entity entity,
                ref KinematicCharacterBody2D characterBody,
                in KinematicCharacterProperties2D characterProperties,
                in KinematicCharacterColliderProxy2D colliderProxy,
                in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
                in LocalToWorld localToWorld,
                DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
                DynamicBuffer<StatefulKinematicCharacterHit2D> statefulHitsBuffer,
                DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
                DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHits,
                DynamicBuffer<PhysicsBody2DCommand> commandBuffer
            )
            {
                // The base context's scratch list is created lazily inside the job (per-thread Temp).
                BaseContext.EnsureCreationOfTmpCollections();

                // Build the per-entity context bundling the read-mostly data + the buffers (the non-Aspect
                // replacement for the 3D KinematicCharacterAspect). The mutated body is threaded as a ref param.
                var characterContext = new KinematicCharacterContext2D
                {
                    Entity = entity,
                    CharacterProperties = characterProperties,
                    ColliderProxy = colliderProxy,
                    CharacterHitsBuffer = characterHitsBuffer,
                    StatefulHitsBuffer = statefulHitsBuffer,
                    DeferredImpulsesBuffer = deferredImpulsesBuffer,
                    VelocityProjectionHits = velocityProjectionHits,
                    CommandBuffer = commandBuffer,
                };
                KinematicCharacterPhysicsUpdate2D.ReadPoseFromLocalToWorld(ref characterContext, in localToWorld);

                // Default grounding-up = the character transform's up (Default_UpdateGroundingUp,
                // REF/KinematicCharacterUtilities.cs:1185). Set on the live body so the snapshot below mirrors it
                // and the processor's UpdateGroundingUp callback is a no-op.
                characterBody.GroundingUp = MathUtilities2D.UpFromAngle(characterContext.CurrentRotation);

                // Pre-set the was-grounded snapshot field that Update_Initialize will set anyway
                // (WasGroundedBeforeCharacterUpdate = IsGrounded). Doing it here too means the processor snapshot
                // built below already carries THIS frame's was-grounded — the read-only grounding callbacks
                // (IsGroundedOnHit, ProjectVelocityOnHits) read it off the snapshot, and Initialize re-applies the
                // same value idempotently inside the chain. (The 3D processor avoids the snapshot entirely by
                // holding a live RefRW to the body; the snapshot is the C#-9-safe, ref-field-free 2D equivalent.)
                characterBody.WasGroundedBeforeCharacterUpdate = characterBody.IsGrounded;

                // Default velocity control: apply gravity while airborne so the default character falls and lands.
                // A richer consumer replaces this with its own control component + processor.
                if (!characterBody.IsGrounded)
                {
                    characterBody.RelativeVelocity += new float2(0f, -DefaultGravityMagnitude) * DeltaTime;
                }

                // The default processor's read-only callbacks read this snapshot (grounding-up, was-grounded, the
                // pre-solve relative velocity, properties, constrain flag), taken AFTER grounding-up / was-grounded
                // / gravity are set, exactly as the 3D callbacks read CharacterBody.ValueRO at the ground-probe
                // phase. The snapshot's RelativeVelocity is the pre-movement velocity — correct for the dominant
                // grounding-probe check; the movement-loop airborne-escape guard reads it slightly stale, a known
                // minor refinement.
                var processor = new DefaultKinematicCharacterProcessor2D
                {
                    CharacterBodySnapshot = characterBody,
                    CharacterProperties = characterProperties,
                    StepAndSlopeHandling = stepAndSlopeHandling,
                    CharacterEntity = entity,
                };

                var userContext = new DefaultCharacterUpdateContext2D();

                // The gravity the default path applies (above) is also the gravity the ground-pushing step presses
                // the dynamic ground down with — keep them the same so the push force matches the character's weight.
                float2 gravity = new float2(0f, -DefaultGravityMagnitude);

                KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D(
                    ref characterContext,
                    ref characterBody,
                    in processor,
                    ref userContext,
                    ref BaseContext,
                    in BodyTransformLookup,
                    in stepAndSlopeHandling,
                    gravity,
                    DeltaTime
                );
            }
        }
    }
}
