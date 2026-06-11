using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// The physics half of the design §8 control→physics split: the fixed-step solve that turns each side-scroller
    /// character's <see cref="CharacterControl2D"/> intent into motion. It is the 2D analogue of the 3D Standard
    /// Characters sample's own solve <c>IJobEntity</c> that called <c>aspect.PhysicsUpdate(...)</c> — the package ships
    /// the steps, the sample chains them. Per fixed step, for every <see cref="SideScrollerCharacterTag"/> character it:
    /// <list type="number">
    /// <item>reads the current pose off <see cref="LocalToWorld"/> and sets <c>GroundingUp = +Y</c> (a side-scroller's
    /// up never changes);</item>
    /// <item>applies the control: gravity always, then horizontal accel toward the desired move speed (on the ground
    /// line when grounded, in the air plane otherwise) via <see cref="CharacterControlUtilities2D"/>;</item>
    /// <item>consumes a latched jump with <see cref="CharacterControlUtilities2D.StandardJump"/> while grounded,
    /// clearing the latch;</item>
    /// <item>runs the package's <see cref="KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D{T,C}"/> chain over the
    /// <see cref="SideScrollerCharacterProcessor"/>, which collide-and-slides, grounds, walks up steps/slopes,
    /// depenetrates, pushes dynamic bodies, and rides a moving platform, then delivers the pose with one swept
    /// <c>MovePosition</c>.</item>
    /// </list>
    ///
    /// <para><b>Ordering (design D3 + the C3 contract).</b> <c>[UpdateAfter(PhysicsWorld2DSystem)]</c> — the substrate's
    /// queries are valid only against the just-stepped world, so the solve reads frame N's world and enqueues the move
    /// the substrate drains on frame N+1 (the one-step pipeline latency the substrate's command/step/write-back cycle
    /// dictates). <c>[UpdateAfter(StoreKinematicCharacterBodyPropertiesSystem2D)]</c> (read the pre-solve char↔char
    /// snapshot) and <c>[UpdateBefore(KinematicCharacterDeferredImpulsesSystem2D)]</c> (the deferred drain applies the
    /// recorded impulses after the solve) — the exact edges the built-in solve declares, so this sample slots into the
    /// same resolved order: <c>PhysicsWorld2DSystem → Store… → (this) → DeferredImpulses…</c>.</para>
    ///
    /// <para><b>Burst.</b> The solve never touches managed input (that was the control system's job); it reads the
    /// already-written <see cref="CharacterControl2D"/> component, so it Bursts and parallelizes. <c>[BurstCompile]</c>
    /// on the <c>ISystem</c> entry points and the nested job only (entry-point rule). Gated on
    /// <see cref="SideScrollerSampleConfig"/> so it is inert on import.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StoreKinematicCharacterBodyPropertiesSystem2D))]
    [UpdateBefore(typeof(KinematicCharacterDeferredImpulsesSystem2D))]
    [BurstCompile]
    public partial struct SideScrollerCharacterPhysicsSystem : ISystem
    {
        /// <summary>Downward gravity magnitude (m/s²) along world -Y — the platformer convention.</summary>
        public const float GravityMagnitude = 20f;

        /// <summary>Max horizontal ground speed (units/s).</summary>
        public const float GroundMoveSpeed = 7f;

        /// <summary>
        /// Sharpness of the interpolation toward the desired ground velocity (higher = snappier). With no move input
        /// the desired velocity is zero, so a positive sharpness is the ground grip that decelerates the character to
        /// a stop — the analogue of the 3D ThirdPerson sample's <c>GroundedMovementSharpness</c>. The earlier
        /// <c>StandardGroundMove_Accelerated</c> call never decelerated toward a zero target, so a released character
        /// slid forever as if the level were ice.
        /// </summary>
        public const float GroundedMovementSharpness = 15f;

        /// <summary>Max horizontal air speed (units/s).</summary>
        public const float AirMoveSpeed = 7f;

        /// <summary>Horizontal air acceleration (units/s²) — looser than ground for a touch of air control.</summary>
        public const float AirAcceleration = 30f;

        /// <summary>Upward jump speed (units/s) added to velocity on a grounded jump.</summary>
        public const float JumpSpeed = 9f;

        EntityQuery _characterQuery;
        KinematicCharacterUpdateContext2D _baseContext;
        ComponentLookup<LocalToWorld> _localToWorldLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // The query MUST list every component AND buffer the job's Execute accesses, plus Simulate (the job is
            // [WithAll(typeof(Simulate))]) — the C4a gate's load-bearing lesson: an IJobEntity with a custom query
            // validates the query against Execute at SCHEDULE time, so an omitted buffer throws there, not at compile.
            _characterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<SideScrollerCharacterTag>()
                .WithAll<KinematicCharacterProperties2D, KinematicCharacterColliderProxy2D>()
                .WithAllRW<KinematicCharacterBody2D>()
                .WithAllRW<CharacterControl2D>()
                .WithAll<BasicStepAndSlopeHandlingParameters2D, LocalToWorld>()
                .WithAll<KinematicCharacterHit2D, StatefulKinematicCharacterHit2D>()
                .WithAll<KinematicCharacterDeferredImpulse2D, KinematicVelocityProjectionHit2D>()
                .WithAll<PhysicsBody2DCommand, Simulate>()
                .Build(ref state);

            state.RequireForUpdate(_characterQuery);
            state.RequireForUpdate<PhysicsWorldSingleton2D>();
            state.RequireForUpdate<SideScrollerSampleConfig>();

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

            SideScrollerSolveJob job = new SideScrollerSolveJob
            {
                BaseContext = _baseContext,
                BodyTransformLookup = _localToWorldLookup,
                DeltaTime = SystemAPI.Time.DeltaTime,
            };

            state.Dependency = job.ScheduleParallel(_characterQuery, state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        partial struct SideScrollerSolveJob : IJobEntity
        {
            public KinematicCharacterUpdateContext2D BaseContext;

            [ReadOnly]
            public ComponentLookup<LocalToWorld> BodyTransformLookup;

            public float DeltaTime;

            void Execute(
                Entity entity,
                ref KinematicCharacterBody2D characterBody,
                ref CharacterControl2D control,
                in KinematicCharacterProperties2D characterProperties,
                in KinematicCharacterColliderProxy2D colliderProxy,
                in BasicStepAndSlopeHandlingParameters2D stepAndSlopeHandling,
                in LocalToWorld localToWorld,
                DynamicBuffer<KinematicCharacterHit2D> characterHitsBuffer,
                DynamicBuffer<StatefulKinematicCharacterHit2D> statefulHitsBuffer,
                DynamicBuffer<KinematicCharacterDeferredImpulse2D> deferredImpulsesBuffer,
                DynamicBuffer<KinematicVelocityProjectionHit2D> velocityProjectionHits,
                DynamicBuffer<PhysicsBody2DCommand> commandBuffer)
            {
                BaseContext.EnsureCreationOfTmpCollections();

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

                // A side-scroller's up is fixed at world +Y; set it on the live body before Initialize so the snapshot
                // mirrors it and the processor's UpdateGroundingUp is a no-op.
                float2 groundingUp = new float2(0f, 1f);
                characterBody.GroundingUp = groundingUp;

                float2 gravity = new float2(0f, -GravityMagnitude);

                // --- Control → velocity (the sample's character logic, on the live body) -----------------------

                // Gravity always (the grounding step zeroes the ground-normal component when grounded, so this does
                // not fight standing on flat ground; it pulls the character down a slope / off a ledge / while airborne).
                characterBody.RelativeVelocity += gravity * DeltaTime;

                float2 targetMove = new float2(control.MoveX, 0f);

                if (characterBody.IsGrounded)
                {
                    // Interpolate horizontal velocity toward the desired ground velocity along the GROUND LINE. With no
                    // move input the desired velocity is zero, so the character DECELERATES to a stop (normal ground
                    // grip) — the accelerated form only ever ADDED acceleration*dt, so a zero input added nothing and
                    // the character kept its horizontal velocity forever (slid as if the level were ice). The 3D
                    // ThirdPerson sample uses this same StandardGroundMove_Interpolated for the same reason.
                    CharacterControlUtilities2D.StandardGroundMove_Interpolated(
                        ref characterBody.RelativeVelocity,
                        targetMove * GroundMoveSpeed,
                        GroundedMovementSharpness,
                        DeltaTime,
                        groundingUp,
                        characterBody.GroundHit.Normal);

                    // Consume a latched jump: unground, cancel downward velocity, add the jump impulse.
                    if (control.JumpPressed)
                    {
                        CharacterControlUtilities2D.StandardJump(
                            ref characterBody,
                            groundingUp * JumpSpeed,
                            cancelVelocityBeforeJump: true,
                            groundingUp);
                        control.JumpPressed = false;
                    }
                }
                else
                {
                    // Airborne: a touch of horizontal control on the horizontal plane (movementPlaneUp = +Y), gravity
                    // already applied above. The jump latch is left set so a jump tapped just before landing fires on
                    // the next grounded step.
                    CharacterControlUtilities2D.StandardAirMove(
                        ref characterBody.RelativeVelocity,
                        targetMove * AirAcceleration,
                        AirMoveSpeed,
                        groundingUp,
                        DeltaTime,
                        forceNoMaxSpeedExcess: false);
                }

                // Snapshot the was-grounded state AFTER the control/jump logic, so a jump (which set IsGrounded = false
                // above) makes WasGroundedBeforeCharacterUpdate = false in the snapshot the processor's grounding
                // callbacks read. This is the load-bearing ordering the default solve uses
                // (KinematicCharacterPhysicsSolveSystem2D sets it after its own ungrounding, before building the
                // snapshot): the solve's Default_IsGroundedOnHit only invokes ShouldPreventGroundingBasedOnVelocity —
                // the guard that prevents re-snapping to the floor a jump is launching off — when WasGrounded is false.
                // Setting it before the jump (the original sample bug) left it true, so the guard never fired, the
                // grounding step re-grounded the character on the floor still directly below it (the MovePosition has a
                // one-step latency, so the body has not physically risen yet), and the +Y jump velocity was projected
                // onto the flat ground plane and zeroed — the character never left the ground.
                characterBody.WasGroundedBeforeCharacterUpdate = characterBody.IsGrounded;

                // --- The package solve over the sample processor -----------------------------------------------

                var processor = new SideScrollerCharacterProcessor
                {
                    CharacterBodySnapshot = characterBody,
                    CharacterProperties = characterProperties,
                    StepAndSlopeHandling = stepAndSlopeHandling,
                    CharacterEntity = entity,
                };

                var userContext = new SideScrollerCharacterUpdateContext();

                KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D(
                    ref characterContext,
                    ref characterBody,
                    in processor,
                    ref userContext,
                    ref BaseContext,
                    in BodyTransformLookup,
                    in stepAndSlopeHandling,
                    gravity,
                    DeltaTime);
            }
        }
    }
}
