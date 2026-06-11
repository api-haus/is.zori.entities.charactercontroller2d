using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// The physics half of the design §8 control→physics split: the fixed-step solve that turns each Platformer
    /// character's <see cref="PlatformerCharacterControl2D"/> intent into motion. It extends the SideScroller's verified
    /// solve with the three-stance machine and the per-character tuning / friction read. Per fixed step, for every
    /// <see cref="PlatformerCharacterTag"/> character it:
    /// <list type="number">
    /// <item>reads the current pose off <see cref="LocalToWorld"/> and sets <c>GroundingUp = +Y</c> (a platformer's up
    /// never changes);</item>
    /// <item>switches on <see cref="PlatformerCharacterState2D.Stance"/> to choose the velocity-control block, all
    /// reading the PER-CHARACTER <see cref="PlatformerCharacterTuning2D"/> (gravity / speeds / accels / jump), NOT
    /// hardcoded consts — so two characters in one scene can move differently;</item>
    /// <item>snapshots <c>WasGroundedBeforeCharacterUpdate</c> AFTER the control/jump block (the jump fix — a jump that
    /// ungrounds the body must leave WasGrounded false so the grounding step does not re-snap the launch);</item>
    /// <item>runs the package's <see cref="KinematicCharacterPhysicsUpdate2D.PhysicsUpdate2D{T,C}"/> chain over the
    /// <see cref="PlatformerCharacterProcessor"/>, which collide-and-slides, grounds, walks up steps/slopes,
    /// depenetrates, pushes dynamic bodies, and rides a moving platform, then delivers the pose with one swept
    /// <c>MovePosition</c>.</item>
    /// </list>
    ///
    /// <para><b>The stance switch.</b>
    /// <list type="bullet">
    /// <item><see cref="PlatformerStance2D.GroundMove"/> — when grounded, accelerate along the ground line at the tuned
    /// ground speed, scaling the acceleration by the <see cref="FrictionModifier2D"/> read off the ground hit entity
    /// (ice → low → slippery, sticky → high), and consume a latched jump; when not grounded, fall through to the
    /// AirMove block (gravity + air control) so a character that walks off a ledge keeps moving.</item>
    /// <item><see cref="PlatformerStance2D.AirMove"/> — gravity always, plus a touch of air control on the horizontal
    /// plane; if a step lands the character grounded, the same block accelerates it on the ground line and consumes a
    /// jump, mirroring the SideScroller's grounded/airborne split inside one stance.</item>
    /// <item><see cref="PlatformerStance2D.RopeSwing"/> — <b>P4 stub.</b> The pendulum swing (position-clamp-to-rope +
    /// inward-velocity-projection against the stored <see cref="RopeSwingState2D"/>) is filled by chunk P4; for now this
    /// case falls through to the AirMove block so a rope-stanced character still moves (gravity + air control) rather
    /// than freezing. The transition logic that ENTERS/EXITS RopeSwing is also P4.</item>
    /// </list>
    /// </para>
    ///
    /// <para><b>Ordering (design D3 + the C3 contract).</b> <c>[UpdateAfter(PhysicsWorld2DSystem)]</c> (implied by the
    /// Store edge below — Store runs after the world step) — the substrate's queries are valid only against the
    /// just-stepped world, so the solve reads frame N's world and enqueues the move the substrate drains on frame N+1.
    /// <c>[UpdateAfter(StoreKinematicCharacterBodyPropertiesSystem2D)]</c> (read the pre-solve char↔char snapshot) and
    /// <c>[UpdateBefore(KinematicCharacterDeferredImpulsesSystem2D)]</c> (the deferred drain applies the recorded
    /// impulses after the solve) — the exact edges the built-in solve and the SideScroller declare, so this slots into
    /// the same resolved order: <c>PhysicsWorld2DSystem → Store… → (this) → DeferredImpulses…</c>.</para>
    ///
    /// <para><b>Burst.</b> The solve never touches managed input (the control system's job); it reads the already-written
    /// <see cref="PlatformerCharacterControl2D"/> component, so it Bursts and parallelizes. <c>[BurstCompile]</c> on the
    /// <c>ISystem</c> entry points and the nested job only (entry-point rule, docs/unity/burst/compilation-context.md:31).
    /// Gated on the <see cref="PlatformerCharacterTag"/> marker so it is inert on import.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StoreKinematicCharacterBodyPropertiesSystem2D))]
    [UpdateBefore(typeof(KinematicCharacterDeferredImpulsesSystem2D))]
    [BurstCompile]
    public partial struct PlatformerCharacterPhysicsSystem : ISystem
    {
        EntityQuery _characterQuery;
        KinematicCharacterUpdateContext2D _baseContext;
        ComponentLookup<LocalToWorld> _localToWorldLookup;
        ComponentLookup<FrictionModifier2D> _frictionModifierLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // The query MUST list every component AND buffer the job's Execute accesses, plus Simulate (the job is
            // [WithAll(typeof(Simulate))]) — the C4a gate's load-bearing lesson: an IJobEntity with a custom query
            // validates the query against Execute at SCHEDULE time, so an omitted buffer throws there, not at compile.
            // The Platformer adds PlatformerCharacterTuning2D (per-character tuning) + PlatformerCharacterState2D
            // (the stance) + RopeSwingState2D (the active rope params, read by the RopeSwing block / P4) to the
            // SideScroller's set.
            _characterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PlatformerCharacterTag>()
                .WithAll<PlatformerCharacterTuning2D>()
                .WithAllRW<PlatformerCharacterState2D>()
                .WithAll<RopeSwingState2D>()
                .WithAll<KinematicCharacterProperties2D, KinematicCharacterColliderProxy2D>()
                .WithAllRW<KinematicCharacterBody2D>()
                .WithAllRW<PlatformerCharacterControl2D>()
                .WithAll<BasicStepAndSlopeHandlingParameters2D, LocalToWorld>()
                .WithAll<KinematicCharacterHit2D, StatefulKinematicCharacterHit2D>()
                .WithAll<KinematicCharacterDeferredImpulse2D, KinematicVelocityProjectionHit2D>()
                .WithAll<PhysicsBody2DCommand, Simulate>()
                .Build(ref state);

            state.RequireForUpdate(_characterQuery);
            state.RequireForUpdate<PhysicsWorldSingleton2D>();
            // Inert on import: gate on the sample's own character marker (the query already requires it). Importing the
            // package bakes no PlatformerCharacterTag, so this runs nothing.
            state.RequireForUpdate<PlatformerCharacterTag>();

            _baseContext = default;
            _baseContext.OnSystemCreate(ref state);
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(true);
            _frictionModifierLookup = state.GetComponentLookup<FrictionModifier2D>(true);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            PhysicsWorld physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton2D>().world;
            _baseContext.OnSystemUpdate(ref state, SystemAPI.Time, physicsWorld);
            _localToWorldLookup.Update(ref state);
            _frictionModifierLookup.Update(ref state);

            PlatformerSolveJob job = new PlatformerSolveJob
            {
                BaseContext = _baseContext,
                BodyTransformLookup = _localToWorldLookup,
                FrictionModifierLookup = _frictionModifierLookup,
                DeltaTime = SystemAPI.Time.DeltaTime,
            };

            state.Dependency = job.ScheduleParallel(_characterQuery, state.Dependency);
        }

        [BurstCompile]
        [WithAll(typeof(Simulate))]
        partial struct PlatformerSolveJob : IJobEntity
        {
            public KinematicCharacterUpdateContext2D BaseContext;

            [ReadOnly]
            public ComponentLookup<LocalToWorld> BodyTransformLookup;

            [ReadOnly]
            public ComponentLookup<FrictionModifier2D> FrictionModifierLookup;

            public float DeltaTime;

            void Execute(
                Entity entity,
                ref KinematicCharacterBody2D characterBody,
                ref PlatformerCharacterControl2D control,
                ref PlatformerCharacterState2D state,
                in RopeSwingState2D ropeState,
                in PlatformerCharacterTuning2D tuning,
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

                // A platformer's up is fixed at world +Y; set it on the live body before Initialize so the snapshot
                // mirrors it and the processor's UpdateGroundingUp is a no-op.
                float2 groundingUp = new float2(0f, 1f);
                characterBody.GroundingUp = groundingUp;

                float2 gravity = new float2(0f, -tuning.GravityMagnitude);

                // --- Control → velocity: the stance machine (the sample's character logic, on the live body) --------
                //
                // GroundMove / AirMove share the grounded-vs-airborne split of the SideScroller; the stance only
                // decides WHICH block is primary. RopeSwing is a P4 stub that falls through to AirMove for now.
                switch (state.Stance)
                {
                    case PlatformerStance2D.GroundMove:
                        SolveGroundMove(ref characterBody, ref control, in tuning, groundingUp, gravity, DeltaTime, in FrictionModifierLookup);
                        break;

                    case PlatformerStance2D.RopeSwing:
                        // P4: replace this fall-through with the pendulum swing — gravity + StandardAirMove +
                        // ApplyDragToVelocity + ConstrainToRope2D(ref position, ref velocity, ropeState.RopeLength,
                        // ropeState.AnchorPoint, ...) against `ropeState`, with grounding suppressed for the swing.
                        // Until then a rope-stanced character behaves as AirMove (gravity + air control) rather than
                        // freezing, and the AirMove↔RopeSwing transitions (P4) never set this stance, so it is unreached
                        // in P2. `ropeState` is read here only to keep it on the job's Execute (and thus in the query).
                        _ = ropeState.RopeLength;
                        SolveAirMove(ref characterBody, ref control, in tuning, groundingUp, gravity, DeltaTime, in FrictionModifierLookup);
                        break;

                    case PlatformerStance2D.AirMove:
                    default:
                        SolveAirMove(ref characterBody, ref control, in tuning, groundingUp, gravity, DeltaTime, in FrictionModifierLookup);
                        break;
                }

                // Snapshot the was-grounded state AFTER the control/jump logic, so a jump (which set IsGrounded = false
                // above) makes WasGroundedBeforeCharacterUpdate = false in the snapshot the processor's grounding
                // callbacks read. This is the load-bearing ordering the default solve and the FIXED SideScroller use:
                // the solve's Default_IsGroundedOnHit only invokes the launch-protecting guard
                // (ShouldPreventGroundingBasedOnVelocity) when WasGrounded is false. Setting it BEFORE the jump (the
                // original sample bug) left it true, the guard never fired, the grounding step re-grounded the
                // character on the floor still directly below it (MovePosition has a one-step latency), and the +Y jump
                // velocity was projected onto the flat ground plane and zeroed — the character never left the ground.
                characterBody.WasGroundedBeforeCharacterUpdate = characterBody.IsGrounded;

                // --- The package solve over the sample processor -----------------------------------------------

                var processor = new PlatformerCharacterProcessor
                {
                    CharacterBodySnapshot = characterBody,
                    CharacterProperties = characterProperties,
                    StepAndSlopeHandling = stepAndSlopeHandling,
                    CharacterEntity = entity,
                };

                var userContext = new PlatformerCharacterUpdateContext
                {
                    FrictionModifierLookup = FrictionModifierLookup,
                };

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

            /// <summary>
            /// GroundMove stance: when grounded, accelerate along the ground line at the tuned ground speed — scaled by
            /// the <see cref="FrictionModifier2D"/> read off the ground hit entity (the "different physics materials"
            /// feel) — and consume a latched jump; when not grounded (walked off a ledge), fall through to the AirMove
            /// block so the character keeps moving under gravity + air control.
            /// </summary>
            static void SolveGroundMove(
                ref KinematicCharacterBody2D characterBody,
                ref PlatformerCharacterControl2D control,
                in PlatformerCharacterTuning2D tuning,
                float2 groundingUp,
                float2 gravity,
                float deltaTime,
                in ComponentLookup<FrictionModifier2D> frictionModifierLookup)
            {
                if (!characterBody.IsGrounded)
                {
                    // Walked off the ground line: behave as AirMove until the grounding step or a transition re-grounds.
                    SolveAirMove(ref characterBody, ref control, in tuning, groundingUp, gravity, deltaTime, in frictionModifierLookup);
                    return;
                }

                float2 targetMove = new float2(control.MoveX, 0f);

                // Different physics materials: scale the ground acceleration by the friction modifier on the surface the
                // character stands on. The kinematic controller is material-blind to Box2D friction (it computes its own
                // velocity), so this component is how ice (low → slow to gain/shed speed) and sticky (high) feel. Read
                // off the ground hit's owning entity; absent ⇒ neutral 1.
                float frictionScale = ReadFrictionScale(in frictionModifierLookup, characterBody.GroundHit.Entity);

                // Accelerate horizontally along the GROUND LINE so the character climbs slopes at the move speed; the
                // friction scale modulates how sharply it reaches that speed (the 3D CharacterFrictionModifier approach).
                CharacterControlUtilities2D.StandardGroundMove_Accelerated(
                    ref characterBody.RelativeVelocity,
                    targetMove * (tuning.GroundAcceleration * frictionScale),
                    tuning.GroundMoveSpeed,
                    deltaTime,
                    groundingUp,
                    characterBody.GroundHit.Normal,
                    forceNoMaxSpeedExcess: false);

                // Consume a latched jump: unground, cancel downward velocity, add the jump impulse. StandardJump sets
                // IsGrounded = false on the live body (so the post-block WasGrounded snapshot reads false).
                if (control.JumpPressed)
                {
                    CharacterControlUtilities2D.StandardJump(
                        ref characterBody,
                        groundingUp * tuning.JumpSpeed,
                        cancelVelocityBeforeJump: true,
                        groundingUp);
                    control.JumpPressed = false;
                }
            }

            /// <summary>
            /// AirMove stance: gravity always (matching the verified default solve, applied only while not standing on
            /// flat ground so it does not fight the grounded state); when not grounded, a touch of horizontal air
            /// control; when a step lands the character grounded mid-stance, accelerate on the ground line and consume a
            /// jump (the SideScroller's grounded branch, so a stance left at AirMove still walks/jumps once grounded).
            /// </summary>
            static void SolveAirMove(
                ref KinematicCharacterBody2D characterBody,
                ref PlatformerCharacterControl2D control,
                in PlatformerCharacterTuning2D tuning,
                float2 groundingUp,
                float2 gravity,
                float deltaTime,
                in ComponentLookup<FrictionModifier2D> frictionModifierLookup)
            {
                float2 targetMove = new float2(control.MoveX, 0f);

                if (characterBody.IsGrounded)
                {
                    float frictionScale = ReadFrictionScale(in frictionModifierLookup, characterBody.GroundHit.Entity);

                    CharacterControlUtilities2D.StandardGroundMove_Accelerated(
                        ref characterBody.RelativeVelocity,
                        targetMove * (tuning.GroundAcceleration * frictionScale),
                        tuning.GroundMoveSpeed,
                        deltaTime,
                        groundingUp,
                        characterBody.GroundHit.Normal,
                        forceNoMaxSpeedExcess: false);

                    if (control.JumpPressed)
                    {
                        CharacterControlUtilities2D.StandardJump(
                            ref characterBody,
                            groundingUp * tuning.JumpSpeed,
                            cancelVelocityBeforeJump: true,
                            groundingUp);
                        control.JumpPressed = false;
                    }
                }
                else
                {
                    // Airborne: apply gravity, then a touch of horizontal control on the horizontal plane
                    // (movementPlaneUp = +Y). The jump latch is left set so a jump tapped just before landing fires on
                    // the next grounded step.
                    characterBody.RelativeVelocity += gravity * deltaTime;

                    CharacterControlUtilities2D.StandardAirMove(
                        ref characterBody.RelativeVelocity,
                        targetMove * tuning.AirAcceleration,
                        tuning.AirMoveSpeed,
                        groundingUp,
                        deltaTime,
                        forceNoMaxSpeedExcess: false);
                }
            }

            /// <summary>
            /// The friction multiplier on the GroundMove acceleration sharpness for the surface under the character.
            /// Reads <see cref="FrictionModifier2D.Friction"/> off the ground hit's owning entity; a surface with no
            /// modifier (or no ground hit) is neutral (1). Low values feel slippery (ice — slow to gain/shed speed),
            /// values above 1 feel sticky.
            /// </summary>
            static float ReadFrictionScale(in ComponentLookup<FrictionModifier2D> frictionModifierLookup, Entity groundEntity)
            {
                if (groundEntity != Entity.Null && frictionModifierLookup.HasComponent(groundEntity))
                {
                    return frictionModifierLookup[groundEntity].Friction;
                }

                return 1f;
            }
        }
    }
}
