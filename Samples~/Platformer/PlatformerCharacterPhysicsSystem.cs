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
                .WithAllRW<RopeSwingState2D>()
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
                ref RopeSwingState2D ropeState,
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

                // --- Stance transitions (P4): GroundMove ↔ AirMove ↔ RopeSwing, consumed BEFORE the velocity block ---
                //
                // The 3D reference flips state inside each state's DetectTransitions (RopeSwingState.cs:91-103,
                // AirMoveState.cs grab edge); in 2D the {GroundMove, AirMove, RopeSwing} enum makes it a small block
                // here. AirMove → RopeSwing on a grab edge near an anchor; RopeSwing → AirMove on a jump/release edge.
                // GroundMove never grabs (grab is reachable only airborne, the 3D rule). Each consumed latch is cleared.
                //
                // GroundMove ↔ AirMove on the grounded edge — the faithful port of the 3D GroundMoveState
                // (DetectTransitions → AirMove when !IsGrounded, GroundMoveState.cs:133-137) and AirMoveState
                // (→ GroundMove when IsGrounded). Without this, the persistent stance was baked GroundMove and never
                // became AirMove except as a rope-EXIT destination, so a character that had never been on a rope could
                // never satisfy the grab gate's `Stance == AirMove` guard — the rope grab was unreachable in normal
                // play (a jump arc stayed labelled GroundMove the whole time). The sync runs only for the two
                // non-rope stances; RopeSwing owns its own exit (grounding is suppressed during the swing). It runs
                // BEFORE the grab gate so the SAME step that goes airborne with a grab latched can grab.
                if (state.Stance != PlatformerStance2D.RopeSwing)
                {
                    state.Stance = characterBody.IsGrounded
                        ? PlatformerStance2D.GroundMove
                        : PlatformerStance2D.AirMove;
                }

                if (state.Stance == PlatformerStance2D.RopeSwing)
                {
                    // Exit the rope on jump or release; either edge returns the character to AirMove carrying its
                    // current swing velocity (the tangential momentum becomes the launch arc — the whole point of a
                    // rope swing). On a JUMP edge, add the jump impulse here: the 2D SolveAirMove only jumps on its
                    // grounded branch (no coyote/double jump), and the character is airborne off the rope, so the
                    // impulse must be applied at the exit. This mirrors the 3D, whose AirMoveState applies an airborne
                    // StandardJump on JumpPressed (AirMoveState.cs:57-69); jumping off a rope launches immediately.
                    if (control.JumpPressed)
                    {
                        CharacterControlUtilities2D.StandardJump(
                            ref characterBody,
                            groundingUp * tuning.JumpSpeed,
                            cancelVelocityBeforeJump: false,
                            groundingUp);
                        control.JumpPressed = false;
                        state.Stance = PlatformerStance2D.AirMove;
                    }
                    else if (control.ReleasePressed)
                    {
                        // Plain release: let go, keep the swing velocity, no extra impulse.
                        control.ReleasePressed = false;
                        state.Stance = PlatformerStance2D.AirMove;
                    }
                }
                else if (state.Stance == PlatformerStance2D.AirMove && control.GrabPressed)
                {
                    // Grab: query the nearest rope anchor within RopeLength on the anchor layer. The character's own
                    // position is both the grab/detection point and the rope-attachment point (a point-mass pendulum;
                    // LocalRopeAnchorPoint = 0, the simplest faithful 2D reduction of the 3D local-offset attach). The
                    // captured RopeLength is the tuned length (the 3D RopeLength used for both detection and the
                    // constraint), so the rope is slack until the character swings out to full extension.
                    if (PlatformerRopeMath.TryDetectRopeAnchor(
                            BaseContext.PhysicsWorld,
                            characterContext.CurrentPosition,
                            tuning.RopeLength,
                            tuning.RopeAnchorLayerMask,
                            BaseContext.TmpQueryHits,
                            out Entity anchorEntity,
                            out float2 anchorPoint))
                    {
                        ropeState.Anchor = anchorEntity;
                        ropeState.AnchorPoint = anchorPoint;
                        ropeState.RopeLength = tuning.RopeLength;
                        state.Stance = PlatformerStance2D.RopeSwing;
                    }

                    control.GrabPressed = false;
                }

                // --- Control → velocity: the stance machine (the sample's character logic, on the live body) --------
                //
                // GroundMove / AirMove share the grounded-vs-airborne split of the SideScroller; the stance only
                // decides WHICH block is primary. RopeSwing runs the pendulum (gravity + air control + drag +
                // ConstrainToRope2D) with grounding suppressed.
                switch (state.Stance)
                {
                    case PlatformerStance2D.GroundMove:
                        SolveGroundMove(ref characterBody, ref control, in tuning, groundingUp, gravity, DeltaTime, in FrictionModifierLookup);
                        break;

                    case PlatformerStance2D.RopeSwing:
                        // Suppress grounding for the swing — the 2D analogue of the 3D OnStateEnter setting
                        // characterProperties.EvaluateGrounding = false (RopeSwingState.cs:18). The grounding step
                        // reads characterContext.CharacterProperties.EvaluateGrounding (KinematicCharacterUtilities2D
                        // .Update_Grounding:327), so clearing it here skips all ground detection / snapping for this
                        // step, leaving the pendulum free to swing without snapping to a floor below the arc.
                        characterContext.CharacterProperties.EvaluateGrounding = false;
                        SolveRopeSwing(ref characterContext, ref characterBody, in control, in ropeState, in tuning, gravity, DeltaTime);
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
                    // Read the EFFECTIVE properties off the context, not the raw `in characterProperties`: the RopeSwing
                    // stance set EvaluateGrounding = false on characterContext.CharacterProperties to suppress grounding
                    // for the swing, so the processor snapshot the callbacks read must match. For GroundMove / AirMove
                    // the context properties are the unmutated authored ones, so this is identical.
                    CharacterProperties = characterContext.CharacterProperties,
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

                // Interpolate horizontal velocity toward the desired ground velocity along the GROUND LINE so the
                // character climbs slopes at the move speed AND decelerates to a stop when input is released (with no
                // input the desired velocity is zero). The friction scale modulates the sharpness, so ice (low) is
                // slow to both reach speed and stop while sticky (high) is snappy — the 3D CharacterFrictionModifier
                // approach. The earlier StandardGroundMove_Accelerated only ADDED accel*dt, so a zero input added
                // nothing and the character slid forever.
                CharacterControlUtilities2D.StandardGroundMove_Interpolated(
                    ref characterBody.RelativeVelocity,
                    targetMove * tuning.GroundMoveSpeed,
                    tuning.GroundedMovementSharpness * frictionScale,
                    deltaTime,
                    groundingUp,
                    characterBody.GroundHit.Normal);

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

                    // Same grounded grip/deceleration as SolveGroundMove (a stance left at AirMove still walks/stops
                    // once a step grounds it).
                    CharacterControlUtilities2D.StandardGroundMove_Interpolated(
                        ref characterBody.RelativeVelocity,
                        targetMove * tuning.GroundMoveSpeed,
                        tuning.GroundedMovementSharpness * frictionScale,
                        deltaTime,
                        groundingUp,
                        characterBody.GroundHit.Normal);

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
            /// RopeSwing stance: the pendulum. Runs the same three forces as the 3D reference's
            /// <c>RopeSwingState.OnStatePhysicsUpdate</c> (REF3D RopeSwingState.cs:43-58), in order — rope air control,
            /// gravity, drag — then constrains the body to the rope. The pendulum motion is emergent: gravity pulls the
            /// body down, <see cref="PlatformerRopeMath.ConstrainToRope2D"/> clamps it onto the rope-length circle and
            /// projects out the radial (rope-stretching) velocity, leaving only the tangential swing. NOT a joint.
            ///
            /// <para>The position clamp lands on <see cref="KinematicCharacterContext2D.CurrentPosition"/> — the
            /// pre-solve tracked position the package's <c>PhysicsUpdate2D</c> reads as the starting
            /// <c>characterPosition</c> and delivers via the single per-step <c>MovePosition</c>. So the rope clamp is a
            /// pre-adjustment of that one verified motion-drive (design D3/D6), NOT a new pose-delivery path: the normal
            /// collide-and-slide then runs from the clamped start and resolves any world collision along the swing arc.
            /// Grounding is already suppressed by the caller (EvaluateGrounding = false on the context), the 2D analogue
            /// of the 3D OnStateEnter (RopeSwingState.cs:18).</para>
            ///
            /// <para>The character is treated as a point-mass pendulum: its own position is its rope-attachment point
            /// (LocalRopeAnchorPoint = 0), so the attachment point and the tracked position coincide. Clamping the
            /// attachment point therefore clamps the tracked position directly.</para>
            /// </summary>
            static void SolveRopeSwing(
                ref KinematicCharacterContext2D characterContext,
                ref KinematicCharacterBody2D characterBody,
                in PlatformerCharacterControl2D control,
                in RopeSwingState2D ropeState,
                in PlatformerCharacterTuning2D tuning,
                float2 gravity,
                float deltaTime)
            {
                float2 groundingUp = characterBody.GroundingUp;

                // Rope air control — accelerate the tangential swing toward the input direction, capped at the rope
                // swing max speed (the 3D StandardAirMove with RopeSwingAcceleration / RopeSwingMaxSpeed). The move is
                // taken on the movement plane (horizontal, +Y up); MoveX is already in-plane, so no extra projection.
                float2 targetMove = new float2(control.MoveX, 0f);

                CharacterControlUtilities2D.StandardAirMove(
                    ref characterBody.RelativeVelocity,
                    targetMove * tuning.RopeSwingAcceleration,
                    tuning.RopeSwingMaxSpeed,
                    groundingUp,
                    deltaTime,
                    forceNoMaxSpeedExcess: false);

                // Gravity — the pendulum's driving force (the 3D AccelerateVelocity with CustomGravity.Gravity).
                CharacterControlUtilities2D.AccelerateVelocity(ref characterBody.RelativeVelocity, gravity, deltaTime);

                // Drag — bleeds energy out of the swing so it settles rather than swinging forever (the 3D
                // ApplyDragToVelocity with RopeSwingDrag).
                CharacterControlUtilities2D.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, tuning.RopeSwingDrag);

                // The rope constraint — clamp the tracked position onto the rope-length circle and project out the
                // radial velocity. The character is a point-mass pendulum, so its position IS its rope-attachment point:
                // snapshot the attachment point before the clamp moves the position (the by-value `anchorOnCharacter`
                // is the pre-clamp point, the `ref position` is moved onto the circle).
                float2 anchorOnCharacter = characterContext.CurrentPosition;
                PlatformerRopeMath.ConstrainToRope2D(
                    ref characterContext.CurrentPosition,
                    ref characterBody.RelativeVelocity,
                    ropeState.RopeLength,
                    ropeState.AnchorPoint,
                    anchorOnCharacter);
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
