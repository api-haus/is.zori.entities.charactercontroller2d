using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    // =============================================================================================================
    // Moving platforms — the SideScroller's verified pattern, generalized to BOTH axes.
    // =============================================================================================================

    /// <summary>
    /// Drives each <see cref="MovingPlatform2D"/> along its per-axis oscillation each fixed step via a swept
    /// <c>PhysicsBody2DCommands.MovePosition</c>. Runs <c>[UpdateBefore(Physics2DSimulationSystemGroup)]</c> so the
    /// substrate drains the move and steps the platform THIS frame (a platform is a DRIVEN kinematic body, not a
    /// solved one — unlike the character, whose move applies next frame via the read-after-step pipeline). The
    /// substrate's <c>TrackedTransformSystem2D</c> then records the platform's just-stepped pose
    /// <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c>, and the character's solve (also after the step) carries
    /// itself with
    /// that one-fixed-step platform delta (<c>Update_ParentMovement</c>) — the moving-platform feature the C4b gate
    /// verified.
    ///
    /// <para>This generalizes the SideScroller's lateral-only mover: the oscillation is a per-axis sine on
    /// <see cref="MovingPlatform2D.TravelHalfExtent"/> around the captured <see cref="MovingPlatform2D.Home"/>, so a
    /// platform travels laterally, vertically, or diagonally by which half-extent components are non-zero. Sine keeps
    /// the velocity continuous across the reversal (no rider-jerking instantaneous reversal a triangle wave gives).</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(Physics2DSimulationSystemGroup))]
    [BurstCompile]
    public partial struct MovingPlatformSystem2D : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<MovingPlatform2D>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            float dt = SystemAPI.Time.DeltaTime;

            foreach (
                var (platform, commands, ltw) in SystemAPI
                    .Query<
                        RefRW<MovingPlatform2D>,
                        DynamicBuffer<PhysicsBody2DCommand>,
                        RefRO<LocalToWorld>
                    >()
                    .WithAll<Simulate>()
            )
            {
                // Capture the home from the baked pose on the first update (so the oscillation centres on where the
                // platform was authored, not on world origin).
                if (!platform.ValueRO.HomeCaptured)
                {
                    platform.ValueRW.Home = ltw.ValueRO.Value.c3.xy;
                    platform.ValueRW.HomeCaptured = true;
                }

                platform.ValueRW.Phase += dt;

                // A per-axis sine oscillation within +/- TravelHalfExtent around Home. The angular rate is derived
                // from speed / half-extent so the peak tangential speed matches the authored Speed, taken from the
                // larger non-zero half-extent component (a zero half-extent on an axis pins that axis to Home).
                float2 half = platform.ValueRO.TravelHalfExtent;
                float span = max(abs(half.x), abs(half.y));
                float omega = span > 0f ? platform.ValueRO.Speed / span : 0f;
                float s = sin(platform.ValueRO.Phase * omega);
                float2 target = platform.ValueRO.Home + (half * s);

                PhysicsBody2DCommands.MovePosition(commands, target);
            }
        }
    }

    /// <summary>
    /// One-shot structural setup for the moving platforms: adds the <see cref="TrackedTransform2D"/> (so the controller
    /// treats the platform as a moving platform and carries the rider) and the
    /// <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> (so <see cref="MovingPlatformSystem2D"/> can drive it) to any
    /// <see cref="MovingPlatform2D"/> that lacks them. A substrate kinematic body baked from
    /// <c>PhysicsBody2DAuthoring</c> has neither (no baker authors a tracked-transform, and the command buffer is the
    /// body owner's responsibility) — this is the runtime-add pattern the C4b gate flagged, identical to the
    /// SideScroller's. Runs in the <see cref="InitializationSystemGroup"/> (a structural change, off the fixed-step hot
    /// path) and uses an <see cref="EntityCommandBuffer"/> so the add happens at a sync point, never mid-query.
    /// </summary>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct PlatformInitSystem2D : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<MovingPlatform2D>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (
                var (_, entity) in SystemAPI
                    .Query<RefRO<MovingPlatform2D>>()
                    .WithNone<TrackedTransform2D>()
                    .WithEntityAccess()
            )
            {
                ecb.AddComponent(entity, new TrackedTransform2D());
            }

            // The command buffer is added in a separate pass (WithNone keyed on the buffer type) so a platform missing
            // both gets both, and a platform missing only one gets only the one.
            foreach (
                var (_, entity) in SystemAPI
                    .Query<RefRO<MovingPlatform2D>>()
                    .WithNone<PhysicsBody2DCommand>()
                    .WithEntityAccess()
            )
            {
                ecb.AddBuffer<PhysicsBody2DCommand>(entity);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }

    // =============================================================================================================
    // Pushable crates — the SideScroller's pattern (the command buffer the deferred-impulse push needs).
    // =============================================================================================================

    /// <summary>
    /// One-shot structural setup for the pushable crates: adds the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> to
    /// any <see cref="Pushable2D"/> body that lacks it. The controller's deferred-impulse system pushes a regular
    /// dynamic body only <c>if HasBuffer(target)</c> — and a <c>PhysicsBody2DAuthoring</c> crate is baked WITHOUT a
    /// command buffer (only the controller's own baker adds one, to the character). So out of the box the character's
    /// mass-scaled push of a crate is silently dropped; this system closes that gap for the tagged crates. The
    /// single most important integration fact from the C4b gate, carried forward from the SideScroller.
    /// </summary>
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct PushableInitSystem2D : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<Pushable2D>();
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            foreach (
                var (_, entity) in SystemAPI
                    .Query<RefRO<Pushable2D>>()
                    .WithNone<PhysicsBody2DCommand>()
                    .WithEntityAccess()
            )
            {
                ecb.AddBuffer<PhysicsBody2DCommand>(entity);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }

    // =============================================================================================================
    // Wind / force zones — the substrate trigger-event channel, NOT effectors.
    // =============================================================================================================

    /// <summary>
    /// Internal sample tag recording that a character is currently inside a wind zone — the consumer-derived "Stay"
    /// the Box2D-v3 begin/end event model needs. The substrate trigger buffer carries only Begin/End (no per-frame
    /// Stay), so <see cref="WindZoneSystem2D"/> adds this on the visitor character on Begin and removes it on End; while
    /// present it re-applies the zone's force each step. A plain <see cref="IComponentData"/>, so it shares this file
    /// with the systems (the one-per-file rule binds only <c>MonoBehaviour</c> types).
    /// </summary>
    public struct InWindZone2D : IComponentData
    {
        /// <summary>The zone entity the character entered (so an End from a DIFFERENT zone does not clear this one).</summary>
        public Entity Zone;

        /// <summary>The zone's force, cached so the per-step apply needs no second lookup.</summary>
        public float2 Force;
    }

    /// <summary>
    /// Adds a <see cref="WindZone2D"/>'s force to the kinematic character's
    /// <c>KinematicCharacterBody2D.RelativeVelocity</c> while the character is inside the zone. Runs
    /// <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> because the substrate trigger-event buffer is valid only
    /// against the just-stepped world (the same read window the character solve uses). It reads
    /// <c>DynamicBuffer&lt;PhysicsTriggerEvent2D&gt;</c> on the <see cref="PhysicsWorldSingleton2D"/> entity, derives
    /// Stay from the Begin..End interval by tracking an <see cref="InWindZone2D"/> tag on the visitor character, and
    /// while in-zone adds <c>Force * dt</c> to the character's relative velocity (then the next character solve
    /// collide-and-slides with that pushed velocity).
    ///
    /// <para>Zones that affect the kinematic character mutate <c>RelativeVelocity</c> from THIS trigger-read system, not
    /// via substrate effectors — an Area/Point effector applies solver forces to dynamic bodies only, and the kinematic
    /// character (driven by <c>RelativeVelocity</c> + <c>MovePosition</c>) is invisible to that solve. Only visitor
    /// entities carrying a <see cref="PlatformerCharacterTag"/> are affected; a loose crate falling through the sensor
    /// raises trigger events too but is not a character and is skipped here (the substrate effector is the right tool
    /// for prop-affecting force fields).</para>
    ///
    /// <para>Structural changes (add/remove the in-zone tag) go through an <see cref="EntityCommandBuffer"/> at a sync
    /// point; the per-step force write is a direct lookup write (the body component is enableable, so the write lands
    /// on the live character).</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    public partial struct WindZoneSystem2D : ISystem
    {
        ComponentLookup<WindZone2D> _windZoneLookup;
        ComponentLookup<PlatformerCharacterTag> _characterTagLookup;
        ComponentLookup<KinematicCharacterBody2D> _bodyLookup;
        ComponentLookup<InWindZone2D> _inZoneLookup;

        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<WindZone2D>();
            state.RequireForUpdate<PhysicsWorldSingleton2D>();

            _windZoneLookup = state.GetComponentLookup<WindZone2D>(true);
            _characterTagLookup = state.GetComponentLookup<PlatformerCharacterTag>(true);
            _bodyLookup = state.GetComponentLookup<KinematicCharacterBody2D>(false);
            _inZoneLookup = state.GetComponentLookup<InWindZone2D>(true);
        }

        public void OnUpdate(ref SystemState state)
        {
            // The Stay-force loop writes ComponentLookup<KinematicCharacterBody2D> on the main thread; the character
            // solve job also writes it. Finish the scheduled writer before the main-thread read/write.
            state.CompleteDependency();

            _windZoneLookup.Update(ref state);
            _characterTagLookup.Update(ref state);
            _bodyLookup.Update(ref state);
            _inZoneLookup.Update(ref state);

            float dt = SystemAPI.Time.DeltaTime;
            var ecb = new EntityCommandBuffer(Allocator.Temp);

            // --- Begin/End: maintain the in-zone tag on visitor characters (the consumer-derived Stay) ----------
            var triggers = SystemAPI.GetSingletonBuffer<PhysicsTriggerEvent2D>(isReadOnly: true);
            for (int i = 0; i < triggers.Length; i++)
            {
                PhysicsTriggerEvent2D ev = triggers[i];

                // Resolve which side is the wind zone and which is the visitor. The sensor body is the trigger; the
                // character is the visitor. (A sensor pair would fire symmetric reports, but a wind-zone sensor and a
                // character collider are an asymmetric pair, so the trigger side is the zone.)
                if (!_windZoneLookup.HasComponent(ev.triggerEntity))
                    continue;

                Entity visitor = ev.visitorEntity;
                if (!_characterTagLookup.HasComponent(visitor))
                    continue;

                if (ev.phase == PhysicsEventPhase2D.Begin)
                {
                    float2 force = _windZoneLookup[ev.triggerEntity].Force;
                    if (_inZoneLookup.HasComponent(visitor))
                        ecb.SetComponent(
                            visitor,
                            new InWindZone2D { Zone = ev.triggerEntity, Force = force }
                        );
                    else
                        ecb.AddComponent(
                            visitor,
                            new InWindZone2D { Zone = ev.triggerEntity, Force = force }
                        );
                }
                else // End
                {
                    // Only clear if the character is leaving the SAME zone it last entered (a character could overlap
                    // two zones; the latest Begin wins, and an End from the other zone must not strip the active one).
                    if (
                        _inZoneLookup.HasComponent(visitor)
                        && _inZoneLookup[visitor].Zone == ev.triggerEntity
                    )
                        ecb.RemoveComponent<InWindZone2D>(visitor);
                }
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

            // The ECB playback above is a structural change (add/remove the in-zone tag), which invalidates every
            // ComponentLookup captured before it. Re-acquire the writable body lookup against the post-structural-
            // change state before the Stay-force loop reads/writes it (else: ObjectDisposedException).
            _bodyLookup.Update(ref state);

            // --- Stay: apply each in-zone character's cached force to its relative velocity this step ------------
            foreach (
                var (inZone, entity) in SystemAPI
                    .Query<RefRO<InWindZone2D>>()
                    .WithAll<PlatformerCharacterTag>()
                    .WithEntityAccess()
            )
            {
                if (!_bodyLookup.HasComponent(entity))
                    continue;
                KinematicCharacterBody2D body = _bodyLookup[entity];
                body.RelativeVelocity += inZone.ValueRO.Force * dt;
                _bodyLookup[entity] = body;
            }
        }
    }

    // =============================================================================================================
    // Teleporters — proper instantaneous teleport on a character's Enter (SetTransform + SkipInterpolation).
    // =============================================================================================================

    /// <summary>
    /// Teleports a <see cref="PlatformerCharacterTag"/> character to a <see cref="Teleporter2D.Destination"/>'s world
    /// position on a trigger Begin — a PROPER instantaneous teleport, the 2D mirror of the 3D
    /// <c>com.unity.charactercontroller</c> Platformer sample's <c>TeleporterSystem</c> (which writes
    /// <c>LocalTransform.Position</c> + calls <c>CharacterInterpolation.SkipNextInterpolation()</c>). Runs
    /// <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> (the trigger buffer is valid only against the just-stepped
    /// world). Reads the substrate trigger-event buffer; for each Begin where the trigger is a
    /// <see cref="Teleporter2D"/> and
    /// the visitor is a platformer character, it teleports the character with the two distinct substrate APIs that are
    /// the 2D analogues of the 3D pair:
    /// <list type="number">
    /// <item><b><see cref="PhysicsBody2DCommands.SetTransform"/></b> (the analogue of writing
    /// <c>LocalTransform.Position</c>) — a hard, NON-swept write to the body's <c>transform</c> (native
    /// <c>b2Body_SetTransform</c>), so the body reaches the destination in ONE step at any distance with no speed
    /// clamp. This replaces the prior best-effort swept <c>MovePosition</c>, which could undershoot a far destination
    /// because <c>SetTransformTarget</c> is velocity-based and the world clamps it to <c>maximumLinearSpeed · dt</c>.</item>
    /// <item><b><see cref="PhysicsBody2DCommands.SkipInterpolation"/></b> (the analogue of
    /// <c>CharacterInterpolation.SkipNextInterpolation()</c>) — resets the body's render-rate smoothing so the next
    /// frame draws the teleported pose with no interpolation streak from the old location.</item>
    /// </list>
    ///
    /// <para><b>The controller's solve position.</b> Unlike the 3D controller (which owns <c>LocalTransform</c>), the
    /// 2D solve re-reads its position off the character's <see cref="LocalToWorld"/> every fixed step
    /// (<c>ReadPoseFromLocalToWorld</c>) and enqueues its own move from there. So the teleport ALSO writes the
    /// destination into the character's <see cref="LocalToWorld"/>: otherwise the next solve would read the
    /// pre-teleport pose and move the character back. With both the body's <c>SetTransform</c> and the solve's
    /// <c>LocalToWorld</c> at the destination, the body and the solve agree from the teleport step onward.</para>
    ///
    /// <para>It also zeroes <c>RelativeVelocity</c> so no pre-teleport momentum carries through (a respawn-style
    /// teleport; <c>SetTransform</c> deliberately does not clear velocity itself).</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    public partial struct TeleporterSystem2D : ISystem
    {
        ComponentLookup<Teleporter2D> _teleporterLookup;
        ComponentLookup<PlatformerCharacterTag> _characterTagLookup;
        ComponentLookup<KinematicCharacterBody2D> _bodyLookup;
        ComponentLookup<LocalToWorld> _localToWorldLookup;
        BufferLookup<PhysicsBody2DCommand> _commandLookup;

        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<Teleporter2D>();
            state.RequireForUpdate<PhysicsWorldSingleton2D>();

            _teleporterLookup = state.GetComponentLookup<Teleporter2D>(true);
            _characterTagLookup = state.GetComponentLookup<PlatformerCharacterTag>(true);
            _bodyLookup = state.GetComponentLookup<KinematicCharacterBody2D>(false);
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(false);
            _commandLookup = state.GetBufferLookup<PhysicsBody2DCommand>(false);
        }

        public void OnUpdate(ref SystemState state)
        {
            // This main-thread system reads/writes ComponentLookup<LocalToWorld> (and the body lookup), both written
            // by scheduled jobs (the transform-export BatchTransformToLocalToWorldJob and the character solve). Finish
            // those before touching the lookups, or reading LocalToWorld throws (a scheduled writer is still live).
            state.CompleteDependency();

            _teleporterLookup.Update(ref state);
            _characterTagLookup.Update(ref state);
            _bodyLookup.Update(ref state);
            _localToWorldLookup.Update(ref state);
            _commandLookup.Update(ref state);

            var triggers = SystemAPI.GetSingletonBuffer<PhysicsTriggerEvent2D>(isReadOnly: true);
            for (int i = 0; i < triggers.Length; i++)
            {
                PhysicsTriggerEvent2D ev = triggers[i];
                if (ev.phase != PhysicsEventPhase2D.Begin)
                    continue;

                if (!_teleporterLookup.HasComponent(ev.triggerEntity))
                    continue;

                Entity visitor = ev.visitorEntity;
                if (!_characterTagLookup.HasComponent(visitor))
                    continue;

                Entity destination = _teleporterLookup[ev.triggerEntity].Destination;
                if (destination == Entity.Null || !_localToWorldLookup.HasComponent(destination))
                    continue; // a teleporter with no resolved destination is a no-op (never teleport to the origin)

                float2 target = _localToWorldLookup[destination].Value.c3.xy;

                // (1) Set the SOLVE position: write the destination into the character's LocalToWorld so the next
                // fixed-step solve's ReadPoseFromLocalToWorld starts the collide-and-slide from the destination.
                if (_localToWorldLookup.HasComponent(visitor))
                {
                    LocalToWorld ltw = _localToWorldLookup[visitor];
                    float4x4 m = ltw.Value;
                    m.c3.xy = target;
                    ltw.Value = m;
                    _localToWorldLookup[visitor] = ltw;
                }

                // (2) Set the BODY pose INSTANTANEOUSLY + skip interpolation, in that order on the command buffer
                // (both drain before the next step; SkipInterpolation reads the post-SetTransform pose). SetTransform
                // is the hard, unclamped b2Body_SetTransform write — it reaches any destination in one step, unlike
                // the swept MovePosition this replaces. SkipInterpolation suppresses the render-rate smoothing streak
                // (the 2D analogue of CharacterInterpolation.SkipNextInterpolation()); it is a no-op on a None-
                // interpolation body, so no separate smoothing-component check is needed here.
                if (_commandLookup.HasBuffer(visitor))
                {
                    DynamicBuffer<PhysicsBody2DCommand> commands = _commandLookup[visitor];
                    PhysicsBody2DCommands.SetTransform(commands, target, 0f);
                    PhysicsBody2DCommands.SkipInterpolation(commands);
                }

                // (3) Zero pre-teleport momentum so the character does not carry velocity through the teleport
                // (a respawn-style teleport drops momentum; SetTransform itself does not clear velocity).
                if (_bodyLookup.HasComponent(visitor))
                {
                    KinematicCharacterBody2D body = _bodyLookup[visitor];
                    body.RelativeVelocity = new float2(0f, 0f);
                    _bodyLookup[visitor] = body;
                }
            }
        }
    }

    // =============================================================================================================
    // Respawn — remember the last safe (grounded, stable) position; teleport back to it on a fall.
    // =============================================================================================================

    /// <summary>
    /// Auto-respawn to the last safe point: while a <see cref="PlatformerCharacterTag"/> character is grounded AND
    /// stable, this records its world position into <see cref="LastSafePoint2D"/>; when the character's Y falls below
    /// <see cref="PlatformerCharacterTuning2D.FallRespawnThresholdY"/> (it fell off the course), it teleports the
    /// character back to that last safe point with zeroed velocity and no interpolation streak — the same proper
    /// teleport <see cref="TeleporterSystem2D"/> uses (the destination written into <c>LocalToWorld</c> so the next
    /// solve starts there, the instantaneous unclamped <see cref="PhysicsBody2DCommands.SetTransform"/> on the body,
    /// <see cref="PhysicsBody2DCommands.SkipInterpolation"/> to suppress the render streak, and a velocity reset).
    ///
    /// <para><b>The "safe" predicate.</b> A frame is safe only when the character is grounded this step AND was
    /// grounded at the start of the step (<see cref="KinematicCharacterBody2D.WasGroundedBeforeCharacterUpdate"/>) —
    /// i.e. it has been on the ground for at least one prior step, not just landed mid-fall — AND is not mid-step-up
    /// (<see cref="KinematicCharacterBody2D.SuppressGroundSnappingUntilSteppedClear"/> false, so the recorded pose is a
    /// settled stand, not a transient step-mount pose) AND not standing on a moving platform (its ground hit is not a
    /// <see cref="MovingPlatform2D"/> body). Requiring the prior-step grounding keeps the recorded point off a one-frame
    /// graze of a ledge edge or a passing platform the character bounced off — only a sustained stand becomes a respawn
    /// target. The moving-platform exclusion keeps the safe point on a STATIC surface: a moving platform travels, so a
    /// point recorded on it would respawn the character to a stale (or mid-gap) platform pose. The character can still
    /// ride the platform and, after a fall, respawn to the last STATIC point it stood on.</para>
    ///
    /// <para><b>Ordering.</b> Runs <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c> AND
    /// <c>[UpdateAfter(PlatformerCharacterPhysicsSystem)]</c> in the fixed-step group, so it reads the just-solved
    /// grounding state and the current <c>LocalToWorld</c> pose AND — load-bearing — runs after the solve has enqueued
    /// its own swept <c>MovePosition</c> for the step. The respawn's <c>SetTransform</c> must be the LAST command on
    /// the buffer: it and the solve's <c>MovePosition</c> both drain at the next <c>PhysicsWorld2DSystem</c> in buffer
    /// order, and if the solve's swept move came after the instant set, its clamped target would sweep the body off
    /// the just-set destination (landing it ~6.67 u — the <c>maximumLinearSpeed</c> per-step cap — short). Ordering
    /// the respawn after the solve puts the instant set last, so it wins. The commands drain at the NEXT
    /// <c>PhysicsWorld2DSystem</c>. Not Bursted — a low-frequency main-thread system doing
    /// <c>ComponentLookup</c>/<c>BufferLookup</c> reads-writes after a <c>CompleteDependency</c>, mirroring
    /// <see cref="TeleporterSystem2D"/>.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    [UpdateAfter(typeof(PlatformerCharacterPhysicsSystem))]
    public partial struct PlatformerRespawnSystem : ISystem
    {
        ComponentLookup<MovingPlatform2D> _movingPlatformLookup;

        public void OnCreate(ref SystemState state)
        {
            // Inert on import: only a baked Platformer character carries the tag + the respawn state.
            state.RequireForUpdate<PlatformerCharacterTag>();
            state.RequireForUpdate<LastSafePoint2D>();

            // To exclude a moving platform from the safe-point set, check the character's ground hit entity against
            // the MovingPlatform2D marker (a moving / kinematic body is not a stable respawn anchor).
            _movingPlatformLookup = state.GetComponentLookup<MovingPlatform2D>(true);
        }

        public void OnUpdate(ref SystemState state)
        {
            // This main-thread system reads LocalToWorld and the body (both written by scheduled jobs — the
            // transform-export job and the character solve). Finish those before touching the data, or a read throws
            // because a scheduled writer is still live (the same reason TeleporterSystem2D completes here).
            state.CompleteDependency();
            _movingPlatformLookup.Update(ref state);

            foreach (
                var (body, ltw, lastSafe, tuning, commands) in SystemAPI
                    .Query<
                        RefRW<KinematicCharacterBody2D>,
                        RefRW<LocalToWorld>,
                        RefRW<LastSafePoint2D>,
                        RefRO<PlatformerCharacterTuning2D>,
                        DynamicBuffer<PhysicsBody2DCommand>
                    >()
                    .WithAll<PlatformerCharacterTag>()
            )
            {
                float2 position = ltw.ValueRO.Value.c3.xy;

                // A moving platform is NOT a stable respawn anchor — its pose oscillates, so a point recorded while
                // standing on it would respawn the character to where the platform USED to be (or where it now is,
                // mid-travel, possibly over a gap). Exclude any frame whose ground hit is a MovingPlatform2D body, so
                // the last safe point is only ever a STATIC walkable surface. The character can still ride the platform
                // and, after falling off, respawn to the last STATIC point it stood on. (Identified by the explicit
                // marker rather than raw kinematic-body-ness: every moving platform in the sample carries it, and a
                // marker check is cheaper and more precise than reading the substrate body type.)
                Entity groundEntity = body.ValueRO.GroundHit.Entity;
                bool groundedOnMovingPlatform =
                    groundEntity != Entity.Null && _movingPlatformLookup.HasComponent(groundEntity);

                // Record the last safe point while grounded AND stable: grounded this step, grounded the prior step
                // (a sustained stand, not a just-landed graze), not mid-step-up (a settled pose), ABOVE the fall
                // threshold (a pose below the fall line is by definition not safe — guarding on it stops a transient
                // still-grounded frame during a fast fall-or-teleport from poisoning the safe point with a below-course
                // position, which would then respawn the character onto its own fall), and NOT standing on a moving
                // platform (an unstable, travelling surface).
                bool safe =
                    body.ValueRO.IsGrounded
                    && body.ValueRO.WasGroundedBeforeCharacterUpdate
                    && !body.ValueRO.SuppressGroundSnappingUntilSteppedClear
                    && position.y >= tuning.ValueRO.FallRespawnThresholdY
                    && !groundedOnMovingPlatform;
                if (safe)
                {
                    lastSafe.ValueRW.Position = position;
                    lastSafe.ValueRW.HasPoint = true;
                }

                // Fell off the course: teleport back to the last safe point (only once one has been recorded — before
                // the first safe stand there is nowhere to send the character, so the threshold check is a no-op).
                if (position.y >= tuning.ValueRO.FallRespawnThresholdY || !lastSafe.ValueRO.HasPoint)
                    continue;

                float2 target = lastSafe.ValueRO.Position + new float2(0f, tuning.ValueRO.RespawnHeightOffset);

                // (1) Set the SOLVE position: write the destination into LocalToWorld so the next fixed-step solve's
                // ReadPoseFromLocalToWorld starts the collide-and-slide from the safe point (the 2D solve owns
                // LocalToWorld, not LocalTransform — miss this and the next solve walks the character back).
                LocalToWorld m = ltw.ValueRO;
                float4x4 mat = m.Value;
                mat.c3.xy = target;
                m.Value = mat;
                ltw.ValueRW = m;

                // (2) Set the BODY pose INSTANTANEOUSLY + zero its linear velocity + skip interpolation, in that order
                // on the command buffer (all drain before the next step, SkipInterpolation reads the post-SetTransform
                // pose). SetTransform is the hard, unclamped b2Body_SetTransform write — it reaches any distance in one
                // step (unlike the swept MovePosition, whose implied velocity is clamped to maximumLinearSpeed and
                // would land a far respawn ~6.67 u short). SetLinearVelocity(0) drops the falling body's leftover
                // Box2D momentum so Simulate does not drift it off the just-set pose; it is issued AFTER SetTransform
                // (the pose set carries no velocity, so a separate velocity reset is needed). This is the same
                // SetTransform + SkipInterpolation pair TeleporterSystem2D uses, plus the velocity reset a fall needs.
                PhysicsBody2DCommands.SetTransform(commands, target, 0f);
                PhysicsBody2DCommands.SetLinearVelocity(commands, new float2(0f, 0f));
                PhysicsBody2DCommands.SkipInterpolation(commands);

                // (3) Zero the controller's relative velocity too, so the fall momentum does not carry through.
                body.ValueRW.RelativeVelocity = new float2(0f, 0f);
            }
        }
    }
}
