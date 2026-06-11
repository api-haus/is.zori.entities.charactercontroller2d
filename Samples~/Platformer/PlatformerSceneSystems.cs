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
    /// <c>PhysicsBody2DCommands.MovePosition</c>. Runs <c>[UpdateBefore(PhysicsWorld2DSystem)]</c> so the substrate
    /// drains the move and steps the platform THIS frame (a platform is a DRIVEN kinematic body, not a solved one —
    /// unlike the character, whose move applies next frame via the read-after-step pipeline). The substrate's
    /// <c>TrackedTransformSystem2D</c> then records the platform's just-stepped pose
    /// <c>[UpdateAfter(PhysicsWorld2DSystem)]</c>, and the character's solve (also after the step) carries itself with
    /// that one-fixed-step platform delta (<c>Update_ParentMovement</c>) — the moving-platform feature the C4b gate
    /// verified.
    ///
    /// <para>This generalizes the SideScroller's lateral-only mover: the oscillation is a per-axis sine on
    /// <see cref="MovingPlatform2D.TravelHalfExtent"/> around the captured <see cref="MovingPlatform2D.Home"/>, so a
    /// platform travels laterally, vertically, or diagonally by which half-extent components are non-zero. Sine keeps
    /// the velocity continuous across the reversal (no rider-jerking instantaneous reversal a triangle wave gives).</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(PhysicsWorld2DSystem))]
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

            foreach (var (platform, commands, ltw) in
                     SystemAPI.Query<RefRW<MovingPlatform2D>, DynamicBuffer<PhysicsBody2DCommand>, RefRO<LocalToWorld>>()
                         .WithAll<Simulate>())
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

            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<MovingPlatform2D>>().WithNone<TrackedTransform2D>().WithEntityAccess())
            {
                ecb.AddComponent(entity, new TrackedTransform2D());
            }

            // The command buffer is added in a separate pass (WithNone keyed on the buffer type) so a platform missing
            // both gets both, and a platform missing only one gets only the one.
            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<MovingPlatform2D>>().WithNone<PhysicsBody2DCommand>().WithEntityAccess())
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

            foreach (var (_, entity) in
                     SystemAPI.Query<RefRO<Pushable2D>>().WithNone<PhysicsBody2DCommand>().WithEntityAccess())
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
    /// <c>[UpdateAfter(PhysicsWorld2DSystem)]</c> because the substrate trigger-event buffer is valid only against the
    /// just-stepped world (the same read window the character solve uses). It reads
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
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
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
                        ecb.SetComponent(visitor, new InWindZone2D { Zone = ev.triggerEntity, Force = force });
                    else
                        ecb.AddComponent(visitor, new InWindZone2D { Zone = ev.triggerEntity, Force = force });
                }
                else // End
                {
                    // Only clear if the character is leaving the SAME zone it last entered (a character could overlap
                    // two zones; the latest Begin wins, and an End from the other zone must not strip the active one).
                    if (_inZoneLookup.HasComponent(visitor) && _inZoneLookup[visitor].Zone == ev.triggerEntity)
                        ecb.RemoveComponent<InWindZone2D>(visitor);
                }
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

            // --- Stay: apply each in-zone character's cached force to its relative velocity this step ------------
            foreach (var (inZone, entity) in
                     SystemAPI.Query<RefRO<InWindZone2D>>().WithAll<PlatformerCharacterTag>().WithEntityAccess())
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
    // Teleporters — best-effort instantaneous teleport on a character's Enter.
    // =============================================================================================================

    /// <summary>
    /// Teleports a <see cref="PlatformerCharacterTag"/> character to a <see cref="Teleporter2D.Destination"/>'s world
    /// position on a trigger Begin. Runs <c>[UpdateAfter(PhysicsWorld2DSystem)]</c> (the trigger buffer is valid only
    /// against the just-stepped world). Reads the substrate trigger-event buffer; for each Begin where the trigger is a
    /// <see cref="Teleporter2D"/> and the visitor is a platformer character, it performs a BEST-EFFORT teleport.
    ///
    /// <para><b>Best-effort, because the substrate has no instantaneous <c>SetTransform</c> (design teleport decision A
    /// was not built — the binding exposes only the swept <c>SetTransformTarget</c>). The teleport therefore does three
    /// things:</b>
    /// <list type="number">
    /// <item>writes the destination into the character's <see cref="LocalToWorld"/> (the pose the next fixed-step solve
    /// reads with <c>ReadPoseFromLocalToWorld</c>), so the character's collide-and-slide starts from the destination;</item>
    /// <item>enqueues a swept <c>MovePosition(destination)</c> on the character's command buffer, so the body's
    /// authoritative Box2D pose is driven toward the destination;</item>
    /// <item>resets <c>PhysicsBody2DSmoothing.hasPrev = 0</c> so the render-rate smoothing writes the new pose with no
    /// interpolation streak — the 2D analogue of the 3D <c>CharacterInterpolation.SkipNextInterpolation()</c>.</item>
    /// </list>
    /// It also zeroes <c>RelativeVelocity</c> so no pre-teleport momentum carries through.</para>
    ///
    /// <para>// P7-PROBE: the OPEN QUESTION this best-effort form cannot answer without running is whether a kinematic
    /// body driven to a FAR <c>MovePosition</c> target REACHES it in one step, or STOPS at obstructions along the swept
    /// path. <c>MovePosition</c> maps to Box2D's swept <c>SetTransformTarget</c> — for a near target the sweep is short
    /// and lands, but a teleport across the level sweeps through the whole world. A kinematic body generally pushes
    /// through collisions (infinite mass, no solver reaction) and should reach the target generating contact events en
    /// route, in which case this best-effort teleport is correct and sample-level. If instead it stops at an
    /// obstruction, teleport is a genuine binding gap and is flagged for a substrate <c>SetTransform</c> addition
    /// (decision A). This is VERIFIED at the P7 behavioural gate, not blocked on here. The <see cref="LocalToWorld"/>
    /// write (step 1) makes the NEXT solve start from the destination regardless, so even if the body's swept move
    /// undershoots this step, the character's solve position is already at the destination — the residual risk is a
    /// one-step disagreement between the body pose and the solve pose, observed at P7.</para>
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsWorld2DSystem))]
    public partial struct TeleporterSystem2D : ISystem
    {
        ComponentLookup<Teleporter2D> _teleporterLookup;
        ComponentLookup<PlatformerCharacterTag> _characterTagLookup;
        ComponentLookup<KinematicCharacterBody2D> _bodyLookup;
        ComponentLookup<LocalToWorld> _localToWorldLookup;
        ComponentLookup<PhysicsBody2DSmoothing> _smoothingLookup;
        BufferLookup<PhysicsBody2DCommand> _commandLookup;

        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<Teleporter2D>();
            state.RequireForUpdate<PhysicsWorldSingleton2D>();

            _teleporterLookup = state.GetComponentLookup<Teleporter2D>(true);
            _characterTagLookup = state.GetComponentLookup<PlatformerCharacterTag>(true);
            _bodyLookup = state.GetComponentLookup<KinematicCharacterBody2D>(false);
            _localToWorldLookup = state.GetComponentLookup<LocalToWorld>(false);
            _smoothingLookup = state.GetComponentLookup<PhysicsBody2DSmoothing>(false);
            _commandLookup = state.GetBufferLookup<PhysicsBody2DCommand>(false);
        }

        public void OnUpdate(ref SystemState state)
        {
            _teleporterLookup.Update(ref state);
            _characterTagLookup.Update(ref state);
            _bodyLookup.Update(ref state);
            _localToWorldLookup.Update(ref state);
            _smoothingLookup.Update(ref state);
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

                // (2) Drive the BODY: enqueue a swept MovePosition(destination) so Box2D moves the authoritative pose.
                if (_commandLookup.HasBuffer(visitor))
                {
                    DynamicBuffer<PhysicsBody2DCommand> commands = _commandLookup[visitor];
                    PhysicsBody2DCommands.MovePosition(commands, target);
                }

                // (3) Skip interpolation: reset hasPrev so the smoothing system writes the new pose with no streak.
                if (_smoothingLookup.HasComponent(visitor))
                {
                    PhysicsBody2DSmoothing smoothing = _smoothingLookup[visitor];
                    smoothing.hasPrev = 0;
                    _smoothingLookup[visitor] = smoothing;
                }

                // Zero pre-teleport momentum so the character does not carry velocity through the teleport.
                if (_bodyLookup.HasComponent(visitor))
                {
                    KinematicCharacterBody2D body = _bodyLookup[visitor];
                    body.RelativeVelocity = new float2(0f, 0f);
                    _bodyLookup[visitor] = body;
                }
            }
        }
    }
}
