using Unity.Entities;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    // ---- character intent / identity ------------------------------------------------------------------------

    /// <summary>
    /// One frame of platformer intent, written by the control system from input and consumed by the fixed-step
    /// physics solve. The 2D Platformer analogue of the SideScroller's <c>CharacterControl2D</c>, extended with the
    /// two rope edges the RopeSwing stance needs. The control system fills it on the regular (render-rate) update,
    /// the Bursted fixed-step solve reads it and turns it into the character's
    /// <see cref="KinematicCharacterBody2D.RelativeVelocity"/> and stance transitions.
    ///
    /// <para><see cref="JumpPressed"/>, <see cref="GrabPressed"/>, and <see cref="ReleasePressed"/> are latched edges
    /// (set on the input frame, cleared by the consuming system) so an edge tapped between two fixed steps is never
    /// dropped nor double-applied across the differing input vs fixed-step rates.</para>
    /// </summary>
    public struct PlatformerCharacterControl2D : IComponentData
    {
        /// <summary>Desired horizontal move in [-1, 1] (left/right), in world space. Zero means stand still.</summary>
        public float MoveX;

        /// <summary>Latched jump edge — true when a jump was requested and not yet consumed. Drives a grounded jump in
        /// GroundMove, and the RopeSwing → AirMove release-by-jump transition.</summary>
        public bool JumpPressed;

        /// <summary>Latched rope-grab edge — true when a grab was requested and not yet consumed. In AirMove, triggers
        /// the anchor-detection query and the AirMove → RopeSwing transition when an anchor is in range.</summary>
        public bool GrabPressed;

        /// <summary>Latched rope-release edge — true when a release was requested and not yet consumed. In RopeSwing,
        /// triggers the RopeSwing → AirMove transition (the no-jump way off the rope).</summary>
        public bool ReleasePressed;
    }

    /// <summary>
    /// Marks a character as driven by the Platformer sample's control + physics + stance systems. As with the
    /// SideScroller, the sample ships its own worked solve over the Platformer processor rather than the package's
    /// <see cref="DefaultCharacterController2DTag"/>; the sample systems gate on this tag (<c>RequireForUpdate</c>) so
    /// a bare package import — which bakes no Platformer markers — runs nothing.
    /// </summary>
    public struct PlatformerCharacterTag : IComponentData { }

    // ---- per-character tuning (NOT a scene config singleton) ------------------------------------------------

    /// <summary>
    /// Per-character movement tuning, baked onto the character entity and read per-entity in the solve. This is the
    /// coordinator correction to the SideScroller's <c>const</c>-on-the-physics-system tuning (gravity 20, speed 7,
    /// accels 90/30, jump 9 buried as constants): each Platformer character carries its OWN tuning, so two characters
    /// in the same scene can differ in gravity / speed / jump. There is deliberately NO scene-ambient config singleton
    /// holding these — movement tuning is a per-character property, not a scene-global resource.
    /// </summary>
    public struct PlatformerCharacterTuning2D : IComponentData
    {
        /// <summary>Downward gravity acceleration magnitude (units/s²), applied every step in every stance.</summary>
        public float GravityMagnitude;

        /// <summary>Target horizontal speed on the ground line in GroundMove (units/s).</summary>
        public float GroundMoveSpeed;

        /// <summary>
        /// Sharpness of the interpolation toward the desired ground velocity (higher = snappier). With no move input
        /// the desired velocity is zero, so this is the ground grip that decelerates the character to a stop. It is
        /// scaled per-surface by <see cref="FrictionModifier2D"/> (low → slippery ice, slow to reach speed AND slow to
        /// stop; high → sticky, snappy both ways). A pure acceleration model never decelerates toward a zero target,
        /// so a released character slid forever — the 3D ThirdPerson sample uses this interpolated form for grip.
        /// </summary>
        public float GroundedMovementSharpness;

        /// <summary>Target horizontal speed of air control in AirMove (units/s).</summary>
        public float AirMoveSpeed;

        /// <summary>How sharply air velocity approaches the AirMove target (acceleration sharpness).</summary>
        public float AirAcceleration;

        /// <summary>Initial upward speed imparted by a grounded jump (units/s).</summary>
        public float JumpSpeed;

        // ---- rope tuning ----

        /// <summary>The rope's length (units): both the max distance the grab query reaches for an anchor and the
        /// radius of the circle the swing constrains the character onto. Matches the 3D reference's single
        /// <c>RopeLength</c> used for both detection and the constraint (REF3D RopeSwingState). The rope is slack —
        /// no clamp — until the character swings out to this full extension.</summary>
        public float RopeLength;

        /// <summary>Target tangential speed of air control while swinging in RopeSwing (units/s).</summary>
        public float RopeSwingMaxSpeed;

        /// <summary>How sharply rope-swing air control approaches its target (acceleration sharpness).</summary>
        public float RopeSwingAcceleration;

        /// <summary>Velocity drag applied each step while swinging, bleeding energy out of the pendulum.</summary>
        public float RopeSwingDrag;

        /// <summary>Collision-layer mask the RopeSwing grab query (<c>OverlapCircle</c>) filters rope anchors by — only
        /// shapes on these layers are candidate anchors.</summary>
        public ulong RopeAnchorLayerMask;
    }

    // ---- stance state ---------------------------------------------------------------------------------------

    /// <summary>
    /// The Platformer's three movement stances. Unlike the 3D Platformer's 12-state polymorphic state machine, the 2D
    /// sample needs only these three — the minimal set the five named features (moving platforms, zones, materials,
    /// teleports, rope swings) require. Dispatched by a <c>switch</c> on
    /// <see cref="PlatformerCharacterState2D.Stance"/> in the solve, not a per-state interface dispatch.
    /// </summary>
    public enum PlatformerStance2D : byte
    {
        /// <summary>Accelerated horizontal move on the ground line + grounded jump; friction-modifier-aware.</summary>
        GroundMove = 0,

        /// <summary>Air control + gravity while airborne; the transition source into RopeSwing and the destination on
        /// jump/release off the rope.</summary>
        AirMove = 1,

        /// <summary>Pendulum motion by position-clamp-to-rope-length + inward-velocity-projection, fed through the
        /// existing collide-and-slide. Entered from AirMove on grab near an anchor; exited to AirMove on jump/release.</summary>
        RopeSwing = 2,
    }

    /// <summary>
    /// The character's current stance, the one piece of persistent state the stance machine carries between steps. The
    /// solve reads <see cref="Stance"/> to choose the velocity-control block run before <c>PhysicsUpdate2D</c>, and the
    /// transition logic writes it on a grab / jump / release edge.
    /// </summary>
    public struct PlatformerCharacterState2D : IComponentData
    {
        /// <summary>The stance currently driving the character's velocity-control block.</summary>
        public PlatformerStance2D Stance;
    }

    /// <summary>
    /// The active rope's parameters, valid only while <see cref="PlatformerCharacterState2D.Stance"/> is
    /// <see cref="PlatformerStance2D.RopeSwing"/>. Written by the AirMove → RopeSwing transition when the grab query
    /// finds an anchor in range, and read by the RopeSwing velocity-control block to clamp the character onto the
    /// rope-length circle and project out the inward velocity component (the dimension-reduced 3D <c>ConstrainToRope</c>).
    /// </summary>
    public struct RopeSwingState2D : IComponentData
    {
        /// <summary>The grabbed anchor entity (the static collider on the rope-anchor layer the grab query selected).</summary>
        public Entity Anchor;

        /// <summary>The anchor's world-space pivot point — the centre of the rope-length circle the swing clamps to.</summary>
        public float2 AnchorPoint;

        /// <summary>The rope length: the radius of the circle the character is constrained onto while swinging.</summary>
        public float RopeLength;
    }

    // ---- feature prop markers -------------------------------------------------------------------------------

    /// <summary>
    /// Marks a kinematic body the Platformer drives as a moving platform. Generalizes the SideScroller's
    /// lateral-only <c>SideScrollerMovingPlatform</c> to support both lateral and vertical travel. A platform-init
    /// system adds the <c>TrackedTransform2D</c> + the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> at runtime (no
    /// baker authors them), and a mover system drives it <c>[UpdateBefore(PhysicsWorld2DSystem)]</c> via
    /// <c>PhysicsBody2DCommands.MovePosition</c> so the platform steps THIS frame and the rider is carried.
    /// </summary>
    public struct MovingPlatform2D : IComponentData
    {
        /// <summary>The platform's home position (the centre of its oscillation), captured at first update.</summary>
        public float2 Home;

        /// <summary>Whether <see cref="Home"/> has been captured yet (false until the first update reads the pose).</summary>
        public bool HomeCaptured;

        /// <summary>Travel half-extents per axis: the platform oscillates within ±this around <see cref="Home"/>. A zero
        /// component pins that axis (set X for a lateral platform, Y for a vertical one, or both for a diagonal).</summary>
        public float2 TravelHalfExtent;

        /// <summary>Travel speed (units/second) along the oscillation path.</summary>
        public float Speed;

        /// <summary>Accumulated phase (seconds) driving the oscillation; advanced each fixed step.</summary>
        public float Phase;
    }

    /// <summary>
    /// Marks a regular dynamic body the Platformer character should be able to push (a crate, barrel, …). A
    /// pushable-init system ensures every tagged body carries the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> the
    /// controller's deferred-impulse system needs to apply a mass-scaled push — a substrate body baked from
    /// <c>PhysicsBody2DAuthoring</c> has no command buffer, and only the controller's own baker adds one (to the
    /// character). Without it the character's push is silently dropped. The SideScroller's load-bearing integration fact.
    /// </summary>
    public struct Pushable2D : IComponentData { }

    /// <summary>
    /// Marks a trigger-sensor body as a force / wind zone. A wind-zone system reading the substrate's
    /// <c>DynamicBuffer&lt;PhysicsTriggerEvent2D&gt;</c> (Begin/End, Stay derived from the interval), running
    /// <c>[UpdateAfter(PhysicsWorld2DSystem)]</c>, adds <see cref="Force"/> to the kinematic character's
    /// <see cref="KinematicCharacterBody2D.RelativeVelocity"/> while it is inside the zone. Zones that affect the
    /// kinematic character mutate <c>RelativeVelocity</c> from a trigger-event-read system, NOT via substrate effectors
    /// (effectors apply solver forces to dynamic bodies only — the kinematic character is invisible to them).
    /// </summary>
    public struct WindZone2D : IComponentData
    {
        /// <summary>The constant world-space force the zone applies to a character inside it (added to relative velocity
        /// per step). A side-scroller's force zones are constant-direction; gravity-reorient zones are out of scope.</summary>
        public float2 Force;
    }

    /// <summary>
    /// A per-surface friction modifier the GroundMove stance reads off the character's ground hit
    /// (<c>BasicHit2D.Entity</c>) via a <c>ComponentLookup&lt;FrictionModifier2D&gt;</c> and uses to scale the
    /// ground-move acceleration sharpness — low <see cref="Friction"/> → slippery ice, high → sticky. This is the 3D
    /// reference's component approach (<c>CharacterFrictionModifier</c>), ported because the kinematic controller is
    /// material-blind to Box2D friction (its velocity is controller-computed, not solver-resolved). The substrate's real
    /// <c>PhysicsShape2D</c> material governs the dynamic props (crates), not the character.
    /// </summary>
    public struct FrictionModifier2D : IComponentData
    {
        /// <summary>Friction multiplier on the GroundMove acceleration sharpness for a character standing on this
        /// surface. 1 is neutral; toward 0 is ice (slow to gain/shed horizontal speed); above 1 is sticky.</summary>
        public float Friction;
    }

    /// <summary>
    /// Marks a trigger-sensor body as a teleporter pad. A teleporter system reading the substrate trigger-event buffer
    /// teleports the character to <see cref="Destination"/> on entry (an instantaneous, non-swept position set —
    /// either the substrate <c>SetTransform</c> command path or the controller-side pending-teleport fallback,
    /// resolved by a later chunk). The 2D analogue of the 3D <c>Teleporter { Entity DestinationEntity }</c>.
    /// </summary>
    public struct Teleporter2D : IComponentData
    {
        /// <summary>The destination entity whose transform the character is moved to on entering this teleporter.</summary>
        public Entity Destination;
    }
}
