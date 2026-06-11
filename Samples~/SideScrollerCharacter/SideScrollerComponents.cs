using Unity.Entities;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// One frame of side-scroller intent, written by <see cref="SideScrollerCharacterControlSystem"/> from input and
    /// consumed by <see cref="SideScrollerCharacterPhysicsSystem"/>'s solve. The 2D analogue of the 3D Standard
    /// Characters sample's <c>FirstPersonCharacterControl</c> / <c>ThirdPersonCharacterControl</c>
    /// (REF/Samples~/…/*CharacterControl.cs): the control system fills it on the regular update, the fixed-step
    /// physics system reads it and turns it into the character's <see cref="KinematicCharacterBody2D.RelativeVelocity"/>.
    ///
    /// <para>The split is the design's control→physics contract (design §8): control is managed input (main thread),
    /// the solve is Bursted fixed-step physics, and this component is the one-frame hand-off between them. <see cref="JumpPressed"/>
    /// is a latched edge (set on the input frame, cleared by the physics system after it consumes a jump) so a jump is
    /// never missed nor double-applied across the differing input/fixed rates.</para>
    /// </summary>
    public struct CharacterControl2D : IComponentData
    {
        /// <summary>Desired horizontal move in [-1, 1] (left/right), in world space. Zero means stand still.</summary>
        public float MoveX;

        /// <summary>
        /// True when a jump was requested and not yet consumed. Latched on the input frame and cleared by the
        /// physics system once it applies the jump while grounded, so the differing input vs fixed-step rates cannot
        /// drop or duplicate a jump.
        /// </summary>
        public bool JumpPressed;
    }

    /// <summary>
    /// Marks a character as driven by this sample's control + physics systems. The sample does NOT use the package's
    /// <see cref="DefaultCharacterController2DTag"/> (which would run the built-in default solve with the default
    /// processor and the default gravity); instead it ships its own worked solve over the
    /// <see cref="SideScrollerCharacterProcessor"/>, gated on this tag, so the sample is the canonical "write your own
    /// processor and drive the steps yourself" example the design §8 prescribes (and the foundation the Phase-B
    /// Platformer sample extends).
    /// </summary>
    public struct SideScrollerCharacterTag : IComponentData { }

    /// <summary>
    /// The platform-mover authoring data baked onto the moving platform: the lateral travel half-extent and speed the
    /// <see cref="SideScrollerMovingPlatformSystem"/> drives the platform back and forth over via
    /// <c>PhysicsBody2DCommands.MovePosition</c>.
    /// </summary>
    public struct SideScrollerMovingPlatform : IComponentData
    {
        /// <summary>The platform's home X (the centre of its lateral oscillation), captured at first update.</summary>
        public float HomeX;

        /// <summary>Whether <see cref="HomeX"/> has been captured yet (false until the first update reads the pose).</summary>
        public bool HomeCaptured;

        /// <summary>Half-extent of the lateral travel: the platform oscillates in X within ±this around <see cref="HomeX"/>.</summary>
        public float TravelHalfExtentX;

        /// <summary>Lateral travel speed (units/second).</summary>
        public float SpeedX;

        /// <summary>The platform's constant world Y (it only moves in X).</summary>
        public float FixedY;

        /// <summary>Accumulated phase (seconds) driving the oscillation; advanced each fixed step.</summary>
        public float Phase;
    }

    /// <summary>
    /// Marks a regular dynamic body the sample character should be able to push (a crate). The
    /// <see cref="SideScrollerPushableInitSystem"/> ensures every tagged body carries the
    /// <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> the controller's deferred-impulse system needs to apply a
    /// mass-scaled push — a substrate body baked from <c>PhysicsBody2DAuthoring</c> has no command buffer, and only
    /// the controller's own baker adds one (to the character). This is the load-bearing integration fact from the C4b
    /// gate (a pushable body needs the command buffer or the push is silently dropped).
    /// </summary>
    public struct SideScrollerPushable : IComponentData { }
}
