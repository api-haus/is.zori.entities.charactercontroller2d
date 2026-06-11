using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Marks a regular dynamic-body crate (authored with the substrate's <c>PhysicsBody2DAuthoring</c> +
    /// <c>PhysicsShape2DAuthoring</c>) as pushable by the Platformer character. The baker
    /// (<c>PlatformerPropBakers</c>) emits a <see cref="Pushable2D"/> tag; <c>PushableInitSystem2D</c> then gives the
    /// crate the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> the controller's deferred-impulse push needs.
    ///
    /// <para>The push is silently dropped without that buffer — a substrate body baked from
    /// <c>PhysicsBody2DAuthoring</c> carries no command buffer (only the controller's own baker adds one, to the
    /// character), and the deferred-impulse system applies a mass-scaled push only <c>if HasBuffer(target)</c>. This is
    /// the SideScroller's load-bearing integration fact, carried forward unchanged.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Pushable Crate")]
    [DisallowMultipleComponent]
    public sealed class Pushable2DAuthoring : MonoBehaviour { }
}
