using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Marks a kinematic-body platform (authored with the substrate's <c>PhysicsBody2DAuthoring</c>, BodyType
    /// Kinematic) as the Platformer sample's moving platform, carrying its per-axis travel half-extent + speed. The
    /// baker (<c>PlatformerPropBakers</c>) emits a <see cref="MovingPlatform2D"/>; <c>PlatformInitSystem2D</c> then adds
    /// the <c>TrackedTransform2D</c> + the <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c> at runtime, and
    /// <c>MovingPlatformSystem2D</c> drives the body via a swept <c>MovePosition</c> before the physics step.
    ///
    /// <para>This generalizes the SideScroller's lateral-only platform: <see cref="TravelHalfExtent"/> is a 2D vector,
    /// so a platform travels laterally (X only), vertically (Y only), or diagonally (both). A zero component pins that
    /// axis. The oscillation centre (the home) is captured from the baked pose at the platform's first update, so the
    /// platform swings around where it was authored.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Moving Platform")]
    [DisallowMultipleComponent]
    public sealed class MovingPlatform2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "Per-axis half-extent of the platform's travel: it oscillates within +/- this around its authored "
                + "position. Set X for a lateral platform, Y for a vertical one, or both for a diagonal. A zero "
                + "component pins that axis."
        )]
        Vector2 m_TravelHalfExtent = new Vector2(3f, 0f);

        [SerializeField]
        [Tooltip("Travel speed (units/second) along the oscillation path.")]
        float m_Speed = 2f;

        /// <summary>Per-axis half-extent of the platform's travel (a zero component pins that axis).</summary>
        public Vector2 TravelHalfExtent
        {
            get => m_TravelHalfExtent;
            set => m_TravelHalfExtent = value;
        }

        /// <summary>Travel speed (units/second) along the oscillation path.</summary>
        public float Speed
        {
            get => m_Speed;
            set => m_Speed = Mathf.Max(0f, value);
        }

        void OnValidate()
        {
            m_Speed = Mathf.Max(0f, m_Speed);
        }
    }
}
