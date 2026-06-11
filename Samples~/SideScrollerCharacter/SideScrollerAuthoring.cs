using Unity.Entities;
using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// The scene-authored opt-in singleton for the side-scroller sample (design §8 — inert on import). Every sample
    /// system <c>RequireForUpdate</c>s it, so the sample runs ONLY when a scene bakes a
    /// <see cref="SideScrollerSampleConfig"/> (via <see cref="SideScrollerSampleConfigAuthoring"/>). Importing the
    /// package — which never bakes one — runs nothing. The 2D analogue of the substrate sample's
    /// <c>BatchSpawnSampleConfig</c>.
    /// </summary>
    public struct SideScrollerSampleConfig : IComponentData { }

    /// <summary>
    /// Authors the <see cref="SideScrollerSampleConfig"/> opt-in singleton. Drop one on a GameObject in the sample
    /// SubScene to enable the sample's systems. Mirrors the substrate sample's "tiny authoring MonoBehaviour + baker"
    /// note for <c>BatchSpawnSampleConfig</c>.
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Side-Scroller Sample Config")]
    [DisallowMultipleComponent]
    public sealed class SideScrollerSampleConfigAuthoring : MonoBehaviour { }

    /// <summary>
    /// Marks a <see cref="CharacterController2DAuthoring"/> character as driven by the sample's control + physics
    /// systems. Add it alongside the controller authoring on the character GameObject; the baker emits the
    /// <see cref="SideScrollerCharacterTag"/> + an empty <see cref="CharacterControl2D"/>. The character is NOT given
    /// the package's <see cref="DefaultCharacterController2DTag"/>, so only the sample's solve drives it.
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Side-Scroller Character")]
    [DisallowMultipleComponent]
    public sealed class SideScrollerCharacterAuthoring : MonoBehaviour { }

    /// <summary>
    /// Marks a regular dynamic-body crate (authored with the substrate's <c>PhysicsBody2DAuthoring</c> +
    /// <c>PhysicsShape2DAuthoring</c>) as pushable by the sample character. The baker emits the
    /// <see cref="SideScrollerPushable"/> tag; <see cref="SideScrollerPushableInitSystem"/> then gives the crate the
    /// <c>PhysicsBody2DCommand</c> buffer the controller's push needs.
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Side-Scroller Pushable Crate")]
    [DisallowMultipleComponent]
    public sealed class SideScrollerPushableAuthoring : MonoBehaviour { }

    /// <summary>
    /// Marks a kinematic-body platform (authored with the substrate's <c>PhysicsBody2DAuthoring</c>, BodyType
    /// Kinematic) as the sample's moving platform and carries its lateral travel half-extent + speed. The baker emits
    /// <see cref="SideScrollerMovingPlatform"/>; <see cref="SideScrollerPlatformInitSystem"/> then adds the
    /// <see cref="TrackedTransform2D"/> + the <c>PhysicsBody2DCommand</c> buffer, and
    /// <see cref="SideScrollerMovingPlatformSystem"/> drives it.
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Side-Scroller Moving Platform")]
    [DisallowMultipleComponent]
    public sealed class SideScrollerMovingPlatformAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Half-extent of the platform's lateral travel: it oscillates within +/- this in X around its authored position.")]
        float m_TravelHalfExtentX = 3f;

        [SerializeField]
        [Tooltip("Lateral travel speed (units/second).")]
        float m_SpeedX = 2f;

        /// <summary>Half-extent of the platform's lateral travel.</summary>
        public float TravelHalfExtentX
        {
            get => m_TravelHalfExtentX;
            set => m_TravelHalfExtentX = Mathf.Max(0f, value);
        }

        /// <summary>Lateral travel speed (units/second).</summary>
        public float SpeedX
        {
            get => m_SpeedX;
            set => m_SpeedX = Mathf.Max(0f, value);
        }

        void OnValidate()
        {
            m_TravelHalfExtentX = Mathf.Max(0f, m_TravelHalfExtentX);
            m_SpeedX = Mathf.Max(0f, m_SpeedX);
        }
    }
}
