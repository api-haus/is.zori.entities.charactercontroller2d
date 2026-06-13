using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Marks a trigger-sensor body (authored with the substrate's <c>PhysicsShape2DAuthoring</c> set to
    /// <c>CollisionResponse = Sensor</c>) as a teleporter pad, and references the destination GameObject the character
    /// is moved to on entry. The baker (<c>PlatformerPropBakers</c>) resolves <see cref="Destination"/> to its entity
    /// and emits a <see cref="Teleporter2D"/>; <c>TeleporterSystem2D</c> reads the substrate trigger-event buffer and,
    /// on a character's Enter, performs a best-effort teleport to the destination's transform.
    ///
    /// <para>The destination is any GameObject in the scene (an empty marker is enough); the character is teleported to
    /// its world position. The 2D analogue of the 3D <c>Teleporter { Entity DestinationEntity }</c>.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Teleporter")]
    [DisallowMultipleComponent]
    public sealed class Teleporter2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "The destination GameObject whose world position the character is moved to on entering this teleporter."
        )]
        GameObject m_Destination;

        /// <summary>The destination GameObject the character is teleported to on entry.</summary>
        public GameObject Destination
        {
            get => m_Destination;
            set => m_Destination = value;
        }
    }
}
