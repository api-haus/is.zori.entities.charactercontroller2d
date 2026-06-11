using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// A minimal 2D orthographic follow camera for the Platformer sample, the direct analogue of the SideScroller's
    /// <c>SideScrollerFollowCamera</c> retargeted to the Platformer character tag. Each <c>LateUpdate</c> it finds the
    /// Platformer character entity in the default ECS world, reads its <see cref="LocalToWorld"/> translation, and
    /// smoothly follows it in X/Y (the camera keeps its own Z so it looks down the +Z axis at the 2D plane). A plain
    /// MonoBehaviour bridging GameObject-space camera to ECS-space character — the sample's only render concern, kept
    /// out of the package (which is renderer-agnostic) and entirely in <c>Samples~</c>.
    ///
    /// <para>Drop it on the scene's Camera (Orthographic). It is inert until a baked character exists, so it does
    /// nothing on import. It reads the ECS world directly rather than via a system because a camera lives in
    /// GameObject space and updates at render rate; this is the documented bridge pattern for a hybrid camera over an
    /// ECS subject.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer Follow Camera")]
    [RequireComponent(typeof(Camera))]
    public sealed class PlatformerFollowCamera : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("How sharply the camera catches up to the character (higher = snappier).")]
        float m_FollowSharpness = 8f;

        [SerializeField]
        [Tooltip("World-space offset from the character the camera centres on (X/Y).")]
        Vector2 m_Offset = new Vector2(0f, 1.5f);

        EntityQuery _characterQuery;
        World _world;

        void OnEnable()
        {
            TryBindWorld();
        }

        void TryBindWorld()
        {
            _world = World.DefaultGameObjectInjectionWorld;
            if (_world is { IsCreated: true })
            {
                _characterQuery = _world.EntityManager.CreateEntityQuery(
                    ComponentType.ReadOnly<PlatformerCharacterTag>(),
                    ComponentType.ReadOnly<LocalToWorld>());
            }
        }

        void LateUpdate()
        {
            if (_world is not { IsCreated: true })
            {
                TryBindWorld();
                if (_world is not { IsCreated: true })
                {
                    return;
                }
            }

            if (_characterQuery.IsEmptyIgnoreFilter)
            {
                return;
            }

            // Follow the first Platformer character (the sample has one). ToComponentDataArray copies the matches; for
            // the single-character sample this is one element.
            using var transforms = _characterQuery.ToComponentDataArray<LocalToWorld>(Unity.Collections.Allocator.Temp);
            if (transforms.Length == 0)
            {
                return;
            }

            float2 characterPos = transforms[0].Value.c3.xy;
            Vector3 target = new Vector3(characterPos.x + m_Offset.x, characterPos.y + m_Offset.y, transform.position.z);

            float t = 1f - Mathf.Exp(-m_FollowSharpness * Time.deltaTime);
            transform.position = Vector3.Lerp(transform.position, target, t);
        }
    }
}
