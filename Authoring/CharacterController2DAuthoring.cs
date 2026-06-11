using Unity.Mathematics;
using UnityEngine;

namespace Zori.Entities.CharacterController2D.Authoring
{
    /// <summary>
    /// Which cast-proxy shape the controller's solve sweeps against the world. A subset of the substrate's
    /// <c>Zori.Entities.Physics2D.PhysicsShape2DKind</c> — only Circle and Box have a substrate cast surface
    /// (<c>PhysicsQueries2D.CircleCast</c> / <c>BoxCast</c>), so the proxy is one of those two (design D1).
    /// A local authoring enum (rather than exposing the full substrate enum) so the inspector offers only the
    /// two valid choices and the Authoring assembly stays free of a substrate-runtime reference.
    /// </summary>
    public enum CharacterProxyShape2D : byte
    {
        Circle,
        Box,
    }

    /// <summary>
    /// The single authoring MonoBehaviour for a 2D kinematic character. The 2D analogue of the 3D Standard
    /// Characters sample's <c>*CharacterAuthoring</c> MonoBehaviour (REF Samples~), reduced to ONE component
    /// because there is no built-in "character" component to mirror (bake-contract two-surface convention →
    /// one custom surface here). It carries the author-facing <see cref="AuthoringKinematicCharacterProperties2D"/>,
    /// the <see cref="BasicStepAndSlopeHandlingParameters2D"/>, and the collider-proxy choice (circle radius or
    /// box size). <c>CharacterController2DBaker</c> (editor-only Baking assembly) turns this into the full ECS
    /// character archetype plus the substrate's kinematic body+shape so the entity becomes a live Box2D body.
    /// </summary>
    /// <remarks>
    /// Shaped exactly like the substrate's <c>PhysicsBody2DAuthoring</c>: <c>[SerializeField]</c> private
    /// fields with public properties, inspector <c>[Tooltip]</c>/<c>[Header]</c>, <c>[AddComponentMenu]</c>,
    /// <c>[DisallowMultipleComponent]</c>, and an <c>OnValidate</c> clamp. There is NO separate
    /// <c>Collider2D</c>/<c>PhysicsShape2DAuthoring</c> on the character GameObject — the proxy fields here ARE
    /// the character's collision shape, and the baker emits both the cast proxy and the matching substrate
    /// <c>PhysicsShape2D</c> from them. The MonoBehaviour uses inspector-friendly types (a degrees slope angle
    /// inside the properties struct, a proxy-shape enum); everything reduces to <c>float2</c>/radians/dot-product
    /// at bake.
    /// </remarks>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Character Controller 2D")]
    [DisallowMultipleComponent]
    public sealed class CharacterController2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("General character properties (grounding, collision, dynamics). Slope angle is authored in degrees and converted at bake.")]
        AuthoringKinematicCharacterProperties2D m_CharacterProperties =
            AuthoringKinematicCharacterProperties2D.GetDefault();

        [SerializeField]
        [Tooltip("Step- and slope-handling parameters.")]
        BasicStepAndSlopeHandlingParameters2D m_StepAndSlopeHandling =
            BasicStepAndSlopeHandlingParameters2D.GetDefault();

        [SerializeField]
        [Tooltip(
            "The cast-proxy shape the solve sweeps against the world. Circle (round) or Box (flat sides). "
                + "The substrate exposes only circle/box casts, so a capsule character is approximated by one of "
                + "these (design D1)."
        )]
        CharacterProxyShape2D m_ProxyShape = CharacterProxyShape2D.Circle;

        [SerializeField]
        [Tooltip("Circle proxy radius (used when Proxy Shape is Circle).")]
        float m_ProxyRadius = 0.5f;

        [SerializeField]
        [Tooltip("Box proxy full extents (used when Proxy Shape is Box).")]
        float2 m_ProxyBoxSize = new float2(1f, 1f);

        [SerializeField]
        [Tooltip(
            "Render-rate pose smoothing between fixed physics steps. When on, the body gets the substrate's "
                + "Interpolate smoothing (one step of render lag); off is None (the choppy fixed-step pose)."
        )]
        bool m_InterpolateRendering = true;

        /// <summary>General character properties; slope angle in degrees is converted at bake.</summary>
        public AuthoringKinematicCharacterProperties2D CharacterProperties
        {
            get => m_CharacterProperties;
            set => m_CharacterProperties = value;
        }

        /// <summary>Step- and slope-handling parameters baked verbatim onto the entity.</summary>
        public BasicStepAndSlopeHandlingParameters2D StepAndSlopeHandling
        {
            get => m_StepAndSlopeHandling;
            set => m_StepAndSlopeHandling = value;
        }

        /// <summary>The cast-proxy shape (Circle or Box) the solve sweeps.</summary>
        public CharacterProxyShape2D ProxyShape
        {
            get => m_ProxyShape;
            set => m_ProxyShape = value;
        }

        /// <summary>Circle proxy radius, used when <see cref="ProxyShape"/> is Circle.</summary>
        public float ProxyRadius
        {
            get => m_ProxyRadius;
            set => m_ProxyRadius = math.max(0f, value);
        }

        /// <summary>Box proxy full extents, used when <see cref="ProxyShape"/> is Box.</summary>
        public float2 ProxyBoxSize
        {
            get => m_ProxyBoxSize;
            set => m_ProxyBoxSize = math.max(float2.zero, value);
        }

        /// <summary>Whether the rendered pose is smoothed between fixed steps (the substrate's
        /// <c>PhysicsBody2DInterpolation.Interpolate</c>) or shown at the raw fixed-step pose
        /// (<c>None</c>).</summary>
        public bool InterpolateRendering
        {
            get => m_InterpolateRendering;
            set => m_InterpolateRendering = value;
        }

        void OnValidate()
        {
            m_ProxyRadius = math.max(0f, m_ProxyRadius);
            m_ProxyBoxSize = math.max(float2.zero, m_ProxyBoxSize);
            m_CharacterProperties.GroundSnappingDistance = math.max(
                0f,
                m_CharacterProperties.GroundSnappingDistance
            );
            m_CharacterProperties.Mass = math.max(0f, m_CharacterProperties.Mass);
        }
    }
}
