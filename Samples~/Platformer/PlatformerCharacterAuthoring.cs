using UnityEngine;
using Zori.Entities.CharacterController2D.Authoring;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// The single authoring MonoBehaviour that turns an authored GameObject into the Platformer sample's capsule
    /// character. It is a companion to the package's <see cref="CharacterController2DAuthoring"/> — the two ride on
    /// the SAME GameObject and bake independently onto one entity, exactly as the SideScroller's
    /// <c>SideScrollerCharacterAuthoring</c> rides alongside the controller authoring. The heavy character archetype
    /// (the controller components + buffers, the CAPSULE cast proxy, the kinematic body + the capsule world shape +
    /// the move-command buffer) is emitted by <c>CharacterController2DBaker</c> from the sibling controller authoring;
    /// this MonoBehaviour adds ONLY the Platformer-specific surface that the controller authoring lacks: the
    /// per-character movement + rope tuning, and the initial stance. <c>PlatformerCharacterBaker</c> (editor-only
    /// Baking assembly) emits the Platformer tag, the tuning, the initial state, and the default rope state.
    /// </summary>
    /// <remarks>
    /// <para><b>Why the capsule proxy and step/slope params are NOT re-declared here.</b> The controller authoring
    /// already exposes the capsule proxy (set its <c>ProxyShape</c> to <c>Capsule</c> + the <c>ProxyCapsuleSize</c> /
    /// <c>ProxyCapsuleDirection</c>) and the step/slope params, and its baker already reduces them to the
    /// <c>KinematicCharacterColliderProxy2D</c> (Kind = Capsule, derived end-cap centers + radius) AND the matching
    /// substrate capsule <c>PhysicsShape2D</c> from one shared derivation, so the cast geometry the solve sweeps and
    /// the world body's shape describe one capsule. Re-declaring those fields here would duplicate that baking and risk
    /// the proxy and the world shape diverging. The capsule mandate is satisfied by the sibling controller authoring's
    /// <c>Capsule</c> proxy choice — the Platformer scene-builder sets it on the character GameObject. The
    /// <c>[RequireComponent]</c> makes the companion relationship explicit: a Platformer character cannot be authored
    /// without the controller authoring that gives it its capsule body.</para>
    ///
    /// <para>Shaped like the family's authoring MonoBehaviours: <c>[SerializeField]</c> private fields with public
    /// properties, inspector <c>[Tooltip]</c>/<c>[Header]</c>, <c>[AddComponentMenu]</c>,
    /// <c>[DisallowMultipleComponent]</c>, and an <c>OnValidate</c> clamp. Movement tuning is a PER-CHARACTER property
    /// baked onto the entity (the coordinator correction to the SideScroller's <c>const</c>-on-the-system tuning), not a
    /// scene-ambient config — two Platformer characters in one scene can differ in gravity / speed / jump.</para>
    /// </remarks>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer Character")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(CharacterController2DAuthoring))]
    public sealed class PlatformerCharacterAuthoring : MonoBehaviour
    {
        [Header("Movement tuning (per-character)")]
        [SerializeField]
        [Tooltip("Downward gravity acceleration magnitude (units/s^2), applied every step in every stance.")]
        float m_GravityMagnitude = 20f;

        [SerializeField]
        [Tooltip("Target horizontal speed on the ground line in GroundMove (units/s).")]
        float m_GroundMoveSpeed = 7f;

        [SerializeField]
        [Tooltip("How sharply ground velocity approaches the GroundMove target (acceleration sharpness).")]
        float m_GroundAcceleration = 90f;

        [SerializeField]
        [Tooltip("Target horizontal speed of air control in AirMove (units/s).")]
        float m_AirMoveSpeed = 7f;

        [SerializeField]
        [Tooltip("How sharply air velocity approaches the AirMove target (acceleration sharpness).")]
        float m_AirAcceleration = 30f;

        [SerializeField]
        [Tooltip("Initial upward speed imparted by a grounded jump (units/s).")]
        float m_JumpSpeed = 9f;

        [Header("Rope swing tuning (per-character)")]
        [SerializeField]
        [Tooltip(
            "The rope's length (units): both the max distance the grab query reaches for an anchor and the radius of "
                + "the circle the swing constrains the character onto. The rope is slack until the character swings "
                + "out to this full extension."
        )]
        float m_RopeLength = 5f;

        [SerializeField]
        [Tooltip("Target tangential speed of air control while swinging in RopeSwing (units/s).")]
        float m_RopeSwingMaxSpeed = 7f;

        [SerializeField]
        [Tooltip("How sharply rope-swing air control approaches its target (acceleration sharpness).")]
        float m_RopeSwingAcceleration = 30f;

        [SerializeField]
        [Tooltip("Velocity drag applied each step while swinging, bleeding energy out of the pendulum.")]
        float m_RopeSwingDrag = 0.5f;

        [SerializeField]
        [Tooltip(
            "Collision layers the rope-grab query filters rope anchors by — only colliders on these layers are "
                + "candidate anchors. Baked to the substrate's 64-bit category mask (1 << layer per layer)."
        )]
        LayerMask m_RopeAnchorLayerMask = ~0;

        /// <summary>Downward gravity acceleration magnitude (units/s^2).</summary>
        public float GravityMagnitude
        {
            get => m_GravityMagnitude;
            set => m_GravityMagnitude = Mathf.Max(0f, value);
        }

        /// <summary>Target horizontal speed on the ground line in GroundMove (units/s).</summary>
        public float GroundMoveSpeed
        {
            get => m_GroundMoveSpeed;
            set => m_GroundMoveSpeed = Mathf.Max(0f, value);
        }

        /// <summary>How sharply ground velocity approaches the GroundMove target.</summary>
        public float GroundAcceleration
        {
            get => m_GroundAcceleration;
            set => m_GroundAcceleration = Mathf.Max(0f, value);
        }

        /// <summary>Target horizontal speed of air control in AirMove (units/s).</summary>
        public float AirMoveSpeed
        {
            get => m_AirMoveSpeed;
            set => m_AirMoveSpeed = Mathf.Max(0f, value);
        }

        /// <summary>How sharply air velocity approaches the AirMove target.</summary>
        public float AirAcceleration
        {
            get => m_AirAcceleration;
            set => m_AirAcceleration = Mathf.Max(0f, value);
        }

        /// <summary>Initial upward speed imparted by a grounded jump (units/s).</summary>
        public float JumpSpeed
        {
            get => m_JumpSpeed;
            set => m_JumpSpeed = Mathf.Max(0f, value);
        }

        /// <summary>The rope's length (units): the grab reach and the swing-constraint radius.</summary>
        public float RopeLength
        {
            get => m_RopeLength;
            set => m_RopeLength = Mathf.Max(0f, value);
        }

        /// <summary>Target tangential speed of air control while swinging in RopeSwing (units/s).</summary>
        public float RopeSwingMaxSpeed
        {
            get => m_RopeSwingMaxSpeed;
            set => m_RopeSwingMaxSpeed = Mathf.Max(0f, value);
        }

        /// <summary>How sharply rope-swing air control approaches its target.</summary>
        public float RopeSwingAcceleration
        {
            get => m_RopeSwingAcceleration;
            set => m_RopeSwingAcceleration = Mathf.Max(0f, value);
        }

        /// <summary>Velocity drag applied each step while swinging.</summary>
        public float RopeSwingDrag
        {
            get => m_RopeSwingDrag;
            set => m_RopeSwingDrag = Mathf.Max(0f, value);
        }

        /// <summary>
        /// Collision layers the rope-grab query filters rope anchors by. Authored as a Unity <see cref="LayerMask"/>
        /// (a <c>1 &lt;&lt; layer</c> bitfield), baked to the substrate's 64-bit category mask — which the substrate
        /// itself sets to <c>1 &lt;&lt; gameObject.layer</c> per shape, so an anchor on layer N matches a mask with
        /// bit N set.
        /// </summary>
        public LayerMask RopeAnchorLayerMask
        {
            get => m_RopeAnchorLayerMask;
            set => m_RopeAnchorLayerMask = value;
        }

        void OnValidate()
        {
            m_GravityMagnitude = Mathf.Max(0f, m_GravityMagnitude);
            m_GroundMoveSpeed = Mathf.Max(0f, m_GroundMoveSpeed);
            m_GroundAcceleration = Mathf.Max(0f, m_GroundAcceleration);
            m_AirMoveSpeed = Mathf.Max(0f, m_AirMoveSpeed);
            m_AirAcceleration = Mathf.Max(0f, m_AirAcceleration);
            m_JumpSpeed = Mathf.Max(0f, m_JumpSpeed);
            m_RopeLength = Mathf.Max(0f, m_RopeLength);
            m_RopeSwingMaxSpeed = Mathf.Max(0f, m_RopeSwingMaxSpeed);
            m_RopeSwingAcceleration = Mathf.Max(0f, m_RopeSwingAcceleration);
            m_RopeSwingDrag = Mathf.Max(0f, m_RopeSwingDrag);
        }
    }
}
