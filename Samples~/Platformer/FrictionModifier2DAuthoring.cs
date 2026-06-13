using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Authors a <see cref="FrictionModifier2D"/> on a surface entity (a floor segment authored with the substrate's
    /// collider authoring). The baker (<c>PlatformerPropBakers</c>) emits a <see cref="FrictionModifier2D"/> carrying
    /// <see cref="Friction"/>. The GroundMove stance reads it off the character's ground hit
    /// (<c>BasicHit2D.Entity</c>) via a <c>ComponentLookup&lt;FrictionModifier2D&gt;</c> and scales the ground-move
    /// acceleration sharpness — low <see cref="Friction"/> is slippery ice, high is sticky.
    ///
    /// <para>This is the 3D reference's component approach (<c>CharacterFrictionModifier</c>), ported because the
    /// kinematic controller is material-blind to Box2D friction (its velocity is controller-computed, not
    /// solver-resolved). The substrate's real <c>PhysicsShape2D</c> material governs the dynamic crates, not the
    /// character — this component is the per-surface knob the character itself feels.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Friction Modifier")]
    [DisallowMultipleComponent]
    public sealed class FrictionModifier2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "Friction multiplier on the GroundMove acceleration sharpness for a character standing on this surface. "
                + "1 is neutral; toward 0 is ice (slow to gain/shed horizontal speed); above 1 is sticky."
        )]
        float m_Friction = 1f;

        /// <summary>Friction multiplier on the GroundMove acceleration sharpness (1 neutral, toward 0 ice, &gt;1 sticky).</summary>
        public float Friction
        {
            get => m_Friction;
            set => m_Friction = Mathf.Max(0f, value);
        }

        void OnValidate()
        {
            m_Friction = Mathf.Max(0f, m_Friction);
        }
    }
}
