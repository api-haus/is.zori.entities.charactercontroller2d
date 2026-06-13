using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Marks a trigger-sensor body (authored with the substrate's <c>PhysicsShape2DAuthoring</c> set to
    /// <c>CollisionResponse = Sensor</c>) as a constant-direction force / wind zone. The baker
    /// (<c>PlatformerPropBakers</c>) emits a <see cref="WindZone2D"/> carrying <see cref="Force"/>;
    /// <c>WindZoneSystem2D</c> reads the substrate trigger-event buffer (Begin/End, Stay derived from the interval),
    /// running <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c>, and adds the force to the kinematic character's
    /// <c>KinematicCharacterBody2D.RelativeVelocity</c> while it is inside the zone.
    ///
    /// <para>Zones that affect the kinematic character mutate <c>RelativeVelocity</c> from the trigger-event-read
    /// system, NOT via substrate effectors — an Area/Point effector applies solver forces to dynamic bodies only, and
    /// the kinematic character is invisible to that solve. A gravity-reorient zone is out of scope; a side-scroller's
    /// grounding-up is fixed at +Y, so the zone is a constant-direction push only.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Wind Zone")]
    [DisallowMultipleComponent]
    public sealed class WindZone2DAuthoring : MonoBehaviour
    {
        [SerializeField]
        [Tooltip(
            "The constant world-space force the zone adds to a character's relative velocity each step while it is "
                + "inside. A side-scroller's force zones are constant-direction (e.g. an updraft +Y, or a sideways gust)."
        )]
        Vector2 m_Force = new Vector2(0f, 12f);

        /// <summary>The constant world-space force added to a character's relative velocity while inside the zone.</summary>
        public Vector2 Force
        {
            get => m_Force;
            set => m_Force = value;
        }
    }
}
