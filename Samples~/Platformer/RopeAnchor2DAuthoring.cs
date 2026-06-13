using Unity.Entities;
using UnityEngine;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// A static rope-anchor marker the RopeSwing stance detects by overlap query. The anchor is a static collider
    /// (authored with the substrate's collider authoring) placed on a dedicated rope-anchor collision layer; the
    /// RopeSwing grab logic runs <c>PhysicsQueries2D.OverlapCircle</c> filtered to
    /// <c>PlatformerCharacterTuning2D.RopeAnchorLayerMask</c> and, of the anchors in range, grabs the nearest by centre.
    /// This authoring only emits the <see cref="RopeAnchor2D"/> tag; the dedicated layer is authored on the substrate
    /// collider's Layer field (so the grab query's mask selects only anchors).
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Samples/Platformer/Rope Anchor")]
    [DisallowMultipleComponent]
    public sealed class RopeAnchor2DAuthoring : MonoBehaviour { }

    /// <summary>
    /// Tags a static collider as a rope anchor — a candidate pivot the RopeSwing stance's grab query selects.
    /// <c>PlatformerComponents.cs</c> does not define this marker (it is the one prop tag the rope STANCE owns rather
    /// than a feature-prop driver), so it is defined here alongside its authoring. It is a plain
    /// <see cref="IComponentData"/> tag, so it may share this file with the one MonoBehaviour above (the one-per-file
    /// rule binds only <c>MonoBehaviour</c> / <c>ScriptableObject</c> types, not ECS structs).
    ///
    /// <para>The anchor's world pivot point is its transform position, read by the grab query from the candidate
    /// entity's <c>LocalToWorld</c>; the tag itself carries no data.</para>
    /// </summary>
    public struct RopeAnchor2D : IComponentData { }
}
