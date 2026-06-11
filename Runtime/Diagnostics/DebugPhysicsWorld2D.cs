using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using static Unity.Mathematics.math;
using Zori.Entities.Physics2D;
using PhysicsBody2D = Zori.Entities.Physics2D.PhysicsBody2D;
using PhysicsRotate = Unity.U2D.Physics.PhysicsRotate;
using PhysicsShape2D = Zori.Entities.Physics2D.PhysicsShape2D;
using PhysicsTransform = Unity.U2D.Physics.PhysicsTransform;
using PhysicsWorld = Unity.U2D.Physics.PhysicsWorld;

namespace Zori.Entities.CharacterController2D.Diagnostics
{
    /// <summary>
    /// A reusable hybrid component that draws the substrate's collision world into the GAME view so the otherwise-invisible
    /// ECS physics bodies are visible while playing. The physics package is renderer-agnostic — it writes
    /// <c>LocalToWorld</c> and stops, so a body has no visual of its own — and this component is the package's worked
    /// "wire a draw" surface: drop it on any regular scene GameObject (NOT a SubScene — it is a hybrid MonoBehaviour, not
    /// an authoring marker) and it reads the running ECS world each frame and outlines every body.
    ///
    /// <para>It is the same hybrid-MonoBehaviour-reads-ECS pattern the controller's sample follow camera uses: it binds
    /// <see cref="World.DefaultGameObjectInjectionWorld"/>, queries <see cref="PhysicsWorldSingleton2D"/> for the live
    /// <c>Unity.U2D.Physics.PhysicsWorld</c> handle, and issues per-shape draw calls from each body's live pose. It is
    /// inert until the substrate world singleton exists, so it does nothing on import or before a scene bakes a body.</para>
    ///
    /// <para>The draw routes through the physics module's free <c>PhysicsWorld.Draw*</c> surface
    /// (<c>UnityEngine.PhysicsCore2DModule</c>). Drawing is <b>always allowed in the Editor</b>
    /// (<see cref="PhysicsWorld.isRenderingAllowed"/>: "Rendering is always allowed in the Editor"), so the queued
    /// geometry is flushed into the active rendering camera and the outlines appear in the Game view during in-editor
    /// Play — not only as Scene-view gizmos. Each <c>lifetime: 0</c> call is a draw-once-this-frame; issuing them every
    /// <c>LateUpdate</c> produces a continuous visual.</para>
    ///
    /// <para>A character body (one carrying <see cref="KinematicCharacterBody2D"/>) is coloured by its grounding state —
    /// <see cref="GroundedColor"/> when grounded, <see cref="AirborneColor"/> when airborne — so the grounding flag reads
    /// at a glance; a dynamic body is <see cref="DynamicColor"/>; everything else (static/kinematic world geometry) is
    /// <see cref="WorldColor"/>.</para>
    /// </summary>
    [AddComponentMenu("Zori/Entities Character Controller 2D/Debug Physics World 2D")]
    [DisallowMultipleComponent]
    public sealed class DebugPhysicsWorld2D : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("Master toggle: when on, the collision world is drawn into the Game view while playing.")]
        bool m_DrawWorld = true;

        [SerializeField]
        [Tooltip("Outline thickness for the per-shape draw.")]
        float m_Thickness = 2f;

        [SerializeField]
        [Tooltip("Colour of a static or kinematic world collider (floor, walls, slopes, platforms).")]
        Color m_WorldColor = new Color(0.35f, 0.75f, 1f, 1f);

        [SerializeField]
        [Tooltip("Colour of a dynamic body (a pushable crate, a falling body).")]
        Color m_DynamicColor = new Color(1f, 0.85f, 0.2f, 1f);

        [SerializeField]
        [Tooltip("Colour of a kinematic character body when grounded.")]
        Color m_GroundedColor = new Color(0.3f, 1f, 0.3f, 1f);

        [SerializeField]
        [Tooltip("Colour of a kinematic character body when airborne.")]
        Color m_AirborneColor = new Color(1f, 0.3f, 0.3f, 1f);

        /// <summary>The on/off toggle: when false the component draws nothing.</summary>
        public bool DrawWorld
        {
            get => m_DrawWorld;
            set => m_DrawWorld = value;
        }

        /// <summary>Outline thickness for the per-shape draw.</summary>
        public float Thickness
        {
            get => m_Thickness;
            set => m_Thickness = value;
        }

        /// <summary>Colour of a static or kinematic world collider.</summary>
        public Color WorldColor
        {
            get => m_WorldColor;
            set => m_WorldColor = value;
        }

        /// <summary>Colour of a dynamic body.</summary>
        public Color DynamicColor
        {
            get => m_DynamicColor;
            set => m_DynamicColor = value;
        }

        /// <summary>Colour of a character body when grounded.</summary>
        public Color GroundedColor
        {
            get => m_GroundedColor;
            set => m_GroundedColor = value;
        }

        /// <summary>Colour of a character body when airborne.</summary>
        public Color AirborneColor
        {
            get => m_AirborneColor;
            set => m_AirborneColor = value;
        }

        World _world;
        EntityQuery _bodyQuery;
        EntityQuery _worldSingletonQuery;

        void OnEnable()
        {
            TryBindWorld();
        }

        void TryBindWorld()
        {
            _world = World.DefaultGameObjectInjectionWorld;
            if (_world is not { IsCreated: true })
            {
                return;
            }

            // Every created body carries PhysicsBody2D (the handle) + PhysicsShape2D (the geometry) + LocalToWorld
            // (the live pose) — the public components the draw reads. A body without a shape is simply skipped by the
            // query's PhysicsShape2D requirement.
            _bodyQuery = _world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsBody2D>(),
                ComponentType.ReadOnly<PhysicsShape2D>(),
                ComponentType.ReadOnly<LocalToWorld>());
            _worldSingletonQuery = _world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
        }

        void LateUpdate()
        {
            if (!m_DrawWorld)
            {
                return;
            }

            if (_world is not { IsCreated: true })
            {
                TryBindWorld();
                if (_world is not { IsCreated: true })
                {
                    return;
                }
            }

            if (_worldSingletonQuery.IsEmptyIgnoreFilter)
            {
                return; // substrate world not created yet (inert until the scene bakes a body).
            }

            PhysicsWorld world = _worldSingletonQuery.GetSingleton<PhysicsWorldSingleton2D>().world;
            if (!world.isValid)
            {
                return;
            }

            // Set the world's outline thickness, then issue an explicit per-shape outline for each ECS body from its
            // live pose. The PhysicsWorld.Draw* surface enqueues geometry the physics module renders into the active
            // camera; drawing is always allowed in the Editor (PhysicsWorld.isRenderingAllowed), so these outlines show
            // in the GAME view during in-editor Play, not only as Scene-view gizmos. Per-shape calls (not the global
            // drawOptions auto-draw) are used so a character body can be coloured by its grounding state.
            world.drawThickness = m_Thickness;
            DrawBodies(world);
        }

        void DrawBodies(PhysicsWorld world)
        {
            EntityManager em = _world.EntityManager;

            using var entities = _bodyQuery.ToEntityArray(Unity.Collections.Allocator.Temp);
            for (int i = 0; i < entities.Length; i++)
            {
                Entity e = entities[i];

                LocalToWorld ltw = em.GetComponentData<LocalToWorld>(e);
                PhysicsShape2D shape = em.GetComponentData<PhysicsShape2D>(e);

                float2 worldPos = ltw.Value.c3.xy;
                // The body's z-rotation, recovered from the LocalToWorld right axis (c0.xy) — the 2D plane's only DOF.
                float angleRad = atan2(ltw.Value.c0.y, ltw.Value.c0.x);

                Color color = ColorFor(e, em);
                DrawShape(world, in shape, worldPos, angleRad, color);
            }
        }

        Color ColorFor(Entity e, EntityManager em)
        {
            // A kinematic character body: colour by grounding state (the package's own component, available to any
            // controller character — not a sample-only tag).
            if (em.HasComponent<KinematicCharacterBody2D>(e))
            {
                KinematicCharacterBody2D body = em.GetComponentData<KinematicCharacterBody2D>(e);
                return body.IsGrounded ? m_GroundedColor : m_AirborneColor;
            }

            // A dynamic body (a pushable crate) vs a static/kinematic world collider.
            if (em.HasComponent<PhysicsBody2DDefinition>(e))
            {
                PhysicsBody2DDefinition def = em.GetComponentData<PhysicsBody2DDefinition>(e);
                if (def.bodyType == Unity.U2D.Physics.PhysicsBody.BodyType.Dynamic)
                {
                    return m_DynamicColor;
                }
            }

            return m_WorldColor;
        }

        void DrawShape(PhysicsWorld world, in PhysicsShape2D shape, float2 worldPos, float angleRad, Color color)
        {
            // The shape's local offset rotated into world space and added to the body origin.
            float c = cos(angleRad);
            float s = sin(angleRad);
            float2 off = shape.offset;
            float2 center = worldPos + new float2(c * off.x - s * off.y, s * off.x + c * off.y);

            var rot = PhysicsRotate.FromRadians(angleRad);

            switch (shape.kind)
            {
                case PhysicsShape2DKind.Circle:
                    world.DrawCircle(
                        new Vector2(center.x, center.y),
                        shape.radius,
                        color,
                        0f,
                        PhysicsWorld.DrawFillOptions.Outline);
                    break;

                case PhysicsShape2DKind.Box:
                    // The box carries its own local angle (radians) folded on top of the body rotation.
                    var boxRot = PhysicsRotate.FromRadians(angleRad + shape.boxAngleRadians);
                    world.DrawBox(
                        new PhysicsTransform(new Vector2(center.x, center.y), boxRot),
                        new Vector2(shape.size.x, shape.size.y),
                        shape.radius,
                        color,
                        0f,
                        PhysicsWorld.DrawFillOptions.Outline);
                    break;

                case PhysicsShape2DKind.Capsule:
                    world.DrawCapsule(
                        new PhysicsTransform(new Vector2(worldPos.x, worldPos.y), rot),
                        new Vector2(shape.capsuleCenter1.x, shape.capsuleCenter1.y),
                        new Vector2(shape.capsuleCenter2.x, shape.capsuleCenter2.y),
                        shape.radius,
                        color,
                        0f,
                        PhysicsWorld.DrawFillOptions.Outline);
                    break;

                case PhysicsShape2DKind.Polygon:
                case PhysicsShape2DKind.Edge:
                    DrawVertexShape(world, in shape, worldPos, rot, color);
                    break;
            }
        }

        void DrawVertexShape(PhysicsWorld world, in PhysicsShape2D shape, float2 worldPos, PhysicsRotate rot, Color color)
        {
            if (!shape.vertices.IsCreated)
            {
                return;
            }

            ref var pts = ref shape.vertices.Value.points;
            int n = pts.Length;
            if (n < 2)
            {
                return;
            }

            var span = new Vector2[n];
            for (int i = 0; i < n; i++)
            {
                span[i] = new Vector2(pts[i].x, pts[i].y);
            }

            bool loop = shape.kind == PhysicsShape2DKind.Polygon || shape.edgeIsLoop;
            world.DrawLineStrip(
                new PhysicsTransform(new Vector2(worldPos.x, worldPos.y), rot),
                span,
                loop,
                color,
                0f);
        }
    }
}
