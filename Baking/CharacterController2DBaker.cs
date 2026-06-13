using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Physics;
using UnityEngine;
using Zori.Entities.CharacterController2D.Authoring;
using Zori.Entities.Physics2D;
using Zori.Entities.Physics2D.Baking;
using static Unity.Mathematics.math;
// The baker lives in the Zori.Entities.CharacterController2D.Baking namespace, so the substrate's
// PhysicsShape2D is not in an enclosing namespace (unlike the substrate's own bakers) and the unqualified
// name collides with UnityEngine.PhysicsShape2D pulled in by `using UnityEngine;`. Alias to the substrate type.
using PhysicsShape2D = Zori.Entities.Physics2D.PhysicsShape2D;

namespace Zori.Entities.CharacterController2D.Baking
{
    /// <summary>
    /// Bakes a <see cref="CharacterController2DAuthoring"/> GameObject into the full ECS character archetype: the
    /// controller's components and buffers AND the substrate's kinematic body + shape, so the entity converges on
    /// the SAME archetype a substrate-authored kinematic body would carry, plus the character data on top. 2D port
    /// of the 3D <c>KinematicCharacterUtilities.BakeCharacter</c> (REF/KinematicCharacterUtilities.cs:332).
    /// </summary>
    /// <remarks>
    /// The 3D <c>BakeCharacter</c> makes the character a kinematic body by adding <c>PhysicsVelocity</c> + a
    /// kinematic <c>PhysicsMass</c> + a zero <c>PhysicsGravityFactor</c> — the <c>com.unity.physics</c> way to get
    /// a velocity-integrated kinematic body. The 2D substrate has NO kinematic-velocity integration: a body is
    /// moved only by enqueuing a <see cref="PhysicsBody2DCommand"/> that <c>PhysicsWorld2DSystem</c> drains before
    /// the step. So the 2D analogue is a kinematic <see cref="PhysicsBody2DDefinition"/> (<c>gravityScale = 0</c>)
    /// plus the matching <see cref="PhysicsShape2D"/> plus an (empty) <c>DynamicBuffer&lt;PhysicsBody2DCommand&gt;</c>
    /// move queue the solve writes a single <c>MovePosition</c> into each step (design section 7, motion-drive D6).
    /// The emitted body/shape are produced through the same idiom and the same scale-fold helpers
    /// (<see cref="Collider2DBaking"/>) the substrate's own bakers use, so the kinematic body is indistinguishable
    /// from a substrate-authored one (the dual-surface convergence the substrate guarantees).
    ///
    /// Editor-only assembly (<c>includePlatforms: ["Editor"]</c>, like <c>Zori.Entities.Physics2D.Baking</c>), so
    /// the <c>UnityEngine.*</c> authoring references never reach a player build.
    /// </remarks>
    public sealed class CharacterController2DBaker : Baker<CharacterController2DAuthoring>
    {
        public override void Bake(CharacterController2DAuthoring authoring)
        {
            // GetEntity(Dynamic) gives the entity the LocalToWorld the substrate write-back targets — the same
            // request every substrate body/shape baker makes (bake-contract.md:5). The substrate owns LocalToWorld
            // (not LocalTransform), so the controller never requests WorldSpace the way the 3D BakeCharacter does;
            // the substrate write-back lands the simulated pose.
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            var t = GetComponent<Transform>();

            // D9: fold the transform scale into the proxy + shape geometry the way the substrate folds scale into
            // shapes (bake-contract.md "Transform scale"), rather than rejecting non-unit scale like the 3D baker
            // (REF/KinematicCharacterUtilities.cs:337). ReadScale registers the transform dependency.
            var scale = Collider2DBaking.ReadScale(t);

            // --- Character components ----------------------------------------------------------------------

            // Build the runtime properties from authoring, converting the authored slope ANGLE (degrees) to the
            // runtime dot-product threshold: cos(radians(angle)) — the 2D analogue of the 3D
            // KinematicCharacterProperties(AuthoringKinematicCharacterProperties) constructor's
            // MathUtilities.AngleRadiansToDotRatio(radians(MaxGroundedSlopeAngle)) (REF/KinematicCharacterComponents.cs:232,
            // AngleRadiansToDotRatio == cos at REF/MathUtilities.cs:46).
            var a = authoring.CharacterProperties;
            AddComponent(
                entity,
                new KinematicCharacterProperties2D
                {
                    EvaluateGrounding = a.EvaluateGrounding,
                    SnapToGround = a.SnapToGround,
                    GroundSnappingDistance = a.GroundSnappingDistance,
                    EnhancedGroundPrecision = a.EnhancedGroundPrecision,
                    MaxGroundedSlopeDotProduct = cos(radians(a.MaxGroundedSlopeAngle)),
                    DetectMovementCollisions = a.DetectMovementCollisions,
                    DecollideFromOverlaps = a.DecollideFromOverlaps,
                    ProjectVelocityOnInitialOverlaps = a.ProjectVelocityOnInitialOverlaps,
                    MaxContinuousCollisionsIterations = a.MaxContinuousCollisionsIterations,
                    MaxOverlapDecollisionIterations = a.MaxOverlapDecollisionIterations,
                    DiscardMovementWhenExceedMaxIterations = a.DiscardMovementWhenExceedMaxIterations,
                    KillVelocityWhenExceedMaxIterations = a.KillVelocityWhenExceedMaxIterations,
                    DetectObstructionsForParentBodyMovement = a.DetectObstructionsForParentBodyMovement,
                    SimulateDynamicBody = a.SimulateDynamicBody,
                    Mass = a.Mass,
                }
            );

            // The transient per-step state, defaulted (GroundingUp = +Y). Enableable, like the 3D component.
            AddComponent(entity, KinematicCharacterBody2D.GetDefault());

            // The pre-solve snapshot target; StoreKinematicCharacterBodyPropertiesSystem2D fills it each step.
            AddComponent(entity, new StoredKinematicCharacterData2D());

            // Step/slope params, baked verbatim (no reduction — every field is scalar/bool).
            AddComponent(entity, authoring.StepAndSlopeHandling);

            // The circle-or-box cast proxy the solve sweeps (design D1). Scale-folded the same per-kind way the
            // substrate scales the matching shape below, so the proxy the solve casts and the world body's shape
            // describe the same geometry at the body's unit scale.
            var proxy = BuildProxy(authoring, scale);
            AddComponent(entity, proxy);

            // --- The four transient hit / impulse buffers (3D BakeCharacter:356-359) -----------------------

            AddBuffer<KinematicCharacterHit2D>(entity);
            AddBuffer<KinematicCharacterDeferredImpulse2D>(entity);
            AddBuffer<StatefulKinematicCharacterHit2D>(entity);
            AddBuffer<KinematicVelocityProjectionHit2D>(entity);

            // --- The substrate kinematic body + shape + move queue -----------------------------------------

            // The 2D analogue of the 3D BakeCharacter:362-364 (PhysicsVelocity + kinematic PhysicsMass +
            // zero PhysicsGravityFactor): a kinematic body with no gravity. Built like Rigidbody2DBaker /
            // PhysicsBody2DAuthoringBaker so the creation system treats it as any other kinematic body.
            AddComponent(
                entity,
                new PhysicsBody2DDefinition
                {
                    bodyType = PhysicsBody.BodyType.Kinematic,
                    gravityScale = 0f,
                    linearDamping = 0f,
                    angularDamping = 0f,
                    initialPosition = ((float3)t.position).xy,
                    initialRotationRadians = radians(t.eulerAngles.z),
                    // A kinematic body is moved by command, never integrated, so freeze flags / mass are inert;
                    // the controller's own Mass lives on KinematicCharacterProperties2D for the impulse path.
                    constraints = PhysicsBody.BodyConstraints.None,
                    mass = 0f,
                    useAutoMass = false,
                    // Render-rate smoothing: the substrate's PhysicsBody2DSmoothing replaces the 3D
                    // CharacterInterpolation (design D7). On → Interpolate (one step of render lag); off → None.
                    interpolation = authoring.InterpolateRendering
                        ? PhysicsBody2DInterpolation.Interpolate
                        : PhysicsBody2DInterpolation.None,
                    fastCollisions = false,
                    overrideMassDistribution = false,
                    centerOfMass = Unity.Mathematics.float2.zero,
                    rotationalInertia = 0f,
                }
            );

            // The world collider, matching the cast proxy: same circle/box geometry, scale-folded by the same
            // substrate helpers a CircleCollider2DBaker / BoxCollider2DBaker uses, so the baked shape is
            // bit-identical to a substrate-authored shape of the same local geometry. friction/bounciness/density
            // take the substrate's material-less defaults (0.4 / 0 / 1); category/contact bits resolve from the
            // GameObject layer exactly as the substrate collider bakers do.
            Collider2DBaking.ReadFilter(authoring, out var categoryBits, out var contactBits);
            AddComponent(entity, BuildShape(authoring, scale, categoryBits, contactBits));

            // Carry the entity scale to graphics — the shape geometry is baked at this scale (so the Box2D body is
            // unit scale), and the write-back re-applies the scale to LocalToWorld. Emitted by every substrate body
            // baker (PhysicsBody2DAuthoringBaker:85, Collider2DBaking.AddStaticBodyIfNoRigidbody:57).
            AddComponent(entity, new PhysicsBody2DRenderScale { value = scale });

            // The per-entity move queue. The solve enqueues one MovePosition (+ optional MoveRotation) per step;
            // PhysicsWorld2DSystem drains it before Simulate. Added once at bake (runtime-api.md "Add the buffer
            // once"); baking it avoids a runtime structural change on the character's first solved frame.
            AddBuffer<PhysicsBody2DCommand>(entity);
        }

        /// <summary>
        /// Build the cast proxy from the authoring shape choice, folding transform scale into the geometry the
        /// same per-kind way the substrate scales a circle (larger absolute axis) or box (per-axis) shape, so the
        /// proxy and the emitted <see cref="PhysicsShape2D"/> stay the same geometry at the body's unit scale.
        /// </summary>
        static KinematicCharacterColliderProxy2D BuildProxy(CharacterController2DAuthoring authoring, float2 scale)
        {
            switch (authoring.ProxyShape)
            {
                case CharacterProxyShape2D.Box:
                    return new KinematicCharacterColliderProxy2D
                    {
                        Kind = PhysicsShape2DKind.Box,
                        BoxSize = Collider2DBaking.ScaleBoxSize(authoring.ProxyBoxSize, scale),
                        Radius = 0f,
                    };
                case CharacterProxyShape2D.Capsule:
                    DeriveCapsule(authoring, scale, out var capRadius, out var c1, out var c2);
                    return new KinematicCharacterColliderProxy2D
                    {
                        Kind = PhysicsShape2DKind.Capsule,
                        Radius = capRadius,
                        CapsuleCenter1 = c1,
                        CapsuleCenter2 = c2,
                        BoxSize = Unity.Mathematics.float2.zero,
                    };
                default:
                    return new KinematicCharacterColliderProxy2D
                    {
                        Kind = PhysicsShape2DKind.Circle,
                        Radius = Collider2DBaking.ScaleCircleRadius(authoring.ProxyRadius, scale),
                        BoxSize = Unity.Mathematics.float2.zero,
                    };
            }
        }

        /// <summary>
        /// Reduce the authored capsule <c>Size</c> + direction to a Box2D capsule (cap radius + two local end-cap
        /// centers), folding transform scale into the size per-axis FIRST — the exact derivation the substrate's
        /// <see cref="CapsuleCollider2DBaker"/> uses (Collider2DBaking.cs:314-335), so the controller's capsule
        /// proxy and the substrate's built-in capsule shape bake to the same geometry from the same authored size.
        /// </summary>
        static void DeriveCapsule(
            CharacterController2DAuthoring authoring,
            float2 scale,
            out float capsuleRadius,
            out float2 c1,
            out float2 c2
        )
        {
            var halfSize = Collider2DBaking.ScaleBoxSize(authoring.ProxyCapsuleSize, scale) * 0.5f;
            if (authoring.ProxyCapsuleDirection == CharacterCapsuleDirection2D.Vertical)
            {
                capsuleRadius = halfSize.x;
                var half = max(0f, halfSize.y - capsuleRadius);
                c1 = new float2(0f, -half);
                c2 = new float2(0f, half);
            }
            else
            {
                capsuleRadius = halfSize.y;
                var half = max(0f, halfSize.x - capsuleRadius);
                c1 = new float2(-half, 0f);
                c2 = new float2(half, 0f);
            }
        }

        /// <summary>
        /// Build the substrate world collider matching the cast proxy, with the substrate's material-less surface
        /// defaults and the resolved layer filter, scale-folded identically to <see cref="BuildProxy"/> so the
        /// world body's shape and the solve's sensing proxy describe one geometry.
        /// </summary>
        static PhysicsShape2D BuildShape(
            CharacterController2DAuthoring authoring,
            float2 scale,
            ulong categoryBits,
            ulong contactBits
        )
        {
            var shape = new PhysicsShape2D
            {
                offset = Unity.Mathematics.float2.zero,
                // Substrate material-less defaults (Collider2DBaking.ReadSurface's no-material branch), so the
                // character body has a sane surface without an authored PhysicsMaterial2D.
                friction = 0.4f,
                bounciness = 0f,
                density = 1f,
                frictionMixing = PhysicsSurfaceMixing2D.Average,
                bouncinessMixing = PhysicsSurfaceMixing2D.Average,
                categoryBits = categoryBits,
                contactBits = contactBits,
                isTrigger = false,
            };

            switch (authoring.ProxyShape)
            {
                case CharacterProxyShape2D.Box:
                    shape.kind = PhysicsShape2DKind.Box;
                    shape.size = Collider2DBaking.ScaleBoxSize(authoring.ProxyBoxSize, scale);
                    shape.radius = 0f;
                    break;
                case CharacterProxyShape2D.Capsule:
                    DeriveCapsule(authoring, scale, out var capRadius, out var c1, out var c2);
                    shape.kind = PhysicsShape2DKind.Capsule;
                    shape.radius = capRadius;
                    shape.capsuleCenter1 = c1;
                    shape.capsuleCenter2 = c2;
                    break;
                default:
                    shape.kind = PhysicsShape2DKind.Circle;
                    shape.radius = Collider2DBaking.ScaleCircleRadius(authoring.ProxyRadius, scale);
                    break;
            }

            return shape;
        }
    }
}
