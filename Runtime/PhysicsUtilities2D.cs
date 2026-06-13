using System.Runtime.CompilerServices;
using Unity.Mathematics;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// 2D physics utilities for the controller's dynamics path — the dimension-reduced port of
    /// <c>Unity.CharacterController.PhysicsUtilities</c> (REF/PhysicsUtilities.cs). It holds the collision-impulse
    /// solver (with its angular degree of freedom collapsed to a scalar), the impulse-application helpers, the
    /// character mass-from-properties builders, and the read of a regular dynamic body's velocity for impulse
    /// exchange.
    ///
    /// Two reference methods are intentionally dropped (design file-map): <c>GetHitFaceNormal</c> (it recovered a
    /// triangle face normal from a 3D mesh leaf — the 2D substrate's casts already return the contact normal in
    /// <see cref="PhysicsQueryHit2D.normal"/>) and the mesh-leaf <c>SetCollisionResponse</c> path (a
    /// <c>ChildCollider</c> pointer write that has no 2D analogue). The physics-tag and material helpers are also
    /// dropped: the 2D <see cref="PhysicsQueryHit2D"/> carries no per-hit material or custom-tag.
    ///
    /// <para><b>The angular reduction.</b> In 3D the moment arm is a <c>cross(r, n)</c> vector and inertia is a
    /// <c>float3</c> tensor (REF/PhysicsUtilities.cs:253-258). In 2D angular motion is a single scalar about the z
    /// axis, so the arm becomes the scalar <see cref="MathUtilities2D.cross2"/> and inertia becomes a single
    /// <c>float</c> (<see cref="KinematicCharacterMass2D.InverseInertia"/>). A point's velocity from a planar
    /// angular velocity <c>ω</c> (rad/s) about a center is <c>v + ω·perp(r)</c>, since <c>ω·ẑ × r = ω(-r.y, r.x)</c>.</para>
    ///
    /// <para><b>Burst.</b> Plain <c>static</c> helpers, NO <c>[BurstCompile]</c> (entry-point-only rule,
    /// docs/unity/burst/compilation-context.md:31-56). The solver and the mass/impulse helpers are HPC#-clean and
    /// Burst from the caller's job. The lone exception is the body-velocity read
    /// (<see cref="TryGetDynamicBodyMotion"/>): it touches the raw <c>Unity.U2D.Physics.PhysicsBody</c> handle's
    /// managed <c>linearVelocity</c>/<c>angularVelocity</c> properties, which return <c>UnityEngine.Vector2</c>
    /// and are NOT HPC# — exactly as the substrate's own write-back reads them on the main thread
    /// (PhysicsBody2DWriteBackSystem.cs:95-135, "reads the body's managed velocity via the Unity.U2D.Physics
    /// handle, which is not Burst"). That one helper must be called from main-thread code, never from inside a
    /// Burst entry point. See the C2 deliverable's D5 verdict.</para>
    /// </summary>
    public static class PhysicsUtilities2D
    {
        /// <summary>
        /// Builds the 2D mass properties of a kinematic character from its stored data. 2D port of
        /// <c>GetKinematicCharacterPhysicsMass(StoredKinematicCharacterData)</c> (REF/PhysicsUtilities.cs:279): a
        /// kinematic character has zero rotational inertia (it never spins from impulses) and an inverse mass of
        /// <c>1/Mass</c> only when it simulates as a dynamic body, else 0 (infinite mass — pushes others, is not
        /// pushed). Center of mass is the body origin.
        /// </summary>
        /// <param name="storedCharacterData"> The character's pre-solve stored data </param>
        /// <returns> The character's 2D mass for the impulse solve </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static KinematicCharacterMass2D GetKinematicCharacterMass(
            in StoredKinematicCharacterData2D storedCharacterData
        )
        {
            return new KinematicCharacterMass2D
            {
                CenterOfMass = Unity.Mathematics.float2.zero,
                InverseInertia = 0f,
                InverseMass = storedCharacterData.SimulateDynamicBody ? (1f / storedCharacterData.Mass) : 0f,
            };
        }

        /// <summary>
        /// Builds the 2D mass properties of a kinematic character from its properties component. 2D port of
        /// <c>GetKinematicCharacterPhysicsMass(KinematicCharacterProperties)</c> (REF/PhysicsUtilities.cs:296).
        /// </summary>
        /// <param name="characterProperties"> The character properties </param>
        /// <returns> The character's 2D mass for the impulse solve </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static KinematicCharacterMass2D GetKinematicCharacterMass(
            in KinematicCharacterProperties2D characterProperties
        )
        {
            return new KinematicCharacterMass2D
            {
                CenterOfMass = Unity.Mathematics.float2.zero,
                InverseInertia = 0f,
                InverseMass = characterProperties.SimulateDynamicBody ? (1f / characterProperties.Mass) : 0f,
            };
        }

        /// <summary>
        /// The world-space linear velocity of a point on a body, given the body's linear velocity, its planar
        /// angular velocity (rad/s about z), its center of mass, and the point. 2D form of
        /// <c>PhysicsVelocity.GetLinearVelocity</c> as used at REF/PhysicsUtilities.cs:240: the 3D
        /// <c>v + cross(ω, r)</c> becomes <c>v + ω·perp(r)</c>.
        /// </summary>
        /// <param name="linearVelocity"> The body's linear velocity (m/s) </param>
        /// <param name="angularVelocityRadians"> The body's angular velocity (rad/s, about +z) </param>
        /// <param name="centerOfMassWorld"> The body's center of mass in world space </param>
        /// <param name="point"> The world-space point to evaluate </param>
        /// <returns> The point's world-space linear velocity </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 GetPointVelocity(
            float2 linearVelocity,
            float angularVelocityRadians,
            float2 centerOfMassWorld,
            float2 point
        )
        {
            float2 r = point - centerOfMassWorld;
            return linearVelocity + (angularVelocityRadians * MathUtilities2D.perp(r));
        }

        /// <summary>
        /// Solves a collision between two bodies, outputting the linear impulse to apply on each. 2D port of
        /// <c>SolveCollisionImpulses</c> (REF/PhysicsUtilities.cs:225): the 3D point velocities use
        /// <see cref="GetPointVelocity"/>; the two <c>cross(r, n)</c> moment arms become the scalar
        /// <see cref="MathUtilities2D.cross2"/>; the inverse-inertia tensor term collapses to
        /// <c>crossA²·invInertiaA + crossB²·invInertiaB</c>; and the inverse effective mass adds the two inverse
        /// masses, exactly as the 3D solver. The impulse is applied along <paramref name="collisionNormalBToA"/>
        /// only when the bodies are approaching (the relative velocity along the normal is positive), so a
        /// separating pair produces no impulse — verbatim with the reference.
        /// </summary>
        /// <param name="linearVelA"> Body A's linear velocity </param>
        /// <param name="angularVelARadians"> Body A's angular velocity (rad/s about +z) </param>
        /// <param name="linearVelB"> Body B's linear velocity </param>
        /// <param name="angularVelBRadians"> Body B's angular velocity (rad/s about +z) </param>
        /// <param name="massA"> Body A's 2D mass </param>
        /// <param name="massB"> Body B's 2D mass </param>
        /// <param name="centerOfMassWorldA"> Body A's world-space center of mass </param>
        /// <param name="centerOfMassWorldB"> Body B's world-space center of mass </param>
        /// <param name="collisionPoint"> The world-space collision point </param>
        /// <param name="collisionNormalBToA"> The collision normal, from B toward A </param>
        /// <param name="impulseOnA"> Output linear impulse to apply on A </param>
        /// <param name="impulseOnB"> Output linear impulse to apply on B </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SolveCollisionImpulses(
            float2 linearVelA,
            float angularVelARadians,
            float2 linearVelB,
            float angularVelBRadians,
            in KinematicCharacterMass2D massA,
            in KinematicCharacterMass2D massB,
            float2 centerOfMassWorldA,
            float2 centerOfMassWorldB,
            float2 collisionPoint,
            float2 collisionNormalBToA,
            out float2 impulseOnA,
            out float2 impulseOnB
        )
        {
            impulseOnA = default;
            impulseOnB = default;

            float2 pointVelocityA = GetPointVelocity(
                linearVelA,
                angularVelARadians,
                centerOfMassWorldA,
                collisionPoint
            );
            float2 pointVelocityB = GetPointVelocity(
                linearVelB,
                angularVelBRadians,
                centerOfMassWorldB,
                collisionPoint
            );

            float2 centerOfMassAToPoint = collisionPoint - centerOfMassWorldA;
            float2 centerOfMassBToPoint = collisionPoint - centerOfMassWorldB;

            float2 relativeVelocityAToB = pointVelocityB - pointVelocityA;
            float relativeVelocityOnNormal = dot(relativeVelocityAToB, collisionNormalBToA);

            if (relativeVelocityOnNormal > 0f)
            {
                float crossA = MathUtilities2D.cross2(centerOfMassAToPoint, collisionNormalBToA);
                float crossB = MathUtilities2D.cross2(collisionNormalBToA, centerOfMassBToPoint);
                float angularTerm = (crossA * crossA * massA.InverseInertia) + (crossB * crossB * massB.InverseInertia);
                float invEffectiveMass = angularTerm + (massA.InverseMass + massB.InverseMass);

                if (invEffectiveMass > 0f)
                {
                    float effectiveMass = 1f / invEffectiveMass;

                    float impulseScale = -relativeVelocityOnNormal * effectiveMass;
                    float2 totalImpulse = collisionNormalBToA * impulseScale;

                    impulseOnA = -totalImpulse;
                    impulseOnB = totalImpulse;
                }
            }
        }

        /// <summary>
        /// Applies a linear impulse to a body's linear velocity: <c>v += impulse · invMass</c>. 2D form of
        /// <c>PhysicsVelocity.ApplyLinearImpulse</c> as used at REF/KinematicCharacterUtilities.cs:3288.
        /// </summary>
        /// <param name="linearVelocity"> The modified linear velocity </param>
        /// <param name="mass"> The body's 2D mass </param>
        /// <param name="impulse"> The linear impulse to apply </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyLinearImpulse(
            ref float2 linearVelocity,
            in KinematicCharacterMass2D mass,
            float2 impulse
        )
        {
            linearVelocity += impulse * mass.InverseMass;
        }

        /// <summary>
        /// Applies an impulse at a world point to a body's linear AND angular velocity. 2D form of
        /// <c>PhysicsVelocity.ApplyImpulse</c> as used at REF/KinematicCharacterUtilities.cs:3309: the linear part
        /// is <c>v += impulse · invMass</c>; the angular part adds the scalar torque impulse
        /// <c>cross2(r, impulse) · invInertia</c> where <c>r</c> is the arm from the center of mass to the point.
        /// </summary>
        /// <param name="linearVelocity"> The modified linear velocity </param>
        /// <param name="angularVelocityRadians"> The modified angular velocity (rad/s about +z) </param>
        /// <param name="mass"> The body's 2D mass </param>
        /// <param name="centerOfMassWorld"> The body's world-space center of mass </param>
        /// <param name="impulse"> The linear impulse to apply </param>
        /// <param name="worldPoint"> The world-space point the impulse is applied at </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(
            ref float2 linearVelocity,
            ref float angularVelocityRadians,
            in KinematicCharacterMass2D mass,
            float2 centerOfMassWorld,
            float2 impulse,
            float2 worldPoint
        )
        {
            linearVelocity += impulse * mass.InverseMass;
            float2 r = worldPoint - centerOfMassWorld;
            angularVelocityRadians += MathUtilities2D.cross2(r, impulse) * mass.InverseInertia;
        }

        /// <summary>
        /// Reads a regular (non-character) dynamic body's current linear and angular velocity through its live
        /// <see cref="PhysicsBody"/> handle. This is the 2D stand-in for the read half of
        /// <c>PhysicsUtilities.GetBodyComponents</c> (REF/PhysicsUtilities.cs:126), which reconstructed velocity
        /// and mass from the 3D <c>PhysicsWorld</c>'s <c>MotionVelocities</c>/<c>MotionDatas</c> arrays. The 2D
        /// substrate exposes no such read API, so the velocity is read off the raw handle exactly as the
        /// substrate's own write-back does (PhysicsBody2DWriteBackSystem.cs:116-122).
        ///
        /// <para><b>Not Burst-callable.</b> <see cref="PhysicsBody.linearVelocity"/> returns a
        /// <c>UnityEngine.Vector2</c> and <see cref="PhysicsBody.angularVelocity"/> a managed <c>float</c>
        /// (deg/sec); these are managed property reads, not HPC#. Call this only from main-thread code — the
        /// caller (C4's hit-dynamics path) must read the other body's velocity on the main thread, mirroring the
        /// write-back's main-thread <c>CaptureSmoothing</c> pass. See the C2 deliverable's D5 verdict.</para>
        ///
        /// <para>The angular velocity is converted from the engine's deg/sec to rad/sec so it composes with the
        /// rad/sec the impulse solver and <see cref="GetPointVelocity"/> assume (the package's angular-velocity
        /// unit is deg/sec — PhysicsBody2DCommands.cs:128-131 — and its rotation-angle unit is radians).</para>
        /// </summary>
        /// <param name="body"> The live body handle (from <see cref="PhysicsBody2D.body"/>) </param>
        /// <param name="linearVelocity"> The body's linear velocity (m/s) </param>
        /// <param name="angularVelocityRadians"> The body's angular velocity, converted to rad/s </param>
        /// <returns> True if the handle was valid and the velocity was read </returns>
        public static bool TryGetDynamicBodyMotion(
            PhysicsBody body,
            out float2 linearVelocity,
            out float angularVelocityRadians
        )
        {
            linearVelocity = Unity.Mathematics.float2.zero;
            angularVelocityRadians = 0f;

            if (!body.isValid)
            {
                return false;
            }

            UnityEngine.Vector2 v = body.linearVelocity;
            linearVelocity = new float2(v.x, v.y);
            angularVelocityRadians = radians(body.angularVelocity);
            return true;
        }
    }
}
