using System.Runtime.CompilerServices;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// 2D character velocity/rotation control helpers, the dimension-reduced port of
    /// <c>Unity.CharacterController.CharacterControlUtilities</c> (REF/CharacterControlUtilities.cs). A consumer's
    /// processor calls these to turn intent (a desired velocity, a jump, a facing direction) into the character's
    /// <see cref="KinematicCharacterBody2D.RelativeVelocity"/> and z-rotation. The reductions: every <c>float3</c>
    /// velocity / direction / up becomes a <c>float2</c>; the <c>cross(forward, up)</c> that builds a "right" axis
    /// (REF/CharacterControlUtilities.cs:156) becomes <see cref="MathUtilities2D.perp"/>; and the quaternion
    /// <c>slerp</c> rotation helpers (REF/CharacterControlUtilities.cs:234,251,267) become a single
    /// angle-lerp toward a target z-angle (the package stores rotation as a <c>float</c> radian).
    ///
    /// Plain <c>static</c> helpers, NO <c>[BurstCompile]</c> (entry-point-only rule,
    /// docs/unity/burst/compilation-context.md:31-56): HPC#-clean, Bursting from the caller's job.
    /// </summary>
    public static class CharacterControlUtilities2D
    {
        /// <summary>
        /// The signed slope angle in the move direction, positive when the slope rises ahead and negative when it
        /// falls. 2D port of REF/CharacterControlUtilities.cs:20 with <c>float2</c>.
        /// </summary>
        /// <param name="useDegrees"> Whether to return degrees or radians </param>
        /// <param name="moveDirection"> The character's move direction </param>
        /// <param name="slopeNormal"> The slope's surface normal </param>
        /// <param name="groundingUp"> The character's grounding up direction </param>
        /// <returns> The signed slope angle </returns>
        public static float GetSlopeAngleTowardsDirection(
            bool useDegrees,
            float2 moveDirection,
            float2 slopeNormal,
            float2 groundingUp
        )
        {
            float2 moveDirectionOnSlopePlane = normalizesafe(
                MathUtilities2D.ProjectOnPlane(moveDirection, slopeNormal)
            );
            float angleRadiansWithUp = MathUtilities2D.AngleRadians(moveDirectionOnSlopePlane, groundingUp);

            if (useDegrees)
            {
                return 90f - degrees(angleRadiansWithUp);
            }

            return (PI * 0.5f) - angleRadiansWithUp;
        }

        /// <summary>
        /// Updates velocity for standard interpolated ground movement: reorients current and target velocity onto
        /// the ground line, then interpolates toward the target. 2D port of REF/CharacterControlUtilities.cs:45.
        /// </summary>
        /// <param name="velocity"> Current character velocity </param>
        /// <param name="targetVelocity"> Desired character velocity </param>
        /// <param name="sharpness"> The sharpness of the velocity change </param>
        /// <param name="deltaTime"> The character update time delta </param>
        /// <param name="groundingUp"> The character's grounding up direction </param>
        /// <param name="groundedHitNormal"> The ground hit normal </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardGroundMove_Interpolated(
            ref float2 velocity,
            float2 targetVelocity,
            float sharpness,
            float deltaTime,
            float2 groundingUp,
            float2 groundedHitNormal
        )
        {
            velocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, groundedHitNormal, groundingUp);
            targetVelocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                targetVelocity,
                groundedHitNormal,
                groundingUp
            );
            InterpolateVelocityTowardsTarget(ref velocity, targetVelocity, deltaTime, sharpness);
        }

        /// <summary>
        /// Updates velocity for standard accelerated ground movement: reorients velocity and the acceleration
        /// delta onto the ground line, clamps the addition to the max speed on the line, and applies it. 2D port
        /// of REF/CharacterControlUtilities.cs:63.
        /// </summary>
        /// <param name="velocity"> Current character velocity </param>
        /// <param name="acceleration"> Acceleration vector </param>
        /// <param name="maxSpeed"> Maximum reachable speed </param>
        /// <param name="deltaTime"> The character update time delta </param>
        /// <param name="groundingUp"> The character's grounding up direction </param>
        /// <param name="groundedHitNormal"> The ground hit normal </param>
        /// <param name="forceNoMaxSpeedExcess"> Trim total velocity to an absolute maximum (prevents exploits, can break momentum) </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardGroundMove_Accelerated(
            ref float2 velocity,
            float2 acceleration,
            float maxSpeed,
            float deltaTime,
            float2 groundingUp,
            float2 groundedHitNormal,
            bool forceNoMaxSpeedExcess
        )
        {
            float2 addedVelocityFromAcceleration = Unity.Mathematics.float2.zero;
            AccelerateVelocity(ref addedVelocityFromAcceleration, acceleration, deltaTime);

            velocity = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(velocity, groundedHitNormal, groundingUp);
            addedVelocityFromAcceleration = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                addedVelocityFromAcceleration,
                groundedHitNormal,
                groundingUp
            );
            ClampAdditiveVelocityToMaxSpeedOnPlane(
                ref addedVelocityFromAcceleration,
                velocity,
                maxSpeed,
                groundedHitNormal,
                forceNoMaxSpeedExcess
            );
            velocity += addedVelocityFromAcceleration;
        }

        /// <summary>
        /// Updates velocity for standard accelerated air movement: clamps the acceleration addition to the max
        /// speed on the movement-plane and applies it. 2D port of REF/CharacterControlUtilities.cs:84.
        /// </summary>
        /// <param name="velocity"> Current character velocity </param>
        /// <param name="acceleration"> Acceleration vector </param>
        /// <param name="maxSpeed"> Maximum reachable speed </param>
        /// <param name="movementPlaneUp"> The up direction of the reference movement plane </param>
        /// <param name="deltaTime"> The character update time delta </param>
        /// <param name="forceNoMaxSpeedExcess"> Trim total velocity to an absolute maximum </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardAirMove(
            ref float2 velocity,
            float2 acceleration,
            float maxSpeed,
            float2 movementPlaneUp,
            float deltaTime,
            bool forceNoMaxSpeedExcess
        )
        {
            float2 addedVelocityFromAcceleration = Unity.Mathematics.float2.zero;
            AccelerateVelocity(ref addedVelocityFromAcceleration, acceleration, deltaTime);
            ClampAdditiveVelocityToMaxSpeedOnPlane(
                ref addedVelocityFromAcceleration,
                velocity,
                maxSpeed,
                movementPlaneUp,
                forceNoMaxSpeedExcess
            );
            velocity += addedVelocityFromAcceleration;
        }

        /// <summary>
        /// Interpolates a velocity toward a target with a given sharpness. 2D port of
        /// REF/CharacterControlUtilities.cs:100.
        /// </summary>
        /// <param name="velocity"> The modified velocity </param>
        /// <param name="targetVelocity"> The target velocity </param>
        /// <param name="deltaTime"> The character update time delta </param>
        /// <param name="interpolationSharpness"> The sharpness of the velocity change </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InterpolateVelocityTowardsTarget(
            ref float2 velocity,
            float2 targetVelocity,
            float deltaTime,
            float interpolationSharpness
        )
        {
            velocity = lerp(
                velocity,
                targetVelocity,
                MathUtilities2D.GetSharpnessInterpolant(interpolationSharpness, deltaTime)
            );
        }

        /// <summary>
        /// Accelerates a velocity. 2D port of REF/CharacterControlUtilities.cs:112.
        /// </summary>
        /// <param name="velocity"> The modified velocity </param>
        /// <param name="acceleration"> The acceleration vector </param>
        /// <param name="deltaTime"> The character update time delta </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AccelerateVelocity(ref float2 velocity, float2 acceleration, float deltaTime)
        {
            velocity += acceleration * deltaTime;
        }

        /// <summary>
        /// Adds a velocity to another and clamps the total, but only on the plane orthogonal to
        /// <paramref name="movementPlaneUp"/> (velocity along the up axis stays unclamped). 2D port of
        /// REF/CharacterControlUtilities.cs:126: the lone 3D <c>cross(forward, up)</c> that builds the in-plane
        /// "right" axis (REF/CharacterControlUtilities.cs:156) becomes <see cref="MathUtilities2D.perp"/> of the
        /// in-plane forward.
        /// </summary>
        /// <param name="additiveVelocity"> The added velocity to clamp </param>
        /// <param name="originalVelocity"> The original velocity </param>
        /// <param name="maxSpeed"> Maximum allowed speed on the clamping plane </param>
        /// <param name="movementPlaneUp"> Up direction of the clamping plane </param>
        /// <param name="forceNoMaxSpeedExcess"> Trim total velocity to an absolute maximum </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampAdditiveVelocityToMaxSpeedOnPlane(
            ref float2 additiveVelocity,
            float2 originalVelocity,
            float maxSpeed,
            float2 movementPlaneUp,
            bool forceNoMaxSpeedExcess
        )
        {
            if (forceNoMaxSpeedExcess)
            {
                float2 totalVelocity = originalVelocity + additiveVelocity;
                float2 velocityUp = projectsafe(totalVelocity, movementPlaneUp);
                float2 velocityHorizontal = MathUtilities2D.ProjectOnPlane(totalVelocity, movementPlaneUp);
                velocityHorizontal = MathUtilities2D.ClampToMaxLength(velocityHorizontal, maxSpeed);
                additiveVelocity = (velocityHorizontal + velocityUp) - originalVelocity;
            }
            else
            {
                float maxSpeedSq = maxSpeed * maxSpeed;

                float2 additiveVelocityOnPlaneUp = projectsafe(additiveVelocity, movementPlaneUp);
                float2 additiveVelocityOnPlane = additiveVelocity - additiveVelocityOnPlaneUp;

                float2 originalVelocityOnPlaneUp = projectsafe(originalVelocity, movementPlaneUp);
                float2 originalVelocityOnPlane = originalVelocity - originalVelocityOnPlaneUp;

                float2 totalVelocityOnPlane = originalVelocityOnPlane + additiveVelocityOnPlane;

                if (lengthsq(totalVelocityOnPlane) > maxSpeedSq)
                {
                    float2 originalVelocityForwardOnPlane = normalizesafe(originalVelocityOnPlane);
                    float2 totalVelocityDirectionOnPlane = normalizesafe(totalVelocityOnPlane);

                    float2 totalClampedVelocityOnPlane;
                    if (dot(totalVelocityDirectionOnPlane, originalVelocityForwardOnPlane) > 0f)
                    {
                        float2 originalVelocityRightOnPlane = normalizesafe(
                            MathUtilities2D.perp(originalVelocityForwardOnPlane)
                        );

                        // trim additive velocity excess in original velocity direction
                        float2 trimmedTotalVelocityForwardComponent = MathUtilities2D.ClampToMaxLength(
                            projectsafe(totalVelocityOnPlane, originalVelocityForwardOnPlane),
                            max(maxSpeed, length(originalVelocityOnPlane))
                        );
                        float2 trimmedTotalVelocityRightComponent = MathUtilities2D.ClampToMaxLength(
                            projectsafe(totalVelocityOnPlane, originalVelocityRightOnPlane),
                            maxSpeed
                        );
                        totalClampedVelocityOnPlane =
                            trimmedTotalVelocityForwardComponent + trimmedTotalVelocityRightComponent;
                    }
                    else
                    {
                        // clamp total velocity to circle
                        totalClampedVelocityOnPlane = MathUtilities2D.ClampToMaxLength(totalVelocityOnPlane, maxSpeed);
                    }

                    float2 clampedAdditiveVelocityOnPlane = totalClampedVelocityOnPlane - originalVelocityOnPlane;
                    additiveVelocity = clampedAdditiveVelocityOnPlane + additiveVelocityOnPlaneUp;
                }
            }
        }

        /// <summary>
        /// Standard jump: clears the grounded state and ground hit (so ground-snapping does not undo the jump),
        /// optionally cancels velocity along the velocity-canceling up direction, then adds the jump velocity. 2D
        /// port of REF/CharacterControlUtilities.cs:183 — operates on the 2D <see cref="KinematicCharacterBody2D"/>.
        /// </summary>
        /// <param name="characterBody"> The character's body component </param>
        /// <param name="jumpVelocity"> The jump velocity to add </param>
        /// <param name="cancelVelocityBeforeJump"> Whether to cancel velocity along <paramref name="velocityCancelingUpDirection"/> first </param>
        /// <param name="velocityCancelingUpDirection"> The velocity-canceling up direction </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardJump(
            ref KinematicCharacterBody2D characterBody,
            float2 jumpVelocity,
            bool cancelVelocityBeforeJump,
            float2 velocityCancelingUpDirection
        )
        {
            // Without this, the ground snapping mechanism would prevent jumping.
            characterBody.IsGrounded = false;
            characterBody.GroundHit = default;

            if (cancelVelocityBeforeJump)
            {
                characterBody.RelativeVelocity = MathUtilities2D.ProjectOnPlane(
                    characterBody.RelativeVelocity,
                    velocityCancelingUpDirection
                );
            }

            characterBody.RelativeVelocity += jumpVelocity;
        }

        /// <summary>
        /// Applies drag to a velocity. 2D port of REF/CharacterControlUtilities.cs:204.
        /// </summary>
        /// <param name="velocity"> The velocity to apply drag to </param>
        /// <param name="deltaTime"> The time delta </param>
        /// <param name="drag"> The drag coefficient </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyDragToVelocity(ref float2 velocity, float deltaTime, float drag)
        {
            velocity *= 1f / (1f + drag * deltaTime);
        }

        /// <summary>
        /// The velocity needed to move by a position delta over the next time delta. 2D port of
        /// REF/CharacterControlUtilities.cs:216.
        /// </summary>
        /// <param name="deltaTime"> The time delta </param>
        /// <param name="positionDelta"> The position delta </param>
        /// <returns> The required velocity </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 GetLinearVelocityForMovePosition(float deltaTime, float2 positionDelta)
        {
            if (deltaTime > 0f)
            {
                return positionDelta / deltaTime;
            }

            return default;
        }

        /// <summary>
        /// Interpolates a z-rotation (radians) toward facing a direction, with a given sharpness. Replaces the 3D
        /// quaternion <c>slerp</c> toward <c>LookRotationSafe(direction, up)</c>
        /// (REF/CharacterControlUtilities.cs:234): in 2D "facing a direction" is the single z-angle
        /// <c>atan2(dir.y, dir.x)</c>, and the slerp becomes a shortest-arc angle-lerp by the same sharpness
        /// interpolant. A zero direction leaves the rotation unchanged.
        /// </summary>
        /// <param name="rotationRadians"> The modified z-rotation in radians </param>
        /// <param name="deltaTime"> The time delta </param>
        /// <param name="direction"> The direction to face </param>
        /// <param name="orientationSharpness"> The sharpness of the rotation </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SlerpRotationTowardsDirection(
            ref float rotationRadians,
            float deltaTime,
            float2 direction,
            float orientationSharpness
        )
        {
            if (lengthsq(direction) > 0f)
            {
                float targetAngle = MathUtilities2D.AngleOfDirection(normalizesafe(direction));
                rotationRadians = LerpAngleRadians(
                    rotationRadians,
                    targetAngle,
                    MathUtilities2D.GetSharpnessInterpolant(orientationSharpness, deltaTime)
                );
            }
        }

        /// <summary>
        /// Interpolates a z-rotation (radians) so its up direction points toward a target up. Replaces the 3D
        /// <c>SlerpCharacterUpTowardsDirection</c> (REF/CharacterControlUtilities.cs:267): in 2D the rotation has
        /// only one degree of freedom (the z-angle), so aligning "up" with a target up fully determines the
        /// rotation — the target z-angle is the angle whose <see cref="MathUtilities2D.UpFromAngle"/> equals the
        /// target up, i.e. <c>atan2(up.x, up.y)</c> negated to match the <c>(-sin, cos)</c> up convention. A zero
        /// direction leaves the rotation unchanged.
        /// </summary>
        /// <param name="rotationRadians"> The modified z-rotation in radians </param>
        /// <param name="deltaTime"> The time delta </param>
        /// <param name="upDirection"> The up direction to align to </param>
        /// <param name="orientationSharpness"> The sharpness of the rotation </param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SlerpCharacterUpTowardsDirection(
            ref float rotationRadians,
            float deltaTime,
            float2 upDirection,
            float orientationSharpness
        )
        {
            if (lengthsq(upDirection) > 0f)
            {
                float2 up = normalizesafe(upDirection);
                // UpFromAngle(a) = (-sin a, cos a); solving for the angle whose up equals `up` gives atan2(-up.x, up.y).
                float targetAngle = atan2(-up.x, up.y);
                rotationRadians = LerpAngleRadians(
                    rotationRadians,
                    targetAngle,
                    MathUtilities2D.GetSharpnessInterpolant(orientationSharpness, deltaTime)
                );
            }
        }

        /// <summary>
        /// Lerps from one angle to another (radians) along the shortest arc, wrapping across ±π. The scalar
        /// angle analogue of <c>math.slerp</c> between two quaternions, used by the rotation helpers above.
        /// </summary>
        /// <param name="fromRadians"> The source angle in radians </param>
        /// <param name="toRadians"> The destination angle in radians </param>
        /// <param name="t"> The interpolant in [0, 1] </param>
        /// <returns> The interpolated angle in radians </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float LerpAngleRadians(float fromRadians, float toRadians, float t)
        {
            // Shortest signed delta in (-PI, PI], then step a fraction of it.
            float delta = toRadians - fromRadians;
            delta -= floor((delta + PI) / (2f * PI)) * (2f * PI);
            return fromRadians + (delta * t);
        }
    }
}
