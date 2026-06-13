using System.Runtime.CompilerServices;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// 2D math utilities, the dimension-reduced port of <c>Unity.CharacterController.MathUtilities</c>
    /// (REF/MathUtilities.cs). Every operation drops the Z dimension: a <c>float3</c> direction becomes a
    /// <c>float2</c>, a quaternion rotation becomes a single z-angle in radians (the package angular convention,
    /// P2D/Documentation~/parity-matrix.md), a 3D plane (defined by its <c>float3</c> normal) becomes a 2D line
    /// (defined by its <c>float2</c> normal), and the 3D <c>cross</c> products that build a perpendicular axis or
    /// a crease direction collapse to the single 2D perpendicular <see cref="perp"/> and the scalar
    /// <see cref="cross2"/>. The quaternion-up/right/forward accessors become <see cref="UpFromAngle"/> /
    /// <see cref="RightFromAngle"/>.
    ///
    /// These are plain <c>static</c> helpers and carry NO <c>[BurstCompile]</c>: per the entry-point-only rule
    /// (docs/unity/burst/compilation-context.md:31-56) only the job/system that calls them is the Burst entry
    /// point, and a helper reached from that context auto-compiles. Every method is HPC#-clean (value types,
    /// <c>Unity.Mathematics</c>, no managed calls), so it Bursts from a caller's job.
    /// </summary>
    public static class MathUtilities2D
    {
        /// <summary>
        /// The 2D perpendicular of a vector: a 90° counter-clockwise rotation, <c>(-v.y, v.x)</c>. This is the 2D
        /// stand-in for the 3D <c>cross</c> that the reference uses to build a "right" axis from a direction (e.g.
        /// the <c>velocityRight</c> in REF/KinematicCharacterUtilities.cs:3596). In 2D there is exactly one
        /// perpendicular direction (up to sign), so a single function replaces the 3D cross-with-up.
        /// </summary>
        /// <param name="v"> The vector to rotate 90° counter-clockwise </param>
        /// <returns> The perpendicular vector, same length as <paramref name="v"/> </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 perp(float2 v)
        {
            return new float2(-v.y, v.x);
        }

        /// <summary>
        /// The scalar 2D cross product <c>a.x*b.y - a.y*b.z</c>, i.e. the signed area / z-component of the 3D
        /// cross of <c>(a.x,a.y,0)</c> and <c>(b.x,b.y,0)</c>. This is the 2D form of every <c>cross(r, n)</c>
        /// "moment arm" the reference's impulse solver takes (REF/PhysicsUtilities.cs:253-256): in 2D the angular
        /// degree of freedom is a single scalar, so the cross of two planar vectors is one number, not a vector.
        /// </summary>
        /// <param name="a"> The first vector </param>
        /// <param name="b"> The second vector </param>
        /// <returns> The signed scalar cross product </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float cross2(float2 a, float2 b)
        {
            return (a.x * b.y) - (a.y * b.x);
        }

        /// <summary>
        /// Calculates the angle in radians between two normalized direction vectors. Verbatim 2D port of
        /// REF/MathUtilities.cs:30 with <c>float2</c> dot/length.
        /// </summary>
        /// <param name="from"> The source direction </param>
        /// <param name="to"> The destination direction </param>
        /// <returns> Angle in radians </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleRadians(float2 from, float2 to)
        {
            float denominator = sqrt(lengthsq(from) * lengthsq(to));
            if (denominator < EPSILON)
                return 0f;

            float d = clamp(dot(from, to) / denominator, -1f, 1f);
            return acos(d);
        }

        /// <summary>
        /// The dot product between two normalized directions separated by a given angle, i.e. <c>cos(angle)</c>.
        /// Verbatim port of REF/MathUtilities.cs:46 (scalar — no dimension to reduce). Used to convert an authored
        /// max-slope angle into the <c>MaxGroundedSlopeDotProduct</c> threshold the grounding test compares against.
        /// </summary>
        /// <param name="angleRadians"> The angle in radians separating the two directions </param>
        /// <returns> The dot-product ratio </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleRadiansToDotRatio(float angleRadians)
        {
            return cos(angleRadians);
        }

        /// <summary>
        /// The angle in radians between two directions that would have a given dot-product ratio, i.e.
        /// <c>acos(dotRatio)</c>. Verbatim port of REF/MathUtilities.cs:57 (scalar).
        /// </summary>
        /// <param name="dotRatio"> The dot-product ratio </param>
        /// <returns> The angle in radians </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DotRatioToAngleRadians(float dotRatio)
        {
            return acos(dotRatio);
        }

        /// <summary>
        /// Projects a vector onto the line whose normal is <paramref name="onLineNormal"/> (subtracting the
        /// component along the normal). This is the 2D analogue of the 3D plane projection
        /// REF/MathUtilities.cs:69 (<c>vector - projectsafe(vector, normal)</c>), which is dimension-agnostic and
        /// ports verbatim with <c>float2</c>: a 3D plane and a 2D line are both "everything orthogonal to the
        /// normal", so removing the normal component leaves the on-surface component in either dimension.
        /// </summary>
        /// <param name="vector"> The vector to project </param>
        /// <param name="onLineNormal"> The line normal to project onto </param>
        /// <returns> The projected vector (the component along the line) </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ProjectOnPlane(float2 vector, float2 onLineNormal)
        {
            return vector - projectsafe(vector, onLineNormal);
        }

        /// <summary>
        /// De-projects a projected vector back to one along <paramref name="onNormalizedVector"/> such that
        /// projecting the result back onto the projected vector's direction reproduces it. Verbatim port of
        /// REF/MathUtilities.cs:82 with <c>float2</c>.
        /// </summary>
        /// <param name="projectedVector"> The projected vector to de-project </param>
        /// <param name="onNormalizedVector"> The desired normalized direction of the de-projected vector </param>
        /// <param name="maxLength"> The maximum length of the de-projected vector (de-projection can blow up for near-perpendicular directions) </param>
        /// <returns> The de-projected vector </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ReverseProjectOnVector(float2 projectedVector, float2 onNormalizedVector, float maxLength)
        {
            float projectionRatio = dot(normalizesafe(projectedVector), onNormalizedVector);
            if (projectionRatio == 0f)
            {
                return projectedVector;
            }

            float deprojectedLength = clamp(length(projectedVector) / projectionRatio, 0f, maxLength);
            return onNormalizedVector * deprojectedLength;
        }

        /// <summary>
        /// Clamps a vector to a maximum length. 2D port of REF/MathUtilities.cs:101 (drops the Z component of the
        /// 3D normalize-and-scale).
        /// </summary>
        /// <param name="vector"> The vector to clamp </param>
        /// <param name="maxLength"> The max length to clamp to </param>
        /// <returns> The clamped vector </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ClampToMaxLength(float2 vector, float maxLength)
        {
            float sqrMagnitude = lengthsq(vector);
            if (sqrMagnitude > maxLength * maxLength)
            {
                float mag = sqrt(sqrMagnitude);
                float normalizedX = vector.x / mag;
                float normalizedY = vector.y / mag;
                return new float2(normalizedX * maxLength, normalizedY * maxLength);
            }

            return vector;
        }

        /// <summary>
        /// The up direction of a 2D rotation given as a z-angle in radians: <c>(-sin(angle), cos(angle))</c>.
        /// Replaces the 3D <c>GetUpFromRotation(quaternion)</c> (REF/MathUtilities.cs:124), which rotated
        /// <c>math.up()</c> by the quaternion. At <c>angle == 0</c> this is world +Y (<c>(0, 1)</c>), matching the
        /// 3D identity-rotation up.
        /// </summary>
        /// <param name="angleRadians"> The z-rotation in radians </param>
        /// <returns> The up direction </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 UpFromAngle(float angleRadians)
        {
            return new float2(-sin(angleRadians), cos(angleRadians));
        }

        /// <summary>
        /// The right direction of a 2D rotation given as a z-angle in radians: <c>(cos(angle), sin(angle))</c>.
        /// Replaces the 3D <c>GetRightFromRotation(quaternion)</c> (REF/MathUtilities.cs:135). At <c>angle == 0</c>
        /// this is world +X (<c>(1, 0)</c>). <see cref="RightFromAngle"/> is <see cref="UpFromAngle"/> rotated −90°,
        /// i.e. <c>perp(up) == -right</c> / <c>perp(right) == up</c>.
        /// </summary>
        /// <param name="angleRadians"> The z-rotation in radians </param>
        /// <returns> The right direction </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 RightFromAngle(float angleRadians)
        {
            return new float2(cos(angleRadians), sin(angleRadians));
        }

        /// <summary>
        /// The z-angle in radians of a direction vector, <c>atan2(dir.y, dir.x)</c> — the inverse of
        /// <see cref="RightFromAngle"/>. The 2D analogue of recovering a rotation from an axis; used where the
        /// 3D code built a quaternion to face a direction (e.g. the rotation helpers in
        /// CharacterControlUtilities2D). A zero vector returns 0.
        /// </summary>
        /// <param name="direction"> The direction to measure </param>
        /// <returns> The z-angle in radians </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleOfDirection(float2 direction)
        {
            if (lengthsq(direction) < EPSILON)
                return 0f;

            return atan2(direction.y, direction.x);
        }

        /// <summary>
        /// An interpolant for a given exponential sharpness over a time delta, <c>1 - exp(-sharpness*dt)</c>.
        /// Verbatim port of REF/MathUtilities.cs:158 (scalar).
        /// </summary>
        /// <param name="sharpness"> The interpolation sharpness </param>
        /// <param name="dt"> The interpolation time delta </param>
        /// <returns> The interpolant in [0, 1] </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float GetSharpnessInterpolant(float sharpness, float dt)
        {
            return saturate(1f - exp(-sharpness * dt));
        }

        /// <summary>
        /// Reorients a vector onto the line whose normal is <paramref name="onLineNormal"/>, preserving the
        /// vector's length and choosing the on-line direction the 3D double-cross would. This is the faithful 2D
        /// reduction of the 3D reorient REF/MathUtilities.cs:171:
        /// <c>normalize(cross(onPlaneNormal, cross(vector, alongDirection))) * length</c>. Reducing both
        /// cross-products for planar (z=0) inputs:
        /// <list type="bullet">
        /// <item><c>cross(vector, alongDirection)</c> is the pure-z vector <c>(0, 0, cross2(vector, alongDirection))</c>.</item>
        /// <item><c>cross(onPlaneNormal, (0,0,z))</c> is <c>z * (n.y, -n.x)</c> = <c>-z * perp(onLineNormal)</c>.</item>
        /// </list>
        /// So the reoriented direction is <c>-cross2(vector, alongDirection) * perp(onLineNormal)</c>, scaled to the
        /// original length. The SIGN is set by <c>cross2(vector, alongDirection)</c> — the original vector's turn
        /// relative to <paramref name="alongDirection"/> — NOT by <c>dot(lineDir, alongDirection)</c>: when
        /// <paramref name="alongDirection"/> is perpendicular to the line (e.g. grounding-up on flat ground, the
        /// dominant case) that dot is zero and gives no left/right information, which is exactly the case a
        /// dot-based disambiguation flips the wrong way. The cross-based sign matches the 3D result (a character
        /// walking +X on flat ground keeps +X, not -X).
        /// </summary>
        /// <param name="vector"> The original vector to reorient (its length is preserved and its turn sets the sign) </param>
        /// <param name="onLineNormal"> The normal of the line the result must lie on </param>
        /// <param name="alongDirection"> The reference direction the 3D double-cross crosses the vector against </param>
        /// <returns> The reoriented vector — length-preserved, on the line, with the 3D double-cross's sign </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ReorientVectorOnPlaneAlongDirection2D(
            float2 vector,
            float2 onLineNormal,
            float2 alongDirection
        )
        {
            float len = length(vector);
            if (len <= EPSILON)
                return Unity.Mathematics.float2.zero;

            // -cross2(vector, alongDirection) * perp(onLineNormal) — the reduction of cross(n, cross(v, along)).
            float z = cross2(vector, alongDirection);
            float2 reoriented = normalizesafe(-z * perp(onLineNormal));
            if (lengthsq(reoriented) <= EPSILON)
                return Unity.Mathematics.float2.zero;

            return reoriented * len;
        }

        /// <summary>
        /// The unit UP-SLOPE tangent of a step top whose surface normal is <paramref name="stepNormal"/> and whose
        /// up axis is <paramref name="groundingUp"/>. The step-up slope-width check (<c>CheckForSteppingUpHit</c>)
        /// samples the surface a short step along this direction to read the slope ahead of the step lip; the
        /// direction MUST be the in-plane tangent that leans up-into the surface (a positive grounding-up component),
        /// the 2D reduction of the 3D's <c>-normalize(cross(cross(GroundingUp, stepNormal), stepNormal))</c>
        /// (REF/KinematicCharacterUtilities.cs:3998).
        ///
        /// <para>With <paramref name="groundingUp"/> = +Y and a normal <c>n = (n.x, n.y)</c>:
        /// <c>cross(up, n).z = -n.x</c>, and <c>cross((0,0,-n.x), (n.x, n.y, 0)) = (n.x·n.y, -n.x², 0)</c>; the 3D
        /// negates and normalizes that, giving <c>-normalize((n.x·n.y, -n.x²))</c>. This is orientation-aware — it
        /// points up-into the surface for a slope rising in EITHER direction — and is zero only for a flat top
        /// (<c>n = up</c>). Two earlier 2D reductions were wrong: <c>ProjectOnPlane(n, up)</c> gave the normal's
        /// purely-horizontal component (away from the slope, no vertical part), and
        /// <c>ReorientVectorOnPlaneAlongDirection2D(up, n, up)</c> is DEGENERATE (vector == alongDirection, so its
        /// cross2 is identically zero and it returns the zero vector for every normal — the sample then drops
        /// straight down at the step lip and a step+slope corner mis-reads a steep normal there, over-rejecting an
        /// in-range step). The general <c>groundingUp</c> form rotates the +Y derivation into the body's up frame.</para>
        /// </summary>
        /// <param name="stepNormal"> The step top's surface normal </param>
        /// <param name="groundingUp"> The character's grounding-up axis </param>
        /// <returns> The unit up-slope tangent, or zero for a flat step top </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 StepTopUpSlopeTangent2D(float2 stepNormal, float2 groundingUp)
        {
            // n.x is the normal's component along the in-plane (perp-to-up) axis; n.y along up.
            float nx = cross2(groundingUp, stepNormal); // = -stepNormal.x when groundingUp = +Y
            float ny = dot(stepNormal, groundingUp);
            // -normalize((nx·ny, -nx²)) expressed in the (perp(up), up) frame, then mapped back to world.
            float2 inPlane = new float2(nx * ny, -nx * nx);
            float2 t = normalizesafe(-inPlane);
            if (lengthsq(t) <= EPSILON)
                return Unity.Mathematics.float2.zero;
            // Map (perpComponent, upComponent) back to world: perp axis is perp(up) = (-up.y, up.x), up axis is up.
            float2 perpUp = perp(groundingUp);
            return (t.x * perpUp) + (t.y * groundingUp);
        }
    }
}
