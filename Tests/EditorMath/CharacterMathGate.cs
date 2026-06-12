using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Zori.Entities.CharacterController2D;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D.Tests.EditorMath
{
    /// <summary>
    /// The adversarial EditMode gate for the C4a core-solve PURE MATH — the killer cases live in the math, not in
    /// the integration. The validating agent (not the implementer) built every case from the solve's own decision
    /// points, not from inputs the implementer imagined: the D6 velocity-projection corner-kill at its branch
    /// boundaries (single wall slides, two-wall wedge kills, near-parallel walls = the crease-vs-corner boundary,
    /// grounded-slope + wall = the ground-plane constraint), the slope-grounding threshold at exactly /just over /
    /// just under <c>MaxGroundedSlopeAngle</c>, the grounded velocity reorient (parallel-keeps-magnitude /
    /// along-up-is-zero / between-interpolates), and the <see cref="MathUtilities2D"/> angle↔direction round-trips +
    /// the length-preserving reorient. A projection that does not slide on a single wall, a wedge that does not kill,
    /// or a slope threshold off by the boundary is an IMPLEMENTATION bug; these tests are RED for that and GREEN
    /// otherwise.
    ///
    /// <para>Lives in an <c>includePlatforms:["Editor"]</c> (EditorOnly) test assembly so the Unity Test Framework
    /// classifies it EditMode (UTF buckets a whole assembly by the AssemblyFlags.EditorOnly bit, which only the
    /// editor-only platform set carries — a plain [Test] in an all-platforms assembly is discovered ONLY under
    /// PlayMode). The math under test is HPC#-clean and has no editor dependency, so it is exercised directly.</para>
    ///
    /// <para><c>Default_ProjectVelocityOnHits</c> takes a real <c>DynamicBuffer&lt;KinematicVelocityProjectionHit2D&gt;</c>;
    /// a buffer cannot be synthesised without an <see cref="EntityManager"/>, so the projection tests spin up a tiny
    /// throwaway <see cref="World"/>, add the buffer to one entity, populate it with the hit lines under test, call
    /// the real static, then dispose — no mocks, the real public API.</para>
    /// </summary>
    [TestFixture]
    public sealed class CharacterMathGate
    {
        const float Eps = 1e-3f;

        static readonly float2 Up = new float2(0f, 1f);

        static bool Approx(float2 a, float2 b, float eps = Eps) => length(a - b) < eps;

        // A unit direction at `deg` degrees CCW from +X.
        static float2 Dir(float deg)
        {
            sincos(radians(deg), out var s, out var c);
            return new float2(c, s);
        }

        // ===================== A throwaway world for the buffer-taking projection static =====================

        World _world;
        Entity _entity;

        [SetUp]
        public void SetUp()
        {
            _world = new World("CharacterMathGate");
            _entity = _world.EntityManager.CreateEntity();
            _world.EntityManager.AddBuffer<KinematicVelocityProjectionHit2D>(_entity);
        }

        [TearDown]
        public void TearDown()
        {
            if (_world is { IsCreated: true })
                _world.Dispose();
            _world = null;
        }

        DynamicBuffer<KinematicVelocityProjectionHit2D> Buffer()
        {
            return _world.EntityManager.GetBuffer<KinematicVelocityProjectionHit2D>(_entity);
        }

        // Run Default_ProjectVelocityOnHits over a freshly-populated buffer (each `hit` is a (normal, isGroundedOnHit)
        // line; the LAST entry is the "first hit" the projection projects on first — matching the solve's
        // convention that the latest detected hit is the buffer tail). Returns the resulting velocity.
        float2 Project(
            float2 velocity,
            float2 originalVelocityDirection,
            bool grounded,
            float2 groundNormal,
            bool constrainToGroundPlane,
            params (float2 normal, bool groundedOnHit)[] lines
        )
        {
            var buffer = Buffer();
            buffer.Clear();
            foreach (var (normal, groundedOnHit) in lines)
                buffer.Add(
                    new KinematicVelocityProjectionHit2D(
                        normal,
                        Unity.Mathematics.float2.zero,
                        groundedOnHit
                    )
                );

            var characterBody = KinematicCharacterBody2D.GetDefault();
            characterBody.GroundingUp = Up;
            characterBody.IsGrounded = grounded;

            var groundHit = new BasicHit2D(
                Entity.Null,
                Unity.Mathematics.float2.zero,
                groundNormal
            );

            KinematicCharacterUtilities2D.Default_ProjectVelocityOnHits(
                ref velocity,
                ref grounded,
                ref groundHit,
                in buffer,
                originalVelocityDirection,
                constrainToGroundPlane,
                in characterBody
            );

            return velocity;
        }

        // ===================== D6 — velocity projection (the compounding-risk math) =====================

        [Test]
        public void D6_SingleWall_VelocityEndsParallel_MagnitudeReducedNotReversed()
        {
            // Walk RIGHT into a vertical wall whose surface faces LEFT (normal -X). The velocity must end parallel to
            // the wall (no X component), keep no more than its original magnitude, and NOT reverse into +X.
            var wallNormal = new float2(-1f, 0f);
            var velocity = new float2(5f, -2f); // moving right and slightly down (a fall-into-wall)
            var dir = normalize(velocity);

            var result = Project(
                velocity,
                dir,
                grounded: false,
                groundNormal: Unity.Mathematics.float2.zero,
                constrainToGroundPlane: false,
                (wallNormal, false)
            );

            // Slides along the wall: the component INTO the wall (along the normal) is gone.
            Assert.AreEqual(
                0f,
                dot(result, wallNormal),
                Eps,
                "velocity must end parallel to the wall (no into-wall component)"
            );
            // The remaining component is the tangential (vertical) part, magnitude-preserved on that axis.
            Assert.AreEqual(-2f, result.y, Eps, "tangential (Y) component preserved");
            // Not reversed: it did not bounce back out along +X.
            Assert.LessOrEqual(result.x, Eps, "velocity must not reverse out of the wall (+X)");
            // Magnitude reduced (the into-wall part removed), not grown.
            Assert.Less(length(result), length(velocity) + Eps, "magnitude reduced, not amplified");
            Assert.Greater(
                length(result),
                0f,
                "a single non-head-on wall slide keeps some velocity"
            );
        }

        [Test]
        public void D6_Corner_TwoOpposingNonParallelWalls_VelocityKilled()
        {
            // A 2D corner: a left-facing wall (normal -X) and a down-facing... no: two walls that BOTH oppose the
            // motion and are non-parallel wedge the character. Walk diagonally up-right into a wall pair: a vertical
            // wall (normal -X) the character is already touching (prior line) and a ceiling-ish wall (normal
            // pointing down-left) as the latest hit. After projecting on the latest, the velocity re-enters the
            // first wall → wedged corner → killed.
            var firstWallNormal = new float2(-1f, 0f); // prior line: vertical wall facing left
            var secondWallNormal = normalize(new float2(-1f, -1f)); // latest hit: a wall facing down-left
            var velocity = new float2(4f, 3f); // moving up-right into both
            var dir = normalize(velocity);

            var result = Project(
                velocity,
                dir,
                grounded: false,
                groundNormal: Unity.Mathematics.float2.zero,
                constrainToGroundPlane: false,
                // order: prior line first, latest (the "first hit") last.
                (firstWallNormal, false),
                (secondWallNormal, false)
            );

            Assert.IsTrue(
                Approx(result, Unity.Mathematics.float2.zero),
                $"wedged corner must kill velocity, got {result}"
            );
        }

        [Test]
        public void D6_NearParallel_NotAWedge_StillSlides_DoesNotFalselyKill()
        {
            // The crease-vs-corner boundary: two walls whose normals are ALMOST the same line (a tiny angle apart).
            // IsSamePlane treats them as one line (dot > 1 - DotProductSimilarityEpsilon), so the corner test SKIPS
            // them and the character keeps sliding — it must NOT be falsely killed as a wedge. This is the case the
            // implementer most easily gets wrong (treating any second line as a corner).
            var firstWallNormal = new float2(-1f, 0f);
            // ~1.3 degrees off the first normal — within the DotProductSimilarityEpsilon (0.001) same-plane band:
            // cos(1.3deg) ~ 0.99974 > 0.999.
            var secondWallNormal = normalize(Dir(180f + 1.3f));
            var velocity = new float2(5f, -1f);
            var dir = normalize(velocity);

            var result = Project(
                velocity,
                dir,
                grounded: false,
                groundNormal: Unity.Mathematics.float2.zero,
                constrainToGroundPlane: false,
                (firstWallNormal, false),
                (secondWallNormal, false)
            );

            Assert.Greater(
                length(result),
                0.1f,
                $"near-parallel walls are one line, not a wedge — must keep sliding, got {result}"
            );
        }

        [Test]
        public void D6_WideAngleWalls_GenuineWedge_Killed()
        {
            // The other side of the crease-vs-corner boundary: a wide-angle V (the two walls 90 degrees apart, the
            // opposite extreme from the near-parallel case above). The character is driven straight DOWN into the
            // bottom of the V. Projecting onto the latest wall (facing up-LEFT) leaves a velocity that re-enters the
            // prior wall (facing up-RIGHT) → wedged corner → killed. Together with the near-parallel test this
            // brackets the corner branch: near-parallel = one line, keep sliding; wide-angle V into the vertex = kill.
            var firstWallNormal = normalize(new float2(1f, 1f)); // prior line: wall facing up-right
            var secondWallNormal = normalize(new float2(-1f, 1f)); // latest hit: wall facing up-left
            var v = new float2(0f, -3f); // driven straight into the bottom of the V

            var result = Project(
                v,
                normalize(v),
                grounded: false,
                groundNormal: Unity.Mathematics.float2.zero,
                constrainToGroundPlane: false,
                (firstWallNormal, false),
                (secondWallNormal, false)
            );

            Assert.IsTrue(
                Approx(result, Unity.Mathematics.float2.zero),
                $"a wide-angle wedge driven into the vertex must kill velocity, got {result}"
            );
        }

        [Test]
        public void D6_GroundedSlopePlusWall_RespectsGroundPlaneConstraint()
        {
            // Grounded on a shallow slope, then hit a non-grounded wall while constrainToGroundPlane is on. The
            // projected velocity must (a) not drive into the wall and (b) stay constrained to the ground plane (the
            // ground-constrained slide: project on the ground line, then on the wall line — REF :1446 branch).
            var groundNormal = normalize(new float2(0f, 1f)); // flat ground for a clean constraint read
            var wallNormal = new float2(-1f, 0f); // a vertical wall, non-grounded
            var velocity = new float2(6f, 0f); // walking right along the ground into the wall
            var dir = normalize(velocity);

            var result = Project(
                velocity,
                dir,
                grounded: true,
                groundNormal: groundNormal,
                constrainToGroundPlane: true,
                (wallNormal, false)
            );

            Assert.LessOrEqual(dot(result, wallNormal), Eps, "must not drive into the wall");
            // Constrained to the (flat) ground plane: no vertical climb out of the ground.
            Assert.AreEqual(
                0f,
                result.y,
                Eps,
                "ground-plane constraint: no vertical component on flat ground"
            );
            // Walking straight into a perpendicular wall along flat ground stops horizontal motion (wall removes +X,
            // ground removes any +Y).
            Assert.IsTrue(
                Approx(result, Unity.Mathematics.float2.zero),
                $"head-on into a wall while grounded on flat ground stops, got {result}"
            );
        }

        [Test]
        public void D6_GroundedLanding_KillsVerticalThenReorientsOntoGround()
        {
            // Ungrounded, falling, latest hit grounds the character (a floor). The single-hit grounded-landing branch
            // (REF :1464) kills the vertical velocity then reorients the remainder onto the ground line. A pure
            // straight-down fall onto flat ground ends with zero velocity (no tangential component to keep).
            var floorNormal = Up;
            var velocity = new float2(0f, -8f);
            var dir = normalize(velocity);

            var result = Project(
                velocity,
                dir,
                grounded: false,
                groundNormal: Unity.Mathematics.float2.zero,
                constrainToGroundPlane: true,
                (floorNormal, true)
            );

            Assert.IsTrue(
                Approx(result, Unity.Mathematics.float2.zero),
                $"straight fall onto flat ground stops, got {result}"
            );
        }

        // ===================== Slope grounding threshold (boundary / over / under) =====================

        [Test]
        public void IsGroundedOnSlopeNormal_AtBoundary_IsNotGrounded_StrictGreater()
        {
            // The threshold is a STRICT greater-than (dot(up, normal) > maxDot). At EXACTLY the max angle the dot
            // equals the threshold, so it is NOT grounded — the boundary belongs to "ungrounded". This pins the
            // strict-vs-nonstrict choice the implementer had to make.
            const float maxAngleDeg = 50f;
            float maxDot = cos(radians(maxAngleDeg));

            // A slope normal exactly maxAngleDeg from up: normal = up rotated by maxAngleDeg.
            var atBoundary = Dir(90f - maxAngleDeg); // 90-50 = 40 deg from +X == 50 deg from +Y
            Assert.AreEqual(
                maxDot,
                dot(Up, atBoundary),
                1e-5f,
                "constructed boundary normal sanity"
            );

            Assert.IsFalse(
                KinematicCharacterUtilities2D.IsGroundedOnSlopeNormal(maxDot, atBoundary, Up),
                "a normal EXACTLY at the max slope angle is not grounded (strict >)"
            );
        }

        [Test]
        public void IsGroundedOnSlopeNormal_JustUnder_IsGrounded_JustOver_IsNot()
        {
            const float maxAngleDeg = 50f;
            float maxDot = cos(radians(maxAngleDeg));

            var justUnder = Dir(90f - (maxAngleDeg - 0.5f)); // a shallower slope (smaller angle from up)
            var justOver = Dir(90f - (maxAngleDeg + 0.5f)); // a steeper slope (larger angle from up)

            Assert.IsTrue(
                KinematicCharacterUtilities2D.IsGroundedOnSlopeNormal(maxDot, justUnder, Up),
                "a slope just shallower than the max is grounded"
            );
            Assert.IsFalse(
                KinematicCharacterUtilities2D.IsGroundedOnSlopeNormal(maxDot, justOver, Up),
                "a slope just steeper than the max is not grounded"
            );
        }

        [Test]
        public void IsGroundedOnSlopeNormal_AngleToDotRoundTrip_MatchesBakeConversion()
        {
            // The baker converts an authored degrees angle to the runtime dot threshold via cos(radians(angle)) (the
            // same MathUtilities2D.AngleRadiansToDotRatio the bake uses). Confirm the round-trip so a designer's
            // "max 45deg" really grounds a 44deg slope and rejects a 46deg one.
            const float maxAngleDeg = 45f;
            float maxDot = MathUtilities2D.AngleRadiansToDotRatio(radians(maxAngleDeg));

            Assert.IsTrue(
                KinematicCharacterUtilities2D.IsGroundedOnSlopeNormal(maxDot, Dir(90f - 44f), Up)
            );
            Assert.IsFalse(
                KinematicCharacterUtilities2D.IsGroundedOnSlopeNormal(maxDot, Dir(90f - 46f), Up)
            );
            // and the inverse conversion recovers the angle.
            Assert.AreEqual(
                radians(maxAngleDeg),
                MathUtilities2D.DotRatioToAngleRadians(maxDot),
                1e-4f
            );
        }

        // ===================== ProjectVelocityOnGrounding (parallel / along-up / between) =====================

        [Test]
        public void ProjectOnGrounding_VelocityParallelToSlope_KeepsMagnitude()
        {
            // On a 30deg slope, a velocity already pointing up the slope keeps its full magnitude (it is already on
            // the ground plane — the reorient is the identity up to direction).
            const float slopeDeg = 30f;
            var groundNormal = Dir(90f + slopeDeg); // normal of a slope rising to the right
            var slopeDir = normalize(MathUtilities2D.perp(groundNormal)); // a direction along the slope line
            // pick the uphill-right half-line.
            if (slopeDir.x < 0f)
                slopeDir = -slopeDir;
            var velocity = slopeDir * 7f;

            ProjectGround(ref velocity, groundNormal);

            Assert.AreEqual(
                7f,
                length(velocity),
                1e-2f,
                "velocity already along the slope keeps its magnitude"
            );
        }

        [Test]
        public void ProjectOnGrounding_VelocityAlongGroundingUp_BecomesZero()
        {
            // A velocity straight along grounding-up has no on-slope component — the reorient yields zero (REF :2354
            // invariant: zero when velocity is parallel to GroundingUp).
            var groundNormal = Dir(90f + 20f);
            var velocity = Up * 5f;

            ProjectGround(ref velocity, groundNormal);

            Assert.IsTrue(
                Approx(velocity, Unity.Mathematics.float2.zero),
                $"velocity along grounding-up reorients to zero, got {velocity}"
            );
        }

        [Test]
        public void ProjectOnGrounding_InBetween_InterpolatesMagnitude_AndLandsOnSlope()
        {
            // A velocity between "along the slope" and "along up" keeps a fraction of its magnitude and lands ON the
            // slope line (zero component along the ground normal).
            const float slopeDeg = 25f;
            var groundNormal = Dir(90f + slopeDeg);
            var velocity = new float2(4f, 2f); // a mix of horizontal and vertical
            float originalMag = length(velocity);

            ProjectGround(ref velocity, groundNormal);

            Assert.AreEqual(
                0f,
                dot(velocity, groundNormal),
                Eps,
                "reoriented velocity lies on the slope line"
            );
            Assert.Greater(length(velocity), 0f, "an in-between velocity keeps some magnitude");
            Assert.Less(
                length(velocity),
                originalMag + Eps,
                "magnitude not amplified by the reorient"
            );
        }

        static void ProjectGround(ref float2 velocity, float2 groundNormal)
        {
            KinematicCharacterUtilities2D.ProjectVelocityOnGrounding(
                ref velocity,
                groundNormal,
                new float2(0f, 1f)
            );
        }

        // ===================== MathUtilities2D — round-trips and length preservation =====================

        [Test]
        public void Math_AngleDirectionRoundTrip(
            [Values(-179f, -90f, -33f, 0f, 1f, 47f, 90f, 178f)] float deg
        )
        {
            // AngleOfDirection(RightFromAngle(a)) == a, compared as a direction so the ±180 wrap is not a false fail.
            float rad = radians(deg);
            var dir = MathUtilities2D.RightFromAngle(rad);
            float recovered = MathUtilities2D.AngleOfDirection(dir);
            var reDir = MathUtilities2D.RightFromAngle(recovered);
            Assert.IsTrue(
                Approx(reDir, dir),
                $"angle {deg} did not round-trip: {recovered * (180f / PI)}deg"
            );
        }

        [Test]
        public void Math_UpRightOrthonormal([Values(0f, 23f, 90f, 200f)] float deg)
        {
            float rad = radians(deg);
            var up = MathUtilities2D.UpFromAngle(rad);
            var right = MathUtilities2D.RightFromAngle(rad);
            Assert.AreEqual(1f, length(up), Eps, "up is unit length");
            Assert.AreEqual(1f, length(right), Eps, "right is unit length");
            Assert.AreEqual(0f, dot(up, right), Eps, "up and right are orthogonal");
            // The documented relation perp(right) == up.
            Assert.IsTrue(Approx(MathUtilities2D.perp(right), up), "perp(right) == up");
        }

        [Test]
        public void Math_ReorientOnLine_PreservesLength_AndMatchesThe3DDoubleCross(
            [Values(0f, 30f, 75f, 120f, 200f)] float lineDeg,
            [Values(0f, 45f, 135f, 250f)] float velDeg
        )
        {
            // ReorientVectorOnPlaneAlongDirection2D must preserve the vector's length, put the result on the line
            // (zero component along the line's normal), and reproduce the 3D double-cross sign — NOT a dot-based
            // half-line pick (which flips the wrong way when alongDirection is perpendicular to the line). The
            // expected direction is the planar reduction of normalize(cross(n, cross(v, along))) =
            // normalize(-cross2(v, along) * perp(n)), computed here independently from the implementation.
            var lineNormal = Dir(lineDeg);
            var vector = Dir(velDeg) * 3f;
            var along = new float2(1f, 0.2f);

            var result = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                vector,
                lineNormal,
                along
            );

            Assert.AreEqual(
                0f,
                dot(result, lineNormal),
                Eps,
                "result lies on the line (no normal component)"
            );
            Assert.AreEqual(3f, length(result), Eps, "length preserved by the reorient");

            float z = vector.x * along.y - vector.y * along.x; // cross2(vector, along)
            var expectedDir = normalize(-z * MathUtilities2D.perp(lineNormal));
            // Skip the degenerate case where vector is parallel to along (z==0 → no defined sign).
            if (abs(z) > 1e-3f)
                Assert.IsTrue(
                    Approx(normalize(result), expectedDir),
                    "result matches the 3D double-cross direction"
                );
        }

        [Test]
        public void Math_ReorientOnLine_FlatGroundWalkRight_KeepsRightward_Not_Left()
        {
            // The exact case the C4a reduction got wrong: a character walking +X on flat ground. The ground line is
            // horizontal (normal +Y) and alongDirection is grounding-up (+Y, perpendicular to the line). The 3D
            // double-cross keeps the velocity +X; a dot(lineDir, up)-based pick flips it to -X (the dot is zero, so
            // it never flips off the arbitrary perp(up) = -X). This regression pins the +X result.
            var result = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                new float2(4f, 0f),
                new float2(0f, 1f),
                new float2(0f, 1f)
            );
            Assert.Greater(result.x, 0f, $"walking +X on flat ground must stay +X, got {result}");
            Assert.AreEqual(4f, result.x, Eps, "magnitude preserved on the flat ground line");
            Assert.AreEqual(0f, result.y, Eps, "stays on the flat ground line");
        }

        [Test]
        public void Math_ReorientOnLine_ZeroVector_ReturnsZero()
        {
            var result = MathUtilities2D.ReorientVectorOnPlaneAlongDirection2D(
                Unity.Mathematics.float2.zero,
                new float2(0f, 1f),
                new float2(1f, 0f)
            );
            Assert.IsTrue(
                Approx(result, Unity.Mathematics.float2.zero),
                "a zero input vector reorients to zero (no NaN)"
            );
        }

        [Test]
        public void Math_ProjectOnPlane_RemovesNormalComponent(
            [Values(0f, 37f, 91f, 200f)] float normalDeg
        )
        {
            var n = Dir(normalDeg);
            var v = new float2(3f, -5f);
            var projected = MathUtilities2D.ProjectOnPlane(v, n);
            Assert.AreEqual(0f, dot(projected, n), Eps, "projection removes the normal component");
            // The tangential part is preserved: projecting twice is idempotent.
            var twice = MathUtilities2D.ProjectOnPlane(projected, n);
            Assert.IsTrue(Approx(twice, projected), "ProjectOnPlane is idempotent");
        }

        // ---- Future-slope angle sign (gate 4, symptom 2) ----------------------------------------------------
        //
        // CalculateAngleOfHitWithGroundUp signs the slope change so the future-slope feature's downward-ledge test
        // — degrees(angle) < -MaxDownwardSlopeChangeAngle (KinematicCharacterUtilities2D.Update_PreventGrounding-
        // FromFutureSlopeChange, the 2D port of REF :852) — fires only on a DOWNWARD change. A wrong sign would
        // never unground at a downward ledge (or would unground spuriously on an upward step), and the C4b
        // deliverable flagged this 2D reduction as its least-certain. These tests are the direct, deterministic
        // arbiter of the sign that the C4a/C4b gates left unexercised: a downward slope change (relative to the
        // current ground, along the movement direction) must yield a NEGATIVE angle; an upward change a POSITIVE
        // one. The convention matches the 3D: walking +X over flat ground (up = +Y) onto a surface that tilts DOWN
        // ahead (its normal leans forward, +X-ward) is a negative angle; onto a surface that tilts UP ahead (normal
        // leans back, -X-ward) is positive.

        [Test]
        public void Math_AngleOfHitWithGroundUp_DownwardSlopeChange_IsNegative()
        {
            var groundingUp = new float2(0f, 1f);
            var currentGroundUp = new float2(0f, 1f); // currently on flat ground
            var velocityDir = new float2(1f, 0f); // walking +X
            // A surface tilting DOWN to the right (a downhill ahead): its outward normal leans in the +X (forward)
            // direction, e.g. a 30° downslope to the right → normal = (sin30, cos30) = (0.5, 0.866).
            var downhillNormal = normalize(new float2(0.5f, 0.866f));

            float angle = KinematicCharacterUtilities2D.CalculateAngleOfHitWithGroundUp(
                currentGroundUp,
                downhillNormal,
                velocityDir,
                groundingUp
            );

            Assert.Less(
                degrees(angle),
                0f,
                $"a downward slope change in the movement direction must be a NEGATIVE angle (so the "
                    + $"degrees(angle) < -MaxDownwardSlopeChangeAngle ledge test can fire); got {degrees(angle)}°"
            );
            Assert.AreEqual(
                -30f,
                degrees(angle),
                0.5f,
                "magnitude is the 30° slope change between the two normals"
            );
        }

        [Test]
        public void Math_AngleOfHitWithGroundUp_UpwardSlopeChange_IsPositive()
        {
            var groundingUp = new float2(0f, 1f);
            var currentGroundUp = new float2(0f, 1f);
            var velocityDir = new float2(1f, 0f); // walking +X
            // A surface tilting UP to the right (an uphill ahead): its outward normal leans in the -X (backward)
            // direction, e.g. a 30° upslope to the right → normal = (-sin30, cos30) = (-0.5, 0.866).
            var uphillNormal = normalize(new float2(-0.5f, 0.866f));

            float angle = KinematicCharacterUtilities2D.CalculateAngleOfHitWithGroundUp(
                currentGroundUp,
                uphillNormal,
                velocityDir,
                groundingUp
            );

            Assert.Greater(
                degrees(angle),
                0f,
                $"an upward slope change in the movement direction must be a POSITIVE angle (so it never trips the "
                    + $"downward-ledge ungrounding test); got {degrees(angle)}°"
            );
            Assert.AreEqual(
                30f,
                degrees(angle),
                0.5f,
                "magnitude is the 30° slope change between the two normals"
            );
        }

        [Test]
        public void Math_AngleOfHitWithGroundUp_SignFollowsMovementDirection()
        {
            // The sign is RELATIVE to the movement direction: the SAME downhill-to-the-right surface is a downward
            // change when walking +X (toward the downhill) but an upward change when walking -X (toward the uphill
            // it represents from the other side). This pins that the sign tracks velocityDirection, not a fixed
            // world axis — the property the 3D measures about velocityRight and the 2D reduction must preserve.
            var groundingUp = new float2(0f, 1f);
            var currentGroundUp = new float2(0f, 1f);
            var surfaceNormal = normalize(new float2(0.5f, 0.866f)); // tilts down to the right

            float walkingRight = KinematicCharacterUtilities2D.CalculateAngleOfHitWithGroundUp(
                currentGroundUp,
                surfaceNormal,
                new float2(1f, 0f),
                groundingUp
            );
            float walkingLeft = KinematicCharacterUtilities2D.CalculateAngleOfHitWithGroundUp(
                currentGroundUp,
                surfaceNormal,
                new float2(-1f, 0f),
                groundingUp
            );

            Assert.Less(
                degrees(walkingRight),
                0f,
                "walking toward the downhill is a downward (negative) change"
            );
            Assert.Greater(
                degrees(walkingLeft),
                0f,
                "walking the other way the same surface is an upward (positive) change"
            );
        }

        // ---- step-up slope-width check direction (the 606d056 regression: a degenerate-zero forwardSlopeCheckDirection) -

        // The step-up slope-width down-probe samples along StepTopUpSlopeTangent2D to read the slope ahead of the
        // step lip. The direction MUST be the up-slope tangent (a positive grounding-up component), the 2D reduction
        // of the 3D's -normalize(cross(cross(up, n), n)). 606d056 used ReorientVectorOnPlaneAlongDirection2D(up, n,
        // up), which is DEGENERATE — vector == alongDirection, so it returns (0,0) for EVERY step normal, dropping
        // the probe straight down at the lip and over-rejecting an in-range step at a step+slope corner. These cases
        // are RED on that zero-direction and GREEN on the correct tangent.

        [Test]
        public void Math_StepTopUpSlopeTangent_AngledTop_PointsUpSlope_NotZero(
            [Values(15f, 30f, 75f)] float slopeDeg,
            [Values(-1f, +1f)] float normalLeanSign
        )
        {
            // A step top whose normal leans by slopeDeg to the left (normalLeanSign +1 → normal.x < 0 → surface
            // rises to the RIGHT) or to the right (−1 → rises LEFT). The tangent must be a non-zero unit vector with
            // a POSITIVE grounding-up component (it leans up-into the surface), on the step-top plane (perpendicular
            // to the normal). The degenerate zero-direction (606d056) is RED on the non-zero + up-component checks.
            var up = new float2(0f, 1f);
            float r = radians(slopeDeg);
            // normalLeanSign +1 → normal.x = −sin (leans left) → surface rises right.
            var n = normalize(new float2(normalLeanSign * -sin(r), cos(r)));

            var t = MathUtilities2D.StepTopUpSlopeTangent2D(n, up);

            Assert.Greater(
                length(t),
                0.5f,
                $"the slope-check tangent must be non-zero for an angled step top ({slopeDeg}°) — a degenerate "
                    + "zero-direction (the 606d056 mis-port) samples straight down at the lip and over-rejects the step"
            );
            Assert.Less(abs(length(t) - 1f), Eps, "the tangent must be unit length");
            Assert.Greater(
                t.y,
                0.05f,
                "the tangent must lean UP-into the slope (positive grounding-up component)"
            );
            Assert.Less(
                abs(dot(t, n)),
                Eps,
                "the tangent must lie on the step-top plane (perpendicular to the normal)"
            );
            // Orientation-aware: normal leaning LEFT (rises right) → up-RIGHT tangent (+X); leaning right → up-LEFT.
            if (normalLeanSign > 0f)
                Assert.Greater(
                    t.x,
                    0f,
                    "a top whose normal leans left (surface rises right) must give an up-RIGHT tangent"
                );
            else
                Assert.Less(
                    t.x,
                    0f,
                    "a top whose normal leans right (surface rises left) must give an up-LEFT tangent"
                );
        }

        [Test]
        public void Math_StepTopUpSlopeTangent_FlatTop_IsZero()
        {
            // A flat step top (normal == grounding-up) has no slope, so the tangent is zero — the probe then samples
            // straight down at the contact, correctly finding the flat top (0° → no extra height → step-up engages).
            var up = new float2(0f, 1f);
            var t = MathUtilities2D.StepTopUpSlopeTangent2D(up, up);
            Assert.Less(length(t), Eps, "a flat step top must give a zero tangent");
        }

        [Test]
        public void Math_StepTopUpSlopeTangent_MatchesThe3DDoubleCross(
            [Values(10f, 25f, 50f, 80f)] float slopeDeg,
            [Values(-1f, +1f)] float risesSign
        )
        {
            // Pin the 2D reduction exactly to the 3D's -normalize(cross(cross(up, n), n)) (REF:3998), computed here
            // with z-only intermediate cross products. The degenerate 606d056 form does NOT match this.
            var up3 = new float3(0f, 1f, 0f);
            float r = radians(slopeDeg);
            var n2 = normalize(new float2(risesSign * -sin(r), cos(r)));
            var n3 = new float3(n2.x, n2.y, 0f);

            float3 threeD = -normalize(cross(cross(up3, n3), n3));
            var t = MathUtilities2D.StepTopUpSlopeTangent2D(n2, new float2(0f, 1f));

            Assert.Less(
                abs(t.x - threeD.x),
                Eps,
                $"x must match the 3D double-cross ({slopeDeg}°)"
            );
            Assert.Less(
                abs(t.y - threeD.y),
                Eps,
                $"y must match the 3D double-cross ({slopeDeg}°)"
            );
        }
    }
}
