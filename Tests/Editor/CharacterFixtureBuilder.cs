using System.Collections.Generic;
using System.IO;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Zori.Entities.CharacterController2D.Authoring;
using PhysicsBody2DAuthoring = Zori.Entities.Physics2D.Authoring.PhysicsBody2DAuthoring;
using PhysicsBody2DMotionType = Zori.Entities.Physics2D.Authoring.PhysicsBody2DMotionType;
using PhysicsCollisionResponse2D = Zori.Entities.Physics2D.PhysicsCollisionResponse2D;
using PhysicsShape2DAuthoring = Zori.Entities.Physics2D.Authoring.PhysicsShape2DAuthoring;
using PhysicsShape2DKind = Zori.Entities.Physics2D.PhysicsShape2DKind;

namespace Zori.Entities.CharacterController2D.Tests.Editor
{
    /// <summary>
    /// Click-free, reproducible authoring of the C4a behavioural-gate PlayMode fixtures — one parent scene per
    /// gate, each carrying a SubScene with a kinematic <see cref="CharacterController2DAuthoring"/> character and
    /// the static world geometry (floor / wall / slope / overlap) the gate exercises. Mirrors the substrate's
    /// fixture builders (<c>FallingBodyFixtureBuilder</c>, <c>ColliderShapeFixtureBuilder</c>): the static world is
    /// collider-only <see cref="BoxCollider2D"/> GameObjects (no <see cref="Rigidbody2D"/>) so the substrate's
    /// static-body fallback bakes them, the character is a <see cref="CharacterController2DAuthoring"/> that the
    /// controller baker turns into a live kinematic Box2D body + the character archetype, and both parent + child are
    /// registered in build settings so the runtime test reaches them by <c>SceneManager.LoadScene</c>.
    /// </summary>
    /// <remarks>
    /// SubScene authoring is edit-time, so it lives in this editor-only assembly, not the runtime PlayMode test.
    /// Run via the menu or
    /// <c>-executeMethod Zori.Entities.CharacterController2D.Tests.Editor.CharacterFixtureBuilder.BuildAll</c>
    /// before the PlayMode gate. The runtime test (CharacterSolveGate) adds the <c>DefaultCharacterController2DTag</c>
    /// to the baked character at runtime (the baker does not author it — the tag is the opt-in to the default solve),
    /// then drives the FixedStepSimulationSystemGroup and asserts LocalToWorld invariants.
    /// </remarks>
    public static class CharacterFixtureBuilder
    {
        public const string FixtureRoot = "Assets/CharacterController2DFixture";

        public const string GroundingParent = FixtureRoot + "/CC2D_Grounding.unity";
        public const string GroundingChild = FixtureRoot + "/CC2D_Grounding_Sub.unity";
        public const string WallParent = FixtureRoot + "/CC2D_Wall.unity";
        public const string WallChild = FixtureRoot + "/CC2D_Wall_Sub.unity";
        public const string SlopeParent = FixtureRoot + "/CC2D_Slope.unity";
        public const string SlopeChild = FixtureRoot + "/CC2D_Slope_Sub.unity";
        public const string WallGroundedParent = FixtureRoot + "/CC2D_WallGrounded.unity";
        public const string WallGroundedChild = FixtureRoot + "/CC2D_WallGrounded_Sub.unity";
        public const string OverlapShallowParent = FixtureRoot + "/CC2D_OverlapShallow.unity";
        public const string OverlapShallowChild = FixtureRoot + "/CC2D_OverlapShallow_Sub.unity";
        public const string OverlapDeepParent = FixtureRoot + "/CC2D_OverlapDeep.unity";
        public const string OverlapDeepChild = FixtureRoot + "/CC2D_OverlapDeep_Sub.unity";
        public const string OverlapGroundParent = FixtureRoot + "/CC2D_OverlapGround.unity";
        public const string OverlapGroundChild = FixtureRoot + "/CC2D_OverlapGround_Sub.unity";

        // ---- C4b advanced-feature fixtures -----------------------------------------------------------------
        public const string StepLowParent = FixtureRoot + "/CC2D_StepLow.unity";
        public const string StepLowChild = FixtureRoot + "/CC2D_StepLow_Sub.unity";
        public const string StepHighParent = FixtureRoot + "/CC2D_StepHigh.unity";
        public const string StepHighChild = FixtureRoot + "/CC2D_StepHigh_Sub.unity";
        public const string JumpParent = FixtureRoot + "/CC2D_Jump.unity";
        public const string JumpChild = FixtureRoot + "/CC2D_Jump_Sub.unity";
        public const string CharCharParent = FixtureRoot + "/CC2D_CharChar.unity";
        public const string CharCharChild = FixtureRoot + "/CC2D_CharChar_Sub.unity";
        public const string DynamicPushParent = FixtureRoot + "/CC2D_DynamicPush.unity";
        public const string DynamicPushChild = FixtureRoot + "/CC2D_DynamicPush_Sub.unity";
        public const string DynamicPushHeavyParent = FixtureRoot + "/CC2D_DynamicPushHeavy.unity";
        public const string DynamicPushHeavyChild =
            FixtureRoot + "/CC2D_DynamicPushHeavy_Sub.unity";
        public const string MovingPlatformParent = FixtureRoot + "/CC2D_MovingPlatform.unity";
        public const string MovingPlatformChild = FixtureRoot + "/CC2D_MovingPlatform_Sub.unity";

        // ---- P0 capsule-proxy fixtures (the capsule character mandate) -------------------------------------
        public const string CapsuleStepLowParent = FixtureRoot + "/CC2D_CapsuleStepLow.unity";
        public const string CapsuleStepLowChild = FixtureRoot + "/CC2D_CapsuleStepLow_Sub.unity";
        public const string CapsuleGroundParent = FixtureRoot + "/CC2D_CapsuleGround.unity";
        public const string CapsuleGroundChild = FixtureRoot + "/CC2D_CapsuleGround_Sub.unity";
        public const string CapsuleWallParent = FixtureRoot + "/CC2D_CapsuleWall.unity";
        public const string CapsuleWallChild = FixtureRoot + "/CC2D_CapsuleWall_Sub.unity";

        // ---- step + ADJACENT-SLOPE fixtures (the directional step-up lateral-jump regression) --------------
        // A low step (top 0.3, mountable) whose TOP-RIGHT corner an upward slope rises from — the
        // Platformer course's step-next-to-slope cluster. The character is spawned on the LOWER floor to the
        // LEFT (walks +X up onto the step, with the slope rising off the step's far edge) in the *L2R fixture,
        // and on the slope/ledge to the RIGHT (walks −X down onto the step) in the *R2L fixture, so the same
        // step+slope junction is approached from both directions. Capsule + box variants of each.
        public const string CapsuleStepSlopeL2RParent =
            FixtureRoot + "/CC2D_CapsuleStepSlopeL2R.unity";
        public const string CapsuleStepSlopeL2RChild =
            FixtureRoot + "/CC2D_CapsuleStepSlopeL2R_Sub.unity";
        public const string CapsuleStepSlopeR2LParent =
            FixtureRoot + "/CC2D_CapsuleStepSlopeR2L.unity";
        public const string CapsuleStepSlopeR2LChild =
            FixtureRoot + "/CC2D_CapsuleStepSlopeR2L_Sub.unity";
        public const string BoxStepSlopeL2RParent = FixtureRoot + "/CC2D_BoxStepSlopeL2R.unity";
        public const string BoxStepSlopeL2RChild = FixtureRoot + "/CC2D_BoxStepSlopeL2R_Sub.unity";
        public const string BoxStepSlopeR2LParent = FixtureRoot + "/CC2D_BoxStepSlopeR2L.unity";
        public const string BoxStepSlopeR2LChild = FixtureRoot + "/CC2D_BoxStepSlopeR2L_Sub.unity";

        // The LITERAL Platformer Station-2 cluster reproduced VERBATIM (same X spacing, same gaps, the same rotated
        // 30° ramp box) so the step-next-to-slope junction the user reports is exercised with its real geometry,
        // not the gap-removed synthetic above. Shifted −22 in X so StepLow's left face lands at X=2 and the spawn
        // numbers stay small. Both approach directions, capsule (the platformer proxy).
        public const string ClusterL2RParent = FixtureRoot + "/CC2D_ClusterL2R.unity";
        public const string ClusterL2RChild = FixtureRoot + "/CC2D_ClusterL2R_Sub.unity";
        public const string ClusterR2LParent = FixtureRoot + "/CC2D_ClusterR2L.unity";
        public const string ClusterR2LChild = FixtureRoot + "/CC2D_ClusterR2L_Sub.unity";

        // The capsule SPAWNED wedged in the diagonal-ramp + step corner (an initial overlap), so the per-step
        // SolveOverlaps depenetration fires against a skewed-normal contact — the path c719d90 patched. This is the
        // state the walk-in fixtures never reach (they approach cleanly); the user's "teleport/propel" is at a
        // body already overlapping the diagonal beam at the step.
        public const string CornerWedgeParent = FixtureRoot + "/CC2D_CornerWedge.unity";
        public const string CornerWedgeChild = FixtureRoot + "/CC2D_CornerWedge_Sub.unity";

        // The c719d90 suspect probe: a capsule resting ON a wide flat block but spawned with its centre well past
        // the block's centre, so the body→character axis (the cast-back's separating-axis assumption) is nearly
        // HORIZONTAL while the correct depenetration normal is VERTICAL (the block top). c719d90 scales the
        // recovered depth by |dot(dirToCharacter, normal)| ≈ 0 there, so a real overlap is reported as ~zero depth
        // and the character is NOT pushed out — it sinks. Drives a small +X and asserts no-penetration + no-fling.
        public const string SkewBlockParent = FixtureRoot + "/CC2D_SkewBlock.unity";
        public const string SkewBlockChild = FixtureRoot + "/CC2D_SkewBlock_Sub.unity";

        // The STEEP-ramp depenetration probe — the real "X-crossed beam" trigger. A capsule grounded on flat floor
        // is pushed up against a 75° OVER-LIMIT ramp (the Platformer SlopeOverLimit_75deg). Its surface normal is
        // ~15° off horizontal, so the recovered overlap normal has dot(up, n) ≈ 0.26 — inside the reverse-projection
        // amplification band (> MinDotRatioForVerticalDecollision 0.1), where DecollideFromHit divides the
        // ReconstructOverlap depth by that small dot (clamped at depth*10). Combined with ReconstructOverlap's
        // over-reported depth (the A/B-confirmed inflation), this is the multi-unit fling the user sees.
        public const string SteepRampParent = FixtureRoot + "/CC2D_SteepRamp.unity";
        public const string SteepRampChild = FixtureRoot + "/CC2D_SteepRamp_Sub.unity";

        // Same steep-ramp geometry, but step handling OFF — the bisection probe for whether the spurious
        // step-grounding on the over-limit ramp is what pumps the velocity.
        public const string SteepRampNoStepParent = FixtureRoot + "/CC2D_SteepRampNoStep.unity";
        public const string SteepRampNoStepChild = FixtureRoot + "/CC2D_SteepRampNoStep_Sub.unity";

        // ---- sensor / trigger pass-through fixture (the negative-space case: a kinematic character must NOT
        // ground on a sensor; it passes through and the trigger Begin fires) -------------------------------
        public const string SensorPassThroughParent = FixtureRoot + "/CC2D_SensorPassThrough.unity";
        public const string SensorPassThroughChild =
            FixtureRoot + "/CC2D_SensorPassThrough_Sub.unity";

        // ---- gate-4 future-slope / downward-ledge fixtures -------------------------------------------------
        public const string DownLedgeParent = FixtureRoot + "/CC2D_DownLedge.unity";
        public const string DownLedgeChild = FixtureRoot + "/CC2D_DownLedge_Sub.unity";
        public const string DownSlopeGentleParent = FixtureRoot + "/CC2D_DownSlopeGentle.unity";
        public const string DownSlopeGentleChild = FixtureRoot + "/CC2D_DownSlopeGentle_Sub.unity";
        public const string DownSlopeSteepParent = FixtureRoot + "/CC2D_DownSlopeSteep.unity";
        public const string DownSlopeSteepChild = FixtureRoot + "/CC2D_DownSlopeSteep_Sub.unity";

        // Step geometry: the lower step is below MaxStepHeight (0.5), the higher step is above it. The runtime
        // gate asserts the character steps up onto the low one and is blocked by the high one. The step's top
        // surface Y is what the character's grounded settle height tracks after stepping up.
        public const float LowStepTopY = 0.3f;
        public const float HighStepTopY = 0.9f;

        // Dynamic-push: the box's left face starts here so the character (walking +X) reaches it. The two
        // push fixtures differ only in the box's authored mass — the adversarial "heavier pushes less" pair.
        public const float DynamicBoxLeftFaceX = 1.6f;
        public const float DynamicBoxLightMass = 1f;
        public const float DynamicBoxHeavyMass = 50f;

        // Moving-platform: the platform top surface. The lateral drive speed lives in the runtime-side
        // CharacterFixtureBuilderConstants.PlatformSpeedX (the runtime gate drives it; one canonical home).
        public const float PlatformTopY = 0f;

        // The character circle proxy radius used across all fixtures — the runtime test asserts the grounded settle
        // height against this.
        public const float CharacterRadius = 0.5f;

        // The capsule character: a vertical capsule of full extents (width, height). Width 1 → cap radius 0.5;
        // height 2 → end caps at ±0.5, so the bottom-most point is centre − (0.5 + 0.5) = centre − 1.0. The
        // grounded settle puts the centre ~1.0 + CollisionOffset above the surface it rests on. The runtime gate
        // asserts the capsule grounds, slides, and steps up using these dims.
        public const float CapsuleWidth = 1f;
        public const float CapsuleHeight = 2f;

        // Distance from the capsule centre to its bottom-most point (cap radius + cap offset).
        public const float CapsuleBottomReach = CapsuleHeight * 0.5f;

        // gate-4 future-slope / downward-ledge geometry. The downward-ledge platform ends at LedgeEdgeX; the
        // downslope fixtures put the flat-to-downhill lip at SlopeLipX. The gentle downhill is within the gate's
        // configured max downward slope-change angle, the steep one is over it — see CharacterFixtureBuilder-
        // Constants.MaxDownwardSlopeChangeForGate (the runtime gate sets the same value on the character).
        public const float LedgeEdgeX = 3f;
        public const float SlopeLipX = 0f;
        public const float GentleDownSlopeDeg = 20f;
        public const float SteepDownSlopeDeg = 55f;

        // Floor top surface sits at Y=0 (a thin box centred just below 0). The runtime gate
        // (CharacterFixtureBuilderConstants.FloorTopY) hard-codes the same 0 — kept in sync by convention rather than
        // a cross-assembly reference (the editor builder and the all-platforms runtime test do not reference each
        // other).
        public const float FloorTopY = 0f;

        // Sensor pass-through fixture: a SENSOR (isTrigger) box straddling the character's fall path, with a SOLID
        // floor below it. A correct controller passes the character THROUGH the sensor (firing a trigger Begin) and
        // lands it on the solid floor below; the bug grounds the character ON the sensor. The sensor box is centred
        // at Y=0 with a 2-tall extent (so its volume is Y∈[-1,1]); the solid floor top sits at SensorSolidFloorTopY,
        // well below the sensor. The runtime gate references these (kept in sync by convention).
        public const float SensorBoxCenterY = 0f;
        public const float SensorBoxHalfHeight = 1f;
        public const float SensorSolidFloorTopY = -4f;

        [MenuItem("Tools/Zori/Build Character Controller 2D Gate Fixtures")]
        public static void BuildAll()
        {
            Directory.CreateDirectory(FixtureRoot);

            BuildGrounding();
            BuildWall();
            BuildSlope();
            BuildWallGrounded();
            BuildOverlapShallow();
            BuildOverlapDeep();
            BuildOverlapGround();

            // C4b advanced features.
            BuildStepLow();
            BuildStepHigh();
            BuildJump();
            BuildCharChar();
            BuildDynamicPush();
            BuildDynamicPushHeavy();
            BuildMovingPlatform();

            // gate-4 future-slope / downward-ledge.
            BuildDownLedge();
            BuildDownSlopeGentle();
            BuildDownSlopeSteep();

            // Sensor / trigger pass-through (the negative-space regression case).
            BuildSensorPassThrough();

            // P0 capsule-proxy character (the capsule mandate).
            BuildCapsuleGround();
            BuildCapsuleWall();
            BuildCapsuleStepLow();

            // Step + adjacent-slope, both approach directions, capsule + box (the directional step-up regression).
            BuildCapsuleStepSlopeL2R();
            BuildCapsuleStepSlopeR2L();
            BuildBoxStepSlopeL2R();
            BuildBoxStepSlopeR2L();

            // The literal Platformer Station-2 cluster (verbatim spacing + the rotated ramp), both directions.
            BuildClusterL2R();
            BuildClusterR2L();
            BuildCornerWedge();
            BuildSkewBlock();
            BuildSteepRamp();
            BuildSteepRampNoStep();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("Character Controller 2D gate fixtures built.");
        }

        // ---- individual fixtures ---------------------------------------------------------------------------

        // Grounding: a character dropped a little above a flat floor → settles at ~radius above the floor top.
        static void BuildGrounding()
        {
            BuildFixture(
                GroundingChild,
                GroundingParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddCharacter(root, new Vector3(0f, 3f, 0f));
                }
            );
        }

        // Collide-and-slide into a wall: character starts grounded near the floor, the test drives +X velocity into
        // a vertical wall whose left face is at X=3. X must clamp before the proxy penetrates the wall.
        static void BuildWall()
        {
            BuildFixture(
                WallChild,
                WallParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Wall: a tall thin box whose left face is at X = 3 (centre at 3 + 0.5).
                    AddFloor(root, new Vector2(3.5f, 3f), new Vector2(1f, 10f));
                    AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                }
            );
        }

        // Slope within limit: a 30-degree ramp the character climbs. The ramp is a rotated thin box; the character
        // starts on the low (left) end and the test drives +X velocity up it.
        static void BuildSlope()
        {
            BuildFixture(
                SlopeChild,
                SlopeParent,
                root =>
                {
                    var ramp = AddFloor(root, new Vector2(2f, 0f), new Vector2(20f, 1f));
                    ramp.transform.rotation = Quaternion.Euler(0f, 0f, 30f);
                    // Character starts above the low end of the ramp (left side), close to its top surface.
                    AddCharacter(root, new Vector3(-4f, 1.5f, 0f));
                }
            );
        }

        // Wall while grounded: character grounded on a flat floor, walks into a vertical wall. It must slide along
        // (stay grounded, stay near floor height) rather than climb or penetrate.
        static void BuildWallGrounded()
        {
            BuildFixture(
                WallGroundedChild,
                WallGroundedParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddFloor(root, new Vector2(3.5f, 3f), new Vector2(1f, 10f));
                    AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                }
            );
        }

        // Depenetration shallow: character spawned overlapping a wall by a little less than the radius.
        static void BuildOverlapShallow()
        {
            BuildFixture(
                OverlapShallowChild,
                OverlapShallowParent,
                root =>
                {
                    // Wall left face at X = 0. Character centre at X = 0.2 → overlapping by ~0.3 (radius 0.5).
                    AddFloor(root, new Vector2(2.5f, 5f), new Vector2(5f, 20f));
                    var c = AddCharacter(root, new Vector3(0.2f, 5f, 0f));
                    // No grounding/gravity confound: this is a pure horizontal-overlap probe.
                    DisableGrounding(c);
                }
            );
        }

        // Depenetration DEEP (the adversarial probe): character centre well INSIDE the wall, deeper than the
        // collision offset and approaching the radius. The cast-back must still recover a push-out direction and the
        // iterative loop converge to no residual overlap, with no NaN / teleport.
        static void BuildOverlapDeep()
        {
            BuildFixture(
                OverlapDeepChild,
                OverlapDeepParent,
                root =>
                {
                    // Wall left face at X = 0; character centre at X = -0.45 → the proxy centre is 0.45 INSIDE the
                    // wall, much deeper than CollisionOffset (0.01) and nearly the full radius.
                    AddFloor(root, new Vector2(2.5f, 5f), new Vector2(5f, 20f));
                    var c = AddCharacter(root, new Vector3(-0.45f, 5f, 0f));
                    DisableGrounding(c);
                    // Give depenetration more iterations per step so a deep overlap resolves quickly and the test can
                    // assert convergence within a bounded number of fixed steps.
                    BumpOverlapIterations(c, 8);
                }
            );
        }

        // Depenetration against the GROUND while grounded: a character spawned sunk into the floor. It must pop
        // straight UP out of the floor (the grounded vertical-decollide branch), not sideways.
        static void BuildOverlapGround()
        {
            BuildFixture(
                OverlapGroundChild,
                OverlapGroundParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Character sunk so its centre is at Y = 0.1 — well below the radius-above-floor rest height
                    // (~0.51), so it is penetrating the floor by ~0.4.
                    var c = AddCharacter(root, new Vector3(0f, 0.1f, 0f));
                    BumpOverlapIterations(c, 8);
                }
            );
        }

        // ---- C4b advanced-feature fixtures -----------------------------------------------------------------

        // Step within limit: a grounded character walks +X into a low step (top at LowStepTopY = 0.3, below the
        // 0.5 MaxStepHeight). With step handling enabled it must climb onto the step top and stay grounded.
        static void BuildStepLow()
        {
            BuildFixture(
                StepLowChild,
                StepLowParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // The step: a box whose top is at LowStepTopY and whose left face is at X = 2.
                    AddFloor(root, new Vector2(4.5f, LowStepTopY - 1f), new Vector2(5f, 2f));
                    var c = AddBoxCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                    EnableStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // Step over limit: the same walk into a step whose top (HighStepTopY = 0.9) exceeds MaxStepHeight (0.5).
        // The character must be blocked (not climb), sliding into the step wall like an ordinary wall.
        static void BuildStepHigh()
        {
            BuildFixture(
                StepHighChild,
                StepHighParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddFloor(root, new Vector2(4.5f, HighStepTopY - 1f), new Vector2(5f, 2f));
                    var c = AddBoxCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                    EnableStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // ---- P0 capsule-proxy fixtures ---------------------------------------------------------------------

        // Capsule grounding: a CAPSULE character dropped above a flat floor → settles at ~CapsuleBottomReach above
        // the floor top, the capsule analogue of BuildGrounding (which uses a circle).
        static void BuildCapsuleGround()
        {
            BuildFixture(
                CapsuleGroundChild,
                CapsuleGroundParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddCapsuleCharacter(root, new Vector3(0f, CapsuleBottomReach + 2f, 0f));
                }
            );
        }

        // Capsule collide-and-slide: a grounded CAPSULE walks +X into a vertical wall whose left face is at X=3;
        // its right cap edge (centre + cap radius 0.5) must clamp at the wall face, no penetration.
        static void BuildCapsuleWall()
        {
            BuildFixture(
                CapsuleWallChild,
                CapsuleWallParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddFloor(root, new Vector2(3.5f, 3f), new Vector2(1f, 10f));
                    AddCapsuleCharacter(root, new Vector3(0f, CapsuleBottomReach + 0.05f, 0f));
                }
            );
        }

        // Capsule step-up (the UNVERIFIED case the box-only gate-4 fix left open): a grounded CAPSULE walks +X
        // into the same low step (top 0.3, below the 0.5 max). A capsule's rounded bottom should mount the step
        // at least as cleanly as a box (no top-left-corner catch). The runtime gate asserts it climbs and holds a
        // stable stand, the capsule analogue of Step_WalkIntoLowStep_StepsUp_HoldsStableStandOnStep.
        static void BuildCapsuleStepLow()
        {
            BuildFixture(
                CapsuleStepLowChild,
                CapsuleStepLowParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddFloor(root, new Vector2(4.5f, LowStepTopY - 1f), new Vector2(5f, 2f));
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(0f, CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // ---- step + ADJACENT-SLOPE fixtures (the directional step-up lateral-jump regression) --------------
        // The LITERAL Platformer Station-2 step+slope cluster (PlatformerSceneBuilder.BuildChild), shifted −24 in
        // X so StepLow's left face lands near X=0 and the spawn numbers stay small. Reproduced verbatim so the
        // user-observed left-to-right lateral jump at the step-next-to-slope spot is exercised here, not a guess:
        //   Platformer StepLow box (26,-0.7) size(4,2)  → top 0.3, X[24,28]   → shifted (2,-0.7),  top 0.3, X[0,4]
        //   Platformer StepHigh box (31,-0.6) size(4,3) → top 0.9, X[29,33]   → shifted (7,-0.6),  top 0.9, X[5,9]
        //   Platformer slope lip (34,0), len 12, 30°    → rising right        → shifted lip (10,0), rising right
        const float StepSlopeDeg = 30f; // within the 60° MaxGroundedSlopeAngle → a climbable slope
        const float StepSlopeApproachX = -3f; // L2R spawn (on the approach floor, left of StepLow)
        const float StepSlopeSlopeLipX = 10f;

        static void BuildStepSlopeWorld(GameObject root)
        {
            // Approach floor (top 0) under the spawn, up to StepLow's left face X=0. Platformer sticky floor X[14,24].
            AddFloor(root, new Vector2(-7f, FloorTopY - 0.5f), new Vector2(14f, 1f)); // top 0, X[-14,0]
            // StepLow: top LowStepTopY=0.3, X[0,4] (mountable, ≤ 0.5 max).
            AddFloor(root, new Vector2(2f, LowStepTopY - 1f), new Vector2(4f, 2f)); // top 0.3, X[0,4]
            // StepHigh: top 0.9, X[5,9] (over the 0.5 max — a wall the gate-3 high-step test proves is not climbed).
            AddFloor(root, new Vector2(7f, HighStepTopY - 1.5f), new Vector2(4f, 3f)); // top 0.9, X[5,9]
            // The 30° slope-within-limit, lip at (StepSlopeSlopeLipX, 0) at FLOOR level, len 12, rising right —
            // exactly the Platformer AddRamp (a thin box rotated about the lip). A short floor bridges StepHigh's
            // right edge (X=9) to the slope lip (X=10) so the slope lip is a step-up off flat floor as authored.
            AddFloor(root, new Vector2(9.5f, FloorTopY - 0.5f), new Vector2(1f, 1f)); // top 0, X[9,10]
            const float lengthAlong = 12f;
            var ramp = AddFloor(
                root,
                new Vector2(StepSlopeSlopeLipX + lengthAlong * 0.5f, FloorTopY - 0.5f),
                new Vector2(lengthAlong, 1f)
            );
            ramp.transform.RotateAround(
                new Vector3(StepSlopeSlopeLipX, FloorTopY, 0f),
                Vector3.forward,
                StepSlopeDeg
            );
            // A flat top LEDGE where the slope tops out (so the −X-walking character has a stable flat start, then
            // walks DOWN the slope into the step+slope corner — rather than spawning wedged on the slope itself).
            // The ramp top at X = StepSlopeSlopeLipX + lengthAlong*cos(30) reaches Y = lengthAlong*sin(30) = 6; put
            // a small flat ledge butted at the ramp's high corner.
            float topX = StepSlopeSlopeLipX + lengthAlong * Mathf.Cos(Mathf.Deg2Rad * StepSlopeDeg);
            float topY = lengthAlong * Mathf.Sin(Mathf.Deg2Rad * StepSlopeDeg);
            AddFloor(root, new Vector2(topX + 3f, topY - 0.5f), new Vector2(6f, 1f));
        }

        // The flat slope-top ledge spawn (top Y, a little right of the ramp's high corner) for the −X-walking
        // characters: a stable flat rest from which they walk DOWN the slope into the step+slope corner.
        static Vector3 R2LSpawnOnLedge(float bottomReach)
        {
            const float lengthAlong = 12f;
            float topX = StepSlopeSlopeLipX + lengthAlong * Mathf.Cos(Mathf.Deg2Rad * StepSlopeDeg);
            float topY = lengthAlong * Mathf.Sin(Mathf.Deg2Rad * StepSlopeDeg);
            return new Vector3(topX + 1.5f, topY + bottomReach + 0.05f, 0f);
        }

        static void BuildCapsuleStepSlopeL2R()
        {
            BuildFixture(
                CapsuleStepSlopeL2RChild,
                CapsuleStepSlopeL2RParent,
                root =>
                {
                    BuildStepSlopeWorld(root);
                    // Capsule on the approach floor, left of StepLow, walking +X into the step+slope cluster.
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(StepSlopeApproachX, CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        static void BuildCapsuleStepSlopeR2L()
        {
            BuildFixture(
                CapsuleStepSlopeR2LChild,
                CapsuleStepSlopeR2LParent,
                root =>
                {
                    BuildStepSlopeWorld(root);
                    var c = AddCapsuleCharacter(root, R2LSpawnOnLedge(CapsuleBottomReach));
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        static void BuildBoxStepSlopeL2R()
        {
            BuildFixture(
                BoxStepSlopeL2RChild,
                BoxStepSlopeL2RParent,
                root =>
                {
                    BuildStepSlopeWorld(root);
                    var c = AddBoxCharacter(
                        root,
                        new Vector3(StepSlopeApproachX, CharacterRadius + 0.05f, 0f)
                    );
                    EnableStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        static void BuildBoxStepSlopeR2L()
        {
            BuildFixture(
                BoxStepSlopeR2LChild,
                BoxStepSlopeR2LParent,
                root =>
                {
                    BuildStepSlopeWorld(root);
                    var c = AddBoxCharacter(root, R2LSpawnOnLedge(CharacterRadius));
                    EnableStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // ---- the LITERAL Platformer cluster (verbatim geometry, both directions) -----------------------------
        // PlatformerSceneBuilder.BuildChild Station-2, shifted −22 in X (so StepLow left face → X=2):
        //   StepLow box (26,-0.7) size(4,2)  → top 0.3, X[24,28]  → shifted (4,-0.7), top 0.3, X[2,6]
        //   StepHigh box (31,-0.6) size(4,3) → top 0.9, X[29,33]  → shifted (9,-0.6), top 0.9, X[7,11]
        //   slope lip (34,0), len 12, 30° rising right            → shifted lip (12,0), the rotated ramp box
        // The 1-unit GAPS the synthetic BuildStepSlopeWorld closes (X[6,7] between StepLow/StepHigh, X[11,12]
        // between StepHigh and the slope lip) are PRESERVED here — that open floor at the bottom of the steps is
        // exactly where the rotated ramp box's low end sweeps and the diagonal-normal contact the bug needs lives.
        const float ClusterLipX = 12f;
        const float ClusterRampLen = 12f;
        const float ClusterSlopeDeg = 30f;

        static void BuildClusterWorld(GameObject root)
        {
            // The continuous bottom floor (top Y=0) running under the whole cluster, including the 1-unit gaps
            // between the steps and up to the slope lip — the Platformer course's sticky floor butts the cluster
            // at floor level. X[−14, 12].
            AddFloor(root, new Vector2(-1f, FloorTopY - 0.5f), new Vector2(26f, 1f)); // top 0, X[−14,12]
            // StepLow: top 0.3, X[2,6] (mountable, ≤ 0.5 max).
            AddFloor(root, new Vector2(4f, LowStepTopY - 1f), new Vector2(4f, 2f));
            // StepHigh: top 0.9, X[7,11] (over the 0.5 max — a wall).
            AddFloor(root, new Vector2(9f, HighStepTopY - 1.5f), new Vector2(4f, 3f));
            // The 30° slope-within-limit, a thin box rotated about its lip (12,0) — the diagonal "beam" collider.
            var ramp = AddFloor(
                root,
                new Vector2(ClusterLipX + ClusterRampLen * 0.5f, FloorTopY - 0.5f),
                new Vector2(ClusterRampLen, 1f)
            );
            ramp.transform.RotateAround(
                new Vector3(ClusterLipX, FloorTopY, 0f),
                Vector3.forward,
                ClusterSlopeDeg
            );
            // The flat ledge the ramp tops out onto (so an R2L start has a stable flat rest above the slope).
            float topX = ClusterLipX + ClusterRampLen * Mathf.Cos(Mathf.Deg2Rad * ClusterSlopeDeg);
            float topY = ClusterRampLen * Mathf.Sin(Mathf.Deg2Rad * ClusterSlopeDeg);
            AddFloor(root, new Vector2(topX + 3f, topY - 0.5f), new Vector2(6f, 1f));
        }

        static void BuildClusterL2R()
        {
            BuildFixture(
                ClusterL2RChild,
                ClusterL2RParent,
                root =>
                {
                    BuildClusterWorld(root);
                    // Capsule on the bottom floor, a few units left of StepLow (X=2), walking +X into the cluster.
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(-2f, CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        static void BuildClusterR2L()
        {
            BuildFixture(
                ClusterR2LChild,
                ClusterR2LParent,
                root =>
                {
                    BuildClusterWorld(root);
                    // Capsule on the flat top ledge, walking −X down the slope into the step+slope corner.
                    float topX =
                        ClusterLipX + ClusterRampLen * Mathf.Cos(Mathf.Deg2Rad * ClusterSlopeDeg);
                    float topY = ClusterRampLen * Mathf.Sin(Mathf.Deg2Rad * ClusterSlopeDeg);
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(topX + 1.5f, topY + CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // The capsule spawned right at the step lip where the diagonal ramp beam meets the step top — overlapping
        // both the step block and the rotated ramp box at once. This forces SolveOverlaps to reconstruct two
        // overlaps with skewed normals on the first step (the c719d90 path) rather than approaching cleanly. The
        // runtime gate drives a gentle +X and asserts the same bounded-delta / no-propulsion decision points: an
        // initial-overlap depenetration must NOT fling or propel the character.
        static void BuildCornerWedge()
        {
            BuildFixture(
                CornerWedgeChild,
                CornerWedgeParent,
                root =>
                {
                    BuildClusterWorld(root);
                    // Spawn ON the slope just above the lip (X=12, the slope rises right at 30°), so the capsule's
                    // lower body overlaps the StepHigh block's right face (X=11) AND the diagonal ramp box at once.
                    // slope Y at X≈12.3 ≈ (12.3−12)·tan30 ≈ 0.17 → spawn centre a touch into the corner.
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(12.0f, FloorTopY + CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                }
            );
        }

        // The c719d90 skew probe. A WIDE low block (top 0.3, X[0,12], centre at X=6) and a capsule resting on top
        // but SUNK ~0.2 into it and placed at X=11 — far to the right of the block centre (X=6), so the
        // body→character axis is steeply HORIZONTAL while the correct push-out is straight UP (the block top
        // normal). This is exactly the geometry c719d90's |dot(dirToCharacter, normal)| factor collapses to ~0.
        static void BuildSkewBlock()
        {
            BuildFixture(
                SkewBlockChild,
                SkewBlockParent,
                root =>
                {
                    // The wide block: top 0.3, X[0,12], centre (6, -0.7), size (12,2).
                    AddFloor(root, new Vector2(6f, LowStepTopY - 1f), new Vector2(12f, 2f));
                    // Capsule sunk into the block top (centre Y below the clean rest height by ~0.2) at X=11, far
                    // right of the block centre X=6.
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(11f, LowStepTopY + CapsuleBottomReach - 0.2f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                    // Allow several depenetration iterations (default may be low); the gate asserts it pops out.
                    BumpOverlapIterations(c, 8);
                }
            );
        }

        // The steep-ramp fling probe (the real "X-crossed beam"). Flat floor (top 0, X[−10,10]); a 75° over-limit
        // ramp whose lip sits at (5,0) rising right, exactly the Platformer SlopeOverLimit_75deg. A capsule starts
        // grounded on the flat floor just left of the lip and is driven +X INTO the steep ramp — it cannot climb it
        // (over the 60° max), so it must collide-and-slide / depenetrate against the ramp's steep face while
        // grounded. The recovered overlap normal there is ~15° off horizontal, in the reverse-projection
        // amplification band. The gate asserts bounded per-step delta + no propulsion (the user's symptom).
        static void BuildSteepRamp()
        {
            BuildFixture(
                SteepRampChild,
                SteepRampParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(20f, 1f)); // top 0, X[−10,10]
                    // 75° ramp, a thin box rotated about its lip (5,0), len 7 (the Platformer dims).
                    const float lipX = 5f;
                    const float lenAlong = 7f;
                    var ramp = AddFloor(
                        root,
                        new Vector2(lipX + lenAlong * 0.5f, FloorTopY - 0.5f),
                        new Vector2(lenAlong, 1f)
                    );
                    ramp.transform.RotateAround(new Vector3(lipX, FloorTopY, 0f), Vector3.forward, 75f);
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(2f, CapsuleBottomReach + 0.05f, 0f)
                    );
                    EnableCapsuleStepHandling(c, maxStepHeight: 0.5f);
                    BumpOverlapIterations(c, 8);
                }
            );
        }

        static void BuildSteepRampNoStep()
        {
            BuildFixture(
                SteepRampNoStepChild,
                SteepRampNoStepParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(20f, 1f));
                    const float lipX = 5f;
                    const float lenAlong = 7f;
                    var ramp = AddFloor(
                        root,
                        new Vector2(lipX + lenAlong * 0.5f, FloorTopY - 0.5f),
                        new Vector2(lenAlong, 1f)
                    );
                    ramp.transform.RotateAround(new Vector3(lipX, FloorTopY, 0f), Vector3.forward, 75f);
                    var c = AddCapsuleCharacter(
                        root,
                        new Vector3(2f, CapsuleBottomReach + 0.05f, 0f)
                    );
                    // Step handling OFF (the bisection variant).
                    BumpOverlapIterations(c, 8);
                }
            );
        }

        // Jump: a grounded character on a flat floor. The runtime test calls StandardJump2D, then asserts the
        // character ungrounds, rises in Y, falls, and re-grounds.
        static void BuildJump()
        {
            BuildFixture(
                JumpChild,
                JumpParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                }
            );
        }

        // Character ↔ character: two SimulateDynamicBody characters spawned overlapping on a flat floor. The
        // hit-dynamics impulse exchange must separate them (each pushed away from the other along the contact).
        static void BuildCharChar()
        {
            BuildFixture(
                CharCharChild,
                CharCharParent,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Two characters overlapping: centres 0.6 apart, each radius 0.5, so they overlap by ~0.4.
                    // Both SimulateDynamicBody (the default) so they exchange deferred impulses.
                    AddCharacter(root, new Vector3(-0.3f, CharacterRadius + 0.05f, 0f));
                    AddCharacter(root, new Vector3(0.3f, CharacterRadius + 0.05f, 0f));
                }
            );
        }

        // Dynamic-body push (light box): a kinematic character (SimulateDynamicBody on) walks +X into a regular
        // dynamic box (gravity off so it stays at platform height). The box must move +X and the character must
        // not penetrate it.
        static void BuildDynamicPush()
        {
            BuildDynamicPushFixture(DynamicPushChild, DynamicPushParent, DynamicBoxLightMass);
        }

        // Dynamic-body push (heavy box): identical to BuildDynamicPush save the box's authored mass (50× heavier).
        // The adversarial pair: a heavier authored mass must be pushed LESS for the same character push.
        static void BuildDynamicPushHeavy()
        {
            BuildDynamicPushFixture(
                DynamicPushHeavyChild,
                DynamicPushHeavyParent,
                DynamicBoxHeavyMass
            );
        }

        static void BuildDynamicPushFixture(string childPath, string parentPath, float boxMass)
        {
            BuildFixture(
                childPath,
                parentPath,
                root =>
                {
                    AddFloor(root, new Vector2(0f, FloorTopY - 0.5f), new Vector2(40f, 1f));
                    // Dynamic box: left face at DynamicBoxLeftFaceX, 1×1 box (centre at leftFace + 0.5). Gravity
                    // off so it neither falls nor is held by the floor's friction differently per mass — a pure
                    // horizontal push. Explicit mass (UseAutoMass off) so StoredDynamicBodyData2D.Mass is exactly
                    // boxMass, the value the impulse exchange scales by.
                    AddDynamicBox(
                        root,
                        new Vector3(DynamicBoxLeftFaceX + 0.5f, CharacterRadius + 0.05f, 0f),
                        new Unity.Mathematics.float2(1f, 1f),
                        boxMass
                    );
                    var c = AddCharacter(root, new Vector3(0f, CharacterRadius + 0.05f, 0f));
                    // SimulateDynamicBody is on by default — the character pushes the box. Leave it.
                    _ = c;
                }
            );
        }

        // Moving platform: a kinematic platform body the runtime test drives laterally via MovePosition, with a
        // character standing on it. The character is auto-parented to the platform (TrackedTransform2D added at
        // runtime) and carried along in X.
        static void BuildMovingPlatform()
        {
            BuildFixture(
                MovingPlatformChild,
                MovingPlatformParent,
                root =>
                {
                    // The platform: a wide, thin KINEMATIC box whose top is at PlatformTopY. Kinematic so the
                    // runtime test can drive its pose with MovePosition each step (a static body cannot move; a
                    // dynamic body would fall). Centre placed so the top surface sits at PlatformTopY.
                    AddKinematicPlatform(
                        root,
                        new Vector3(0f, PlatformTopY - 0.5f, 0f),
                        new Unity.Mathematics.float2(8f, 1f)
                    );
                    // Character standing on the platform top.
                    AddCharacter(root, new Vector3(0f, PlatformTopY + CharacterRadius + 0.05f, 0f));
                }
            );
        }

        // ---- gate-4 future-slope / downward-ledge fixtures -------------------------------------------------

        // Downward ledge: a flat platform that ENDS in mid-air. A grounded character running +X off the edge, with
        // PreventGroundingWhenMovingTowardsNoGrounding on (the default), must UNGROUND as it crosses the edge
        // (DetectFutureSlopeChange finds no ground ahead → isMovingTowardsNoGrounding) and launch off cleanly,
        // rather than snapping back onto the ledge. This exercises the no-grounding branch of the future-slope path.
        static void BuildDownLedge()
        {
            BuildFixture(
                DownLedgeChild,
                DownLedgeParent,
                root =>
                {
                    // Platform top at Y=0, ending at X = LedgeEdgeX (right face). Beyond it is empty space.
                    AddFloor(
                        root,
                        new Vector2((LedgeEdgeX - 10f) * 0.5f, FloorTopY - 0.5f),
                        new Vector2(10f + LedgeEdgeX, 1f)
                    );
                    var c = AddCharacter(
                        root,
                        new Vector3(LedgeEdgeX - 3f, CharacterRadius + 0.05f, 0f)
                    );
                    // PreventGroundingWhenMovingTowardsNoGrounding is on by default in the params; leave it. No
                    // HasMaxDownwardSlopeChangeAngle needed — the ledge is the no-grounding case.
                    _ = c;
                }
            );
        }

        // Gentle downslope (within the max downward slope-change angle): a flat top that transitions to a shallow
        // downhill. A grounded character running +X onto it, with HasMaxDownwardSlopeChangeAngle on and a generous
        // max, must STAY grounded (the future-slope check finds a grounded slope ahead via the secondary-down ray,
        // and the angle is within the limit so it is not force-ungrounded). The downhill angle (GentleDownSlopeDeg)
        // is below MaxDownwardSlopeChangeForGate.
        static void BuildDownSlopeGentle()
        {
            BuildFixture(
                DownSlopeGentleChild,
                DownSlopeGentleParent,
                root => BuildDownSlopeScene(root, GentleDownSlopeDeg)
            );
        }

        // Steep downslope (over the max downward slope-change angle): the same flat-to-downhill transition but a
        // steep downhill (SteepDownSlopeDeg > MaxDownwardSlopeChangeForGate). A grounded character running +X onto
        // it must UNGROUND at the transition (the SIGNED future-slope angle is negative and its magnitude exceeds
        // the max), launching off the lip instead of snapping down the steep face. This is the direct behavioural
        // arbiter of the angle SIGN: a wrong (positive-for-downward) sign would never trip the < -max test and the
        // character would stay glued to the steep slope.
        static void BuildDownSlopeSteep()
        {
            BuildFixture(
                DownSlopeSteepChild,
                DownSlopeSteepParent,
                root => BuildDownSlopeScene(root, SteepDownSlopeDeg)
            );
        }

        // A flat top platform (so the character starts grounded on flat ground, currentGroundUp = +Y) butted up
        // against a downhill ramp descending to the right at `downSlopeDeg`. The character starts on the flat part
        // and walks +X toward the lip where the flat meets the downhill.
        static void BuildDownSlopeScene(GameObject root, float downSlopeDeg)
        {
            // Flat top: left face well to the left, right face (the lip) at X = SlopeLipX, top at Y=0.
            AddFloor(root, new Vector2(SlopeLipX - 5f, FloorTopY - 0.5f), new Vector2(10f, 1f));
            // Downhill ramp: a long thin box rotated -downSlopeDeg (descending to the right), its high (left) end
            // butted at the lip so its top surface continues down-right from (SlopeLipX, 0).
            var ramp = AddFloor(root, new Vector2(SlopeLipX + 5f, -0.5f), new Vector2(12f, 1f));
            ramp.transform.RotateAround(
                new Vector3(SlopeLipX, 0f, 0f),
                Vector3.forward,
                -downSlopeDeg
            );
            // Character on the flat top, a little left of the lip, close to the surface.
            AddCharacter(root, new Vector3(SlopeLipX - 2f, CharacterRadius + 0.05f, 0f));
        }

        // Sensor pass-through (the negative-space regression case): a character dropped from above a SENSOR box
        // that straddles its fall path, with a SOLID floor far below. A kinematic character must treat the sensor
        // as NON-SOLID — pass THROUGH it (firing a trigger Begin as it enters) and land on the solid floor below —
        // not ground on the sensor as if it were floor. The runtime gate asserts (a) the character never grounds on
        // the sensor entity and (b) a trigger Begin fires for the character entering the sensor, and (c) it settles
        // on the solid floor below.
        static void BuildSensorPassThrough()
        {
            BuildFixture(
                SensorPassThroughChild,
                SensorPassThroughParent,
                root =>
                {
                    // The sensor: a wide box centred at Y=0 (volume Y∈[-1,1]), CollisionResponse = Sensor so it
                    // bakes isTrigger=true. The character falls through its top face at Y=+1.
                    AddSensorBox(
                        root,
                        new Vector3(0f, SensorBoxCenterY, 0f),
                        new Unity.Mathematics.float2(8f, 2f * SensorBoxHalfHeight)
                    );
                    // The solid floor the character should ACTUALLY land on, well below the sensor.
                    AddFloor(
                        root,
                        new Vector2(0f, SensorSolidFloorTopY - 0.5f),
                        new Vector2(40f, 1f)
                    );
                    // Character dropped above the sensor so it falls through it onto the solid floor.
                    AddCharacter(root, new Vector3(0f, SensorBoxHalfHeight + 2f, 0f));
                }
            );
        }

        // ---- authoring helpers -----------------------------------------------------------------------------

        static GameObject AddFloor(GameObject parent, Vector2 center, Vector2 size)
        {
            var go = new GameObject("Static");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = new Vector3(center.x, center.y, 0f);
            var box = go.AddComponent<BoxCollider2D>();
            box.size = size;
            return go;
        }

        static CharacterController2DAuthoring AddCharacter(GameObject parent, Vector3 position)
        {
            var go = new GameObject("Character");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var auth = go.AddComponent<CharacterController2DAuthoring>();

            var props = AuthoringKinematicCharacterProperties2D.GetDefault();
            // The default mass-interaction with dynamic bodies is irrelevant to the C4a core gates and there are no
            // dynamic bodies in these fixtures; leave SimulateDynamicBody default. Use a circle proxy at the shared
            // radius so the grounded settle-height assertion is exact.
            auth.CharacterProperties = props;
            auth.ProxyShape = CharacterProxyShape2D.Circle;
            auth.ProxyRadius = CharacterRadius;
            // Render interpolation lags LocalToWorld by a render frame; turn it off so the test reads the raw
            // fixed-step pose directly off LocalToWorld without a smoothing offset confound.
            auth.InterpolateRendering = false;
            return auth;
        }

        // A box-proxy character: a flat-bottomed box (full extents 2*radius wide, 2*radius tall) so a step-up
        // down-cast lands cleanly on the step top with an up-pointing normal (a circle proxy can catch the step's
        // top-left CORNER with a diagonal normal that fails the bottom-of-character grounding gate). The step
        // gates use this so the step feature is tested without the round-base corner ambiguity.
        static CharacterController2DAuthoring AddBoxCharacter(GameObject parent, Vector3 position)
        {
            var go = new GameObject("Character");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var auth = go.AddComponent<CharacterController2DAuthoring>();

            var props = AuthoringKinematicCharacterProperties2D.GetDefault();
            auth.CharacterProperties = props;
            auth.ProxyShape = CharacterProxyShape2D.Box;
            auth.ProxyBoxSize = new Unity.Mathematics.float2(
                2f * CharacterRadius,
                2f * CharacterRadius
            );
            auth.InterpolateRendering = false;
            return auth;
        }

        // A capsule-proxy character: a vertical capsule (width CapsuleWidth → cap radius 0.5, height CapsuleHeight
        // → caps at ±0.5). The proxy cast is a CapsuleCast, the world body shape a capsule, both derived from the
        // same authored size — the P0 capsule mandate end-to-end.
        static CharacterController2DAuthoring AddCapsuleCharacter(
            GameObject parent,
            Vector3 position
        )
        {
            var go = new GameObject("Character");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var auth = go.AddComponent<CharacterController2DAuthoring>();

            var props = AuthoringKinematicCharacterProperties2D.GetDefault();
            auth.CharacterProperties = props;
            auth.ProxyShape = CharacterProxyShape2D.Capsule;
            auth.ProxyCapsuleSize = new Unity.Mathematics.float2(CapsuleWidth, CapsuleHeight);
            auth.ProxyCapsuleDirection = CharacterCapsuleDirection2D.Vertical;
            auth.InterpolateRendering = false;
            return auth;
        }

        // Step handling for a capsule: the grounding-width check uses the cap diameter (the capsule's width), the
        // capsule analogue of EnableStepHandling's 2*CharacterRadius for a circle.
        static void EnableCapsuleStepHandling(
            CharacterController2DAuthoring auth,
            float maxStepHeight
        )
        {
            var step = auth.StepAndSlopeHandling;
            step.StepHandling = true;
            step.MaxStepHeight = maxStepHeight;
            step.CharacterWidthForStepGroundingCheck = CapsuleWidth;
            auth.StepAndSlopeHandling = step;
        }

        static void DisableGrounding(CharacterController2DAuthoring auth)
        {
            var props = auth.CharacterProperties;
            props.EvaluateGrounding = false;
            props.SnapToGround = false;
            auth.CharacterProperties = props;
        }

        static void BumpOverlapIterations(CharacterController2DAuthoring auth, byte iterations)
        {
            var props = auth.CharacterProperties;
            props.MaxOverlapDecollisionIterations = iterations;
            auth.CharacterProperties = props;
        }

        // Step handling is OFF by default (BasicStepAndSlopeHandlingParameters2D.GetDefault). The step fixtures
        // turn it on and set the max step height. CharacterWidthForStepGroundingCheck is 2x the radius for a
        // circle proxy (the param's documented value).
        static void EnableStepHandling(CharacterController2DAuthoring auth, float maxStepHeight)
        {
            var step = auth.StepAndSlopeHandling;
            step.StepHandling = true;
            step.MaxStepHeight = maxStepHeight;
            step.CharacterWidthForStepGroundingCheck = 2f * CharacterRadius;
            auth.StepAndSlopeHandling = step;
        }

        // A regular DYNAMIC body the character can push: a PhysicsBody2DAuthoring (Dynamic, explicit mass, gravity
        // off) + a box PhysicsShape2DAuthoring. The explicit mass (UseAutoMass off) is the exact value the
        // StoredDynamicBodyData2D snapshot carries and the hit-dynamics impulse exchange scales by.
        static GameObject AddDynamicBox(
            GameObject parent,
            Vector3 position,
            Unity.Mathematics.float2 size,
            float mass
        )
        {
            var go = new GameObject("DynamicBox");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Dynamic;
            body.UseAutoMass = false;
            body.Mass = mass;
            body.GravityScale = 0f; // a pure horizontal push — no fall, no per-mass settling difference.
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = size;
            shape.Density = 1f;
            return go;
        }

        // A static SENSOR (isTrigger) box: a PhysicsBody2DAuthoring (Static) + a box PhysicsShape2DAuthoring with
        // CollisionResponse = Sensor (which bakes PhysicsShape2D.isTrigger = true). A sensor overlaps and reports
        // trigger events but produces no collision response — the controller must pass through it. Authored via the
        // package authoring (not a bare BoxCollider2D) because CollisionResponse lives on PhysicsShape2DAuthoring.
        static GameObject AddSensorBox(
            GameObject parent,
            Vector3 position,
            Unity.Mathematics.float2 size
        )
        {
            var go = new GameObject("SensorZone");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Static;
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = size;
            shape.CollisionResponse = PhysicsCollisionResponse2D.Sensor;
            return go;
        }

        // A KINEMATIC platform body the runtime test drives with MovePosition. Kinematic so its pose is moved
        // directly (a static body cannot move, a dynamic body would fall). The runtime test adds a
        // TrackedTransform2D to it so the controller treats it as a moving platform.
        static GameObject AddKinematicPlatform(
            GameObject parent,
            Vector3 position,
            Unity.Mathematics.float2 size
        )
        {
            var go = new GameObject("MovingPlatform");
            go.transform.SetParent(parent.transform, false);
            go.transform.position = position;
            var body = go.AddComponent<PhysicsBody2DAuthoring>();
            body.BodyType = PhysicsBody2DMotionType.Kinematic;
            var shape = go.AddComponent<PhysicsShape2DAuthoring>();
            shape.Kind = PhysicsShape2DKind.Box;
            shape.BoxSize = size;
            return go;
        }

        // ---- scene plumbing (mirrors the substrate builders) -----------------------------------------------

        static void BuildFixture(
            string childPath,
            string parentPath,
            System.Action<GameObject> populate
        )
        {
            var child = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("FixtureRoot");
            populate(root);
            EditorSceneManager.MarkSceneDirty(child);
            EditorSceneManager.SaveScene(child, childPath);

            var parent = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var subSceneGo = new GameObject(
                Path.GetFileNameWithoutExtension(childPath) + " SubScene"
            );
            var subScene = subSceneGo.AddComponent<SubScene>();
            subScene.SceneAsset = AssetDatabase.LoadAssetAtPath<SceneAsset>(childPath);
            subScene.AutoLoadScene = true;
            EditorSceneManager.MarkSceneDirty(parent);
            EditorSceneManager.SaveScene(parent, parentPath);

            RegisterSceneInBuildSettings(parentPath);
            RegisterSceneInBuildSettings(childPath);
        }

        static void RegisterSceneInBuildSettings(string scenePath)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (scenes.Exists(s => s.path == scenePath))
                return;
            scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }
    }
}
