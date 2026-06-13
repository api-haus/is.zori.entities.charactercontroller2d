using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Zori.Entities.Physics2D;
using static Unity.Mathematics.math;
using PhysicsShape2D = Zori.Entities.Physics2D.PhysicsShape2D;

namespace Zori.Entities.CharacterController2D.Tests
{
    /// <summary>
    /// The sensor (trigger) pass-through regression gate: the negative-space case the core and advanced-feature solve
    /// fixtures never covered. A kinematic character controller must treat a SENSOR (<c>isTrigger</c>) shape as NON-SOLID to its
    /// collision/grounding sweeps — the character passes THROUGH a sensor volume (a zone, a teleporter pad) as a
    /// visitor rather than grounding on it as if it were floor — WHILE the separate trigger-EVENT channel still
    /// reports the character entering the sensor. The Platformer smoke gate surfaced the bug: a character dropped
    /// onto a wind/teleporter sensor GROUNDED on it, so its interior was never entered and the trigger Begin never
    /// fired, killing every sensor-based feature.
    /// </summary>
    /// <remarks>
    /// The fixture (<c>CharacterFixtureBuilder.BuildSensorPassThrough</c>) drops a character through a SENSOR box
    /// straddling its fall path, with a SOLID floor far below. No mocks — the real default solve system
    /// (<c>KinematicCharacterPhysicsSolveSystem2D</c>) runs over a real Box2D world stepped by the
    /// FixedStepSimulationSystemGroup (the substrate's FallingBodyValidation pattern), and the trigger events come
    /// from the substrate's own <c>PhysicsWorld2DSystem</c> event collection. The three assertions are built from the
    /// solve's observable decision points: the character (1) never grounds on the sensor entity, (2) generates a
    /// trigger Begin for entering the sensor, and (3) passes through and settles on the solid floor below. The
    /// trigger-event assertion is the load-bearing one — it falsifies the bug directly, because the bug grounds the
    /// character on the sensor's TOP face and the interior is never entered, so no Begin fires.
    /// </remarks>
    public sealed class CharacterSensorGate
    {
        const int LoadTimeoutFrames = 600;
        const float FixedDt = 1f / 60f;
        const float CharacterRadius = 0.5f;

        World _world;
        FixedStepSimulationSystemGroup _fixedGroup;
        Unity.Entities.IRateManager _savedRateManager;
        EntityQuery _characterQuery;

        [TearDown]
        public void TearDown()
        {
            if (_fixedGroup != null && _savedRateManager != null)
                _fixedGroup.RateManager = _savedRateManager;
            _fixedGroup = null;
            _savedRateManager = null;
        }

        IEnumerator LoadAndPrepare(string sceneName)
        {
            SceneManager.LoadScene(sceneName, LoadSceneMode.Single);
            yield return null;

            _world = World.DefaultGameObjectInjectionWorld;
            Assert.IsNotNull(_world, "No default ECS world — the entities bootstrap did not run.");

            var em = _world.EntityManager;

            var bakedQuery = em.CreateEntityQuery(
                ComponentType.ReadOnly<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );
            var frames = 0;
            while (bakedQuery.CalculateEntityCount() == 0 && frames < LoadTimeoutFrames)
            {
                frames++;
                yield return null;
            }
            Assert.Greater(
                bakedQuery.CalculateEntityCount(),
                0,
                $"No baked character appeared after {frames} frames — build the fixtures via "
                    + "CharacterFixtureBuilder.BuildAll first."
            );

            // Opt the character into the default solve system by adding the gating tag at runtime (the real API,
            // no mock — the baker deliberately does not author it).
            using (var ents = bakedQuery.ToEntityArray(Allocator.Temp))
            {
                foreach (var e in ents)
                {
                    if (!em.HasComponent<DefaultCharacterController2DTag>(e))
                        em.AddComponent<DefaultCharacterController2DTag>(e);
                }
            }

            _characterQuery = em.CreateEntityQuery(
                ComponentType.ReadWrite<KinematicCharacterBody2D>(),
                ComponentType.ReadOnly<LocalToWorld>()
            );

            _fixedGroup = _world.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            Assert.IsNotNull(_fixedGroup, "No FixedStepSimulationSystemGroup in the default world.");
            _savedRateManager = _fixedGroup.RateManager;
            _fixedGroup.RateManager = new Unity.Entities.RateUtils.FixedRateSimpleManager(FixedDt);

            // First update: PhysicsWorld2DSystem creates the Box2D bodies (does not step on the creation frame).
            _fixedGroup.Update();
        }

        Entity TheCharacter()
        {
            using var ents = _characterQuery.ToEntityArray(Allocator.Temp);
            Assert.AreEqual(1, ents.Length, "the sensor fixture carries exactly one character");
            return ents[0];
        }

        float2 Position()
        {
            using var ltw = _characterQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);
            return ltw[0].Position.xy;
        }

        KinematicCharacterBody2D Body() =>
            _world.EntityManager.GetComponentData<KinematicCharacterBody2D>(TheCharacter());

        // The single baked sensor entity (the SensorZone authored CollisionResponse = Sensor → isTrigger = true).
        Entity TheSensor()
        {
            var em = _world.EntityManager;
            using var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsShape2D>());
            using var ents = q.ToEntityArray(Allocator.Temp);
            var found = Entity.Null;
            var count = 0;
            foreach (var e in ents)
            {
                if (em.GetComponentData<PhysicsShape2D>(e).isTrigger)
                {
                    found = e;
                    count++;
                }
            }
            Assert.AreEqual(1, count, "the sensor fixture carries exactly one isTrigger shape");
            return found;
        }

        Entity SingletonEntity()
        {
            var em = _world.EntityManager;
            using var q = em.CreateEntityQuery(ComponentType.ReadOnly<PhysicsWorldSingleton2D>());
            return q.GetSingletonEntity();
        }

        // =====================================================================================================
        // The regression gate: drive the character down through the sensor onto the solid floor, recording every
        // step whether it grounded on the sensor and whether a trigger Begin fired for it entering the sensor.
        // =====================================================================================================
        [UnityTest]
        public IEnumerator Sensor_CharacterFallsThrough_NeverGroundsOnSensor_TriggerBeginFires_LandsOnSolidFloor()
        {
            yield return LoadAndPrepare("CC2D_SensorPassThrough");

            var em = _world.EntityManager;
            var character = TheCharacter();
            var sensor = TheSensor();
            var singleton = SingletonEntity();

            var sawTriggerBeginForCharacter = false;
            var groundedOnSensorEver = false;
            var minY = float.MaxValue;

            // Drive enough steps for the character to fall through the sensor and settle on the solid floor below.
            // Each step: tick the fixed group, then read the just-stepped trigger buffer + the character's grounding.
            for (var s = 0; s < 240; s++)
            {
                _fixedGroup.Update();

                // (1) Trigger-event channel: a Begin where the character is the visitor entering the sensor.
                var trig = em.GetBuffer<PhysicsTriggerEvent2D>(singleton, isReadOnly: true);
                for (var i = 0; i < trig.Length; i++)
                {
                    var e = trig[i];
                    if (
                        e.phase == PhysicsEventPhase2D.Begin
                        && e.triggerEntity == sensor
                        && e.visitorEntity == character
                    )
                    {
                        sawTriggerBeginForCharacter = true;
                    }
                }

                // (2) Grounding: the character must NEVER report grounded ON the sensor entity. (It may ground on
                // the solid floor at the end — that is the sensor entity being EXCLUDED working correctly.)
                var body = Body();
                if (body.IsGrounded && body.GroundHit.Entity == sensor)
                    groundedOnSensorEver = true;

                minY = min(minY, Position().y);
            }

            var finalPos = Position();
            var finalBody = Body();

            Debug.Log(
                $"[CC2D-SENSOR] sawTriggerBegin={sawTriggerBeginForCharacter} groundedOnSensorEver={groundedOnSensorEver} "
                    + $"minY={minY} finalY={finalPos.y} finalGrounded={finalBody.IsGrounded} "
                    + $"finalGroundEntity={finalBody.GroundHit.Entity} sensor={sensor} character={character}."
            );

            // The bug: the controller's grounding/collide-and-slide casts treated the sensor as solid floor, so the
            // character grounded on the sensor's top face and never entered its interior — no trigger Begin fired.
            // The fix excludes isTrigger shapes from those casts, so the character passes through (Begin fires) and
            // lands on the solid floor below.
            Assert.IsFalse(
                groundedOnSensorEver,
                "character grounded ON the sensor entity — the controller's casts did not exclude the isTrigger shape "
                    + "(it treated the sensor as solid floor)."
            );

            Assert.IsTrue(
                sawTriggerBeginForCharacter,
                "no trigger Begin fired for the character entering the sensor — the character did not pass into the "
                    + "sensor interior (it grounded on or above the sensor instead of passing through)."
            );

            // Passed through: the character descended below the sensor's bottom face (Y < sensor bottom), proving it
            // did not rest on or inside the sensor.
            Assert.Less(
                minY,
                CharacterFixtureBuilderConstants.SensorBoxCenterY
                    - CharacterFixtureBuilderConstants.SensorBoxHalfHeight,
                $"character never descended below the sensor's bottom face — it did not pass through; minY {minY}."
            );

            // Landed on the solid floor below the sensor: grounded, settled ~radius above the solid floor top, on
            // the solid floor entity (not the sensor).
            Assert.IsTrue(finalBody.IsGrounded, "character must end grounded on the solid floor below the sensor.");
            Assert.AreNotEqual(
                sensor,
                finalBody.GroundHit.Entity,
                "the final ground hit must be the solid floor, never the sensor."
            );
            var expectedFloorY =
                CharacterFixtureBuilderConstants.SensorSolidFloorTopY
                + CharacterRadius
                + KinematicCharacterUtilities2D.Constants.CollisionOffset;
            Assert.AreEqual(
                expectedFloorY,
                finalPos.y,
                0.1f,
                $"character must settle ~radius above the solid floor top; got {finalPos.y}, expected ~{expectedFloorY}."
            );
            Assert.IsFalse(float.IsNaN(finalPos.x) || float.IsNaN(finalPos.y), "no NaN in the settled pose.");
        }
    }
}
