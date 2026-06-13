using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Zori.Entities.Physics2D;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Snapshots each character's mass and velocity into <see cref="StoredKinematicCharacterData2D"/> before the
    /// solve runs, so the character ↔ character impulse exchange reads a consistent pre-step snapshot of every
    /// other character and the parallel update stays deterministic. 2D port of
    /// <c>Unity.CharacterController.StoreKinematicCharacterBodyPropertiesSystem</c>
    /// (REF/StoreKinematicCharacterBodyPropertiesSystem.cs), which sits <c>OrderFirst</c> in the 3D character
    /// physics group.
    /// </summary>
    /// <remarks>
    /// In 2D there is no <c>AfterPhysicsSystemGroup</c>; the substrate's pipeline runs in
    /// <see cref="Physics2DSimulationSystemGroup"/>, against which the controller orders explicitly
    /// (design section 5). This system runs <c>[UpdateAfter(Physics2DSimulationSystemGroup)]</c>
    /// — it reads the just-stepped poses and writes the snapshot the solve consumes — and the solve system
    /// (<c>KinematicCharacterPhysicsSolveSystem2D</c>, chunk C4) declares
    /// <c>[UpdateAfter(StoreKinematicCharacterBodyPropertiesSystem2D)]</c> to run after it. C3 owns no forward
    /// reference to the C4 solve type; C4 references this type, so the snapshot-before-solve order is C4's to
    /// declare.
    /// </remarks>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(Physics2DSimulationSystemGroup))]
    [BurstCompile]
    public partial struct StoreKinematicCharacterBodyPropertiesSystem2D : ISystem
    {
        EntityQuery _storedCharacterQuery;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            _storedCharacterQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<StoredKinematicCharacterData2D, KinematicCharacterProperties2D, KinematicCharacterBody2D>()
                .Build(ref state);

            state.RequireForUpdate(_storedCharacterQuery);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state) { }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            StoreKinematicCharacterBodyPropertiesJob job = new StoreKinematicCharacterBodyPropertiesJob();
            job.ScheduleParallel();
        }

        /// <summary>
        /// Copies each character's properties + body into its <see cref="StoredKinematicCharacterData2D"/> to
        /// capture a snapshot before the solve modifies them, enabling a deterministic parallel character update.
        /// </summary>
        [BurstCompile]
        [WithAll(typeof(Simulate))]
        public partial struct StoreKinematicCharacterBodyPropertiesJob : IJobEntity
        {
            void Execute(
                ref StoredKinematicCharacterData2D storedData,
                in KinematicCharacterProperties2D characterProperties,
                in KinematicCharacterBody2D characterBody)
            {
                storedData.SetFrom(in characterProperties, in characterBody);
            }
        }
    }
}
