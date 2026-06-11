using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Core;
using Unity.Entities;
using Unity.U2D.Physics;
using Zori.Entities.Physics2D;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The per-system shared context holding the global data a character update reads — the world handle, the
    /// component lookups, and a reusable hit list — passed by <c>ref</c> through every solve step and processor
    /// callback (the interface names this type, <see cref="IKinematicCharacterProcessor2D{C}"/>). 2D port of
    /// <c>Unity.CharacterController.KinematicCharacterUpdateContext</c>
    /// (REF/KinematicCharacterUpdateContext.cs:13). Two reductions against the 3D original:
    /// <list type="bullet">
    /// <item>The 3D <c>Unity.Physics.PhysicsWorld</c> becomes the substrate's <see cref="PhysicsWorld"/> handle,
    /// read from <c>SystemAPI.GetSingleton&lt;PhysicsWorldSingleton2D&gt;().world</c>
    /// (P2D/Runtime/Components/PhysicsWorldSingleton2D.cs). The substrate's queries read against a stepped world,
    /// so the system holding this context runs <c>[UpdateAfter(PhysicsWorld2DSystem)]</c>.</item>
    /// <item>The 3D context's three typed scratch lists (<c>TmpRaycastHits</c> / <c>TmpColliderCastHits</c> /
    /// <c>TmpDistanceHits</c>) plus the rigidbody-index dedup list collapse to ONE
    /// <see cref="NativeList{PhysicsQueryHit2D}"/>: the substrate exposes a single hit type
    /// (P2D/Runtime/Components/PhysicsQueryHit2D.cs) for raycast, circle/box cast, and overlap, so one reusable
    /// list serves every query. The dedup-by-rigidbody-index list is dropped — the 2D substrate identifies bodies
    /// by <see cref="Entity"/>, not a per-step index, so a consumer that needs per-body dedup keys off the
    /// entity.</item>
    /// </list>
    /// It is a plain mutable <c>struct</c> (passed by <c>ref</c> to the processor callbacks), not a
    /// <c>ref struct</c>: the interface takes it by <c>ref</c> and the field set is all blittable, so it is
    /// usable inside a Burst job — no <c>[BurstCompile]</c> on its own methods (the entry-point-only rule;
    /// docs/unity/burst/compilation-context.md). The lifecycle methods (<see cref="OnSystemCreate"/>,
    /// <see cref="OnSystemUpdate"/>, <see cref="EnsureCreationOfTmpCollections"/>) port verbatim from the 3D
    /// context's <c>:62/:74/:91</c>.
    /// </summary>
    public struct KinematicCharacterUpdateContext2D
    {
        /// <summary>
        /// Global time data, set from the solving system's <c>SystemAPI.Time</c> in <see cref="OnSystemUpdate"/>.
        /// </summary>
        public TimeData Time;

        /// <summary>
        /// The substrate physics world this character is part of, read from the
        /// <see cref="PhysicsWorldSingleton2D"/> singleton. Queried through <see cref="PhysicsQueries2D"/>; valid
        /// only against a stepped world, so the holder runs after <c>PhysicsWorld2DSystem</c>.
        /// </summary>
        [ReadOnly]
        public PhysicsWorld PhysicsWorld;

        /// <summary>
        /// Lookup for the <see cref="StoredKinematicCharacterData2D"/> component — the pre-solve snapshot another
        /// character's mass/velocity is read from during the character ↔ character impulse exchange.
        /// </summary>
        [ReadOnly]
        public ComponentLookup<StoredKinematicCharacterData2D> StoredCharacterBodyPropertiesLookup;

        /// <summary>
        /// Lookup for the <see cref="TrackedTransform2D"/> component — the current/previous fixed-rate poses of a
        /// moving-platform parent the character rides.
        /// </summary>
        [ReadOnly]
        public ComponentLookup<TrackedTransform2D> TrackedTransformLookup;

        /// <summary>
        /// Lookup for the <see cref="StoredDynamicBodyData2D"/> component — the main-thread pre-solve snapshot of a
        /// regular (non-character) dynamic body's velocity and mass (written by
        /// <see cref="StoreDynamicBodyDataSystem2D"/>). It is how the Burst character solve reads another body's
        /// motion for the hit-dynamics impulse exchange without touching the managed <c>PhysicsBody</c> handle (the
        /// D5 resolution — the velocity read is not Burst-callable, so it is snapshotted on the main thread and read
        /// here as a Burst-safe component).
        /// </summary>
        [ReadOnly]
        public ComponentLookup<StoredDynamicBodyData2D> DynamicBodyDataLookup;

        /// <summary>
        /// The one reusable hit list every query writes into during the update (raycast, circle/box cast, and
        /// overlap all return <see cref="PhysicsQueryHit2D"/>). Created lazily inside the job via
        /// <see cref="EnsureCreationOfTmpCollections"/>. <see cref="NativeDisableContainerSafetyRestrictionAttribute"/>
        /// matches the 3D context's scratch-list relaxation: it is per-job <c>Allocator.Temp</c> scratch reused
        /// across many queries in one <c>Execute</c>, with no cross-job sharing — the failure mode the relaxation
        /// would otherwise guard (main-thread / cross-job aliasing) cannot arise for a Temp list local to the job.
        /// </summary>
        [NativeDisableContainerSafetyRestriction]
        public NativeList<PhysicsQueryHit2D> TmpQueryHits;

        /// <summary>
        /// Gets and stores the component lookups at the moment of the holding system's creation. Call from the
        /// system's <c>OnCreate</c>. Ports REF/KinematicCharacterUpdateContext.cs:62.
        /// </summary>
        /// <param name="state"> The state of the system calling this method </param>
        public void OnSystemCreate(ref SystemState state)
        {
            StoredCharacterBodyPropertiesLookup = state.GetComponentLookup<StoredKinematicCharacterData2D>(true);
            TrackedTransformLookup = state.GetComponentLookup<TrackedTransform2D>(true);
            DynamicBodyDataLookup = state.GetComponentLookup<StoredDynamicBodyData2D>(true);
        }

        /// <summary>
        /// Refreshes the stored data for the holding system's update: copies the time, reads the world from the
        /// singleton, updates the lookups, and resets the scratch list to default (it is recreated lazily inside
        /// the job). Call from the system's <c>OnUpdate</c>. Ports REF/KinematicCharacterUpdateContext.cs:74,
        /// taking the substrate world handle in place of the 3D <c>PhysicsWorldSingleton</c>.
        /// </summary>
        /// <param name="state"> The state of the system calling this method </param>
        /// <param name="time"> The time data passed on by the system calling this method </param>
        /// <param name="physicsWorld"> The substrate world handle from <c>PhysicsWorldSingleton2D.world</c> </param>
        public void OnSystemUpdate(ref SystemState state, TimeData time, PhysicsWorld physicsWorld)
        {
            Time = time;
            PhysicsWorld = physicsWorld;

            StoredCharacterBodyPropertiesLookup.Update(ref state);
            TrackedTransformLookup.Update(ref state);
            DynamicBodyDataLookup.Update(ref state);

            TmpQueryHits = default;
        }

        /// <summary>
        /// Ensures the temporary hit list is created. Call inside the job, before the character update. Ports
        /// REF/KinematicCharacterUpdateContext.cs:91 (the four 3D lists reduce to this one).
        /// </summary>
        public void EnsureCreationOfTmpCollections()
        {
            if (!TmpQueryHits.IsCreated)
            {
                TmpQueryHits = new NativeList<PhysicsQueryHit2D>(24, Allocator.Temp);
            }
        }
    }
}
