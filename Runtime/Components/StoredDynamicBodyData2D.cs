using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// A pre-solve snapshot of a regular (non-character) dynamic body's velocity and mass, so the Burst character
    /// solve can read another body's motion for the hit-dynamics impulse exchange WITHOUT touching the managed
    /// <c>Unity.U2D.Physics.PhysicsBody</c> handle (whose <c>linearVelocity</c>/<c>angularVelocity</c> are managed
    /// property reads — not HPC#, not Burst-callable). The 2D substrate
    /// surfaces no Burst-safe body-velocity read and no body-mass read at all, so the read is performed once per
    /// step on the MAIN THREAD by <see cref="StoreDynamicBodyDataSystem2D"/> and deposited here, where the Burst
    /// solve reads it through a <see cref="ComponentLookup{T}"/> — the same store-then-read pattern
    /// <see cref="StoreKinematicCharacterBodyPropertiesSystem2D"/> already uses for character ↔ character exchange
    /// (<see cref="StoredKinematicCharacterData2D"/>), extended to regular bodies.
    ///
    /// <para>This is the resolution of the substrate gap: rather than run the hit-dynamics phase on the main
    /// thread (which would serialize the whole parallel solve), a small main-thread pre-pass snapshots only the
    /// dynamic-body data the Burst solve reads, leaving the solve <c>ScheduleParallel</c> and HPC#-clean.</para>
    ///
    /// <para><b>Mass.</b> The substrate exposes no read for a regular body's solved mass/inertia (it only ever
    /// WRITES mass at creation via <c>PhysicsBody.MassConfiguration</c>). The authored mass is sourced from the
    /// body's <see cref="Zori.Entities.Physics2D.PhysicsBody2DDefinition"/> (Burst-OK, a plain component read), so
    /// the snapshot carries the authored mass; a density-derived (un-authored) mass is approximated by the
    /// definition's <c>mass</c> field. The character's processor can still override both masses via
    /// <see cref="IKinematicCharacterProcessor2D{C}.OverrideDynamicHitMasses"/> at solve time.</para>
    /// </summary>
    [Serializable]
    public struct StoredDynamicBodyData2D : IComponentData
    {
        /// <summary>Whether the body responds to impulses (a dynamic body with positive mass).</summary>
        public bool IsDynamic;

        /// <summary>The body's linear velocity (m/s), snapshotted on the main thread before the solve.</summary>
        public float2 LinearVelocity;

        /// <summary>The body's angular velocity (rad/s about +z), snapshotted on the main thread before the solve.</summary>
        public float AngularVelocity;

        /// <summary>The body's mass (kg), sourced from its authored <c>PhysicsBody2DDefinition</c>.</summary>
        public float Mass;

        /// <summary>The body's rotational inertia (kg·m², scalar in 2D); 0 when un-authored.</summary>
        public float RotationalInertia;
    }
}
