using System;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// The mass properties of a body for the controller's 2D impulse exchange — the 2D analogue of
    /// <c>Unity.Physics.PhysicsMass</c> as used by the reference's dynamics path
    /// (REF/PhysicsUtilities.cs:147,228). In 2D the angular degree of freedom is a single scalar, so the 3D
    /// <c>float3 InverseInertia</c> collapses to a <see cref="InverseInertia"/> <c>float</c> and the center of
    /// mass is a <c>float2</c>. This is a plain data struct (not an <see cref="Unity.Entities.IComponentData"/>):
    /// it is constructed at solve time from a character's stored data or from a regular body's properties, and
    /// passed by ref to the processor's <see cref="IKinematicCharacterProcessor2D{C}.OverrideDynamicHitMasses"/>
    /// callback so user logic can tune the masses before the impulse solve. It lives in the data layer because the
    /// processor interface names it; the impulse solver in <see cref="PhysicsUtilities2D"/> consumes it.
    /// </summary>
    [Serializable]
    public struct KinematicCharacterMass2D
    {
        /// <summary>
        /// The body's center of mass, in the body's local space (zero for a uniform circle/box about its origin).
        /// </summary>
        public float2 CenterOfMass;

        /// <summary>
        /// Inverse of the body's mass. Zero means infinite mass (a kinematic or static body that does not respond
        /// to impulses).
        /// </summary>
        public float InverseMass;

        /// <summary>
        /// Inverse of the body's rotational inertia about its center of mass (scalar in 2D). Zero means the body
        /// does not respond to angular impulse.
        /// </summary>
        public float InverseInertia;
    }
}
