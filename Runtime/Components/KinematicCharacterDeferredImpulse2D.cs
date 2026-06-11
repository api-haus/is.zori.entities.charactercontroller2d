using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// An impulse to apply to another body later in the frame, recorded during the solve and drained by the
    /// deferred-impulses system. 2D port of <c>Unity.CharacterController.KinematicCharacterDeferredImpulse</c>
    /// (REF/KinematicCharacterComponents.cs:400): <see cref="LinearVelocityChange"/> and
    /// <see cref="Displacement"/> reduce from <c>float3</c> to <c>float2</c>;
    /// <see cref="AngularVelocityChange"/> reduces from a <c>float3</c> to a single <c>float</c> (the 2D angular
    /// DOF is a scalar z-rate). <c>[InternalBufferCapacity(0)]</c> is kept from the reference: a character's
    /// deferred-impulse count varies per step, so the buffer heap-allocates rather than bloating the chunk.
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(0)]
    public struct KinematicCharacterDeferredImpulse2D : IBufferElementData
    {
        /// <summary>
        /// Entity on which to apply the impulse
        /// </summary>
        public Entity OnEntity;

        /// <summary>
        /// The impulse's change in linear velocity
        /// </summary>
        public float2 LinearVelocityChange;

        /// <summary>
        /// The impulse's change in angular velocity (scalar z-rate, radians/second)
        /// </summary>
        public float AngularVelocityChange;

        /// <summary>
        /// The impulse's change in position
        /// </summary>
        public float2 Displacement;
    }
}
