using System;
using System.Runtime.CompilerServices;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Mathematics.math;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Tracks a body's current and previous poses at the fixed timestep, so a character riding it as a moving
    /// platform can carry its own pose rigidly along with the platform's pose delta over the step. 2D port of
    /// <c>Unity.CharacterController.TrackedTransform</c> (REF/TrackedTransform.cs). The 3D component stores two
    /// <c>RigidTransform</c> (a <c>quaternion</c> + a <c>float3</c>); the 2D port stores two
    /// <see cref="RigidTransform2D"/> — a <c>float2</c> position and a <c>float</c> z-angle in radians (the
    /// package angular convention). <see cref="CalculatePointDisplacement"/> /
    /// <see cref="CalculatePointVelocity"/> port verbatim against the 2D rigid-transform inverse/transform.
    /// </summary>
    [Serializable]
    public struct TrackedTransform2D : IComponentData
    {
        /// <summary>
        /// Current transform, captured this fixed step.
        /// </summary>
        [HideInInspector]
        public RigidTransform2D CurrentFixedRateTransform;

        /// <summary>
        /// Previous transform, the value <see cref="CurrentFixedRateTransform"/> held last fixed step.
        /// </summary>
        [HideInInspector]
        public RigidTransform2D PreviousFixedRateTransform;

        /// <summary>
        /// Calculates the displacement that results from moving a point rigidly from the previous transform to
        /// the current transform — the point's local offset under the previous pose, re-expressed under the
        /// current pose, minus the original point. Ports REF/TrackedTransform.cs:32.
        /// </summary>
        /// <param name="point"> The world-space point to move </param>
        /// <returns> The world-space displacement of the point </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float2 CalculatePointDisplacement(float2 point)
        {
            float2 pointLocalToPreviousParent = PreviousFixedRateTransform.InverseTransformPoint(point);
            float2 pointTargetTranslation = CurrentFixedRateTransform.TransformPoint(pointLocalToPreviousParent);
            return pointTargetTranslation - point;
        }

        /// <summary>
        /// Calculates the linear velocity of a point moved from the previous transform to the current transform
        /// over a time delta. Ports REF/TrackedTransform.cs:46.
        /// </summary>
        /// <param name="point"> The world-space point to move </param>
        /// <param name="deltaTime"> The time delta over which the move happened </param>
        /// <returns> The world-space linear velocity of the point </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float2 CalculatePointVelocity(float2 point, float deltaTime)
        {
            return CalculatePointDisplacement(point) / deltaTime;
        }
    }

    /// <summary>
    /// A 2D rigid transform — a <c>float2</c> position and a <c>float</c> z-rotation in radians — the 2D analogue
    /// of <c>Unity.Mathematics.RigidTransform</c> used by <see cref="TrackedTransform2D"/>. Carries just the
    /// point transform and its inverse the parent-movement math needs; rotation is stored as a scalar angle (the
    /// package angular convention) rather than a quaternion.
    /// </summary>
    [Serializable]
    public struct RigidTransform2D
    {
        /// <summary>The world-space position.</summary>
        public float2 Position;

        /// <summary>The world-space z-rotation in radians.</summary>
        public float Rotation;

        /// <summary>The identity rigid transform (zero position, zero angle).</summary>
        public static RigidTransform2D Identity => new RigidTransform2D { Position = Unity.Mathematics.float2.zero, Rotation = 0f };

        /// <summary>
        /// Transforms a point from this transform's local space into world space: rotate by
        /// <see cref="Rotation"/>, then translate by <see cref="Position"/>.
        /// </summary>
        /// <param name="localPoint"> The point in this transform's local space </param>
        /// <returns> The point in world space </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float2 TransformPoint(float2 localPoint)
        {
            sincos(Rotation, out float s, out float c);
            float2 rotated = new float2(localPoint.x * c - localPoint.y * s, localPoint.x * s + localPoint.y * c);
            return Position + rotated;
        }

        /// <summary>
        /// Transforms a point from world space into this transform's local space: subtract
        /// <see cref="Position"/>, then rotate by <c>-<see cref="Rotation"/></c>.
        /// </summary>
        /// <param name="worldPoint"> The point in world space </param>
        /// <returns> The point in this transform's local space </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float2 InverseTransformPoint(float2 worldPoint)
        {
            sincos(Rotation, out float s, out float c);
            float2 d = worldPoint - Position;
            // Inverse rotation: transpose of the rotation matrix.
            return new float2(d.x * c + d.y * s, -d.x * s + d.y * c);
        }
    }
}
