using Unity.Entities;
using UnityEngine.InputSystem;

namespace Zori.Entities.CharacterController2D.Samples.Platformer
{
    /// <summary>
    /// Reads the keyboard each regular update and writes one frame of intent into every Platformer character's
    /// <see cref="PlatformerCharacterControl2D"/> — the control half of the design §8 control→physics split. Left/right
    /// (A/D or the arrows) set <see cref="PlatformerCharacterControl2D.MoveX"/>; Space/W LATCHES
    /// <see cref="PlatformerCharacterControl2D.JumpPressed"/>, E LATCHES
    /// <see cref="PlatformerCharacterControl2D.GrabPressed"/>, and Q (or Left-Shift) LATCHES
    /// <see cref="PlatformerCharacterControl2D.ReleasePressed"/>. Each latch is set on the input rising edge and cleared
    /// by the consuming fixed-step physics / transition logic, so an edge tapped between two fixed steps is neither
    /// dropped nor double-applied across the differing input vs fixed-step rates.
    ///
    /// <para><b>Why a managed <see cref="SystemBase"/> in the regular group, not a Bursted <c>ISystem</c> in the fixed
    /// group.</b> <c>Keyboard.current</c> is a managed main-thread read (the project's Input System idiom), so this
    /// system cannot Burst and must run on the main thread. It runs in the default
    /// <see cref="SimulationSystemGroup"/> (the regular per-frame update) — input is sampled at the render rate and the
    /// fixed-step physics system consumes the latest sample, the standard input→simulation hand-off. It is gated on the
    /// sample's own <see cref="PlatformerCharacterTag"/> marker (like the physics system), so importing the package —
    /// which bakes no such marker — runs nothing (design §8 — inert on import; gate on the real marker, not an empty
    /// configures-nothing singleton).</para>
    /// </summary>
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class PlatformerCharacterControlSystem : SystemBase
    {
        protected override void OnCreate()
        {
            // Inert on import: run only when a scene bakes a Platformer character (its own marker). Importing the
            // package bakes no PlatformerCharacterTag, so this runs nothing — the inert-on-import guarantee gated on the
            // real marker.
            RequireForUpdate<PlatformerCharacterTag>();
            RequireForUpdate<PlatformerCharacterControl2D>();
        }

        protected override void OnUpdate()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return;
            }

            float moveX = 0f;
            if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed)
            {
                moveX -= 1f;
            }

            if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed)
            {
                moveX += 1f;
            }

            // Rising-edge latches (wasPressedThisFrame), each OR-latched into the control component for the fixed-step
            // logic to consume — so an edge tapped between two fixed steps is not missed.
            bool jumpEdge = keyboard.spaceKey.wasPressedThisFrame || keyboard.wKey.wasPressedThisFrame;
            bool grabEdge = keyboard.eKey.wasPressedThisFrame;
            bool releaseEdge = keyboard.qKey.wasPressedThisFrame || keyboard.leftShiftKey.wasPressedThisFrame;

            foreach (var control in SystemAPI.Query<RefRW<PlatformerCharacterControl2D>>().WithAll<PlatformerCharacterTag>())
            {
                control.ValueRW.MoveX = moveX;
                // Latch (OR), never overwrite to false here: the consuming system clears each latch after it acts on the
                // edge, so an edge sampled this frame survives until a fixed step consumes it.
                if (jumpEdge)
                {
                    control.ValueRW.JumpPressed = true;
                }

                if (grabEdge)
                {
                    control.ValueRW.GrabPressed = true;
                }

                if (releaseEdge)
                {
                    control.ValueRW.ReleasePressed = true;
                }
            }
        }
    }
}
