using Unity.Entities;
using UnityEngine.InputSystem;

namespace Zori.Entities.CharacterController2D.Samples
{
    /// <summary>
    /// Reads the keyboard each regular update and writes one frame of intent into every side-scroller character's
    /// <see cref="CharacterControl2D"/> — the control half of the design §8 control→physics split. Left/right (A/D or
    /// the arrows) set <see cref="CharacterControl2D.MoveX"/>; Space/W LATCHES <see cref="CharacterControl2D.JumpPressed"/>
    /// (the physics system clears it once it consumes the jump), so a jump pressed between two fixed steps is not lost.
    ///
    /// <para><b>Why a managed <see cref="SystemBase"/> in the regular group, not a Bursted <c>ISystem</c> in the fixed
    /// group.</b> <c>Keyboard.current</c> is a managed main-thread read (the project's Input System idiom, see
    /// <c>AppExitHandler</c>), so this system cannot Burst and must run on the main thread. It runs in the default
    /// <see cref="SimulationSystemGroup"/> (the regular per-frame update) — input is sampled at the render rate and the
    /// fixed-step physics system consumes the latest sample, the standard input→simulation hand-off. It is gated on
    /// the same <see cref="SideScrollerSampleConfig"/> singleton as the physics system, so importing the package runs
    /// nothing (design §8 — inert on import).</para>
    /// </summary>
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class SideScrollerCharacterControlSystem : SystemBase
    {
        protected override void OnCreate()
        {
            // Inert on import: run only when a scene opts in with the config singleton (mirrors the substrate
            // BatchSpawnSampleSystem's RequireForUpdate<BatchSpawnSampleConfig>).
            RequireForUpdate<SideScrollerSampleConfig>();
            RequireForUpdate<CharacterControl2D>();
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

            // Jump on the rising edge of Space or W (wasPressedThisFrame), latched into the control component for the
            // fixed-step physics system to consume — so a jump tapped between two fixed steps is not missed.
            bool jumpEdge = keyboard.spaceKey.wasPressedThisFrame || keyboard.wKey.wasPressedThisFrame;

            foreach (var control in SystemAPI.Query<RefRW<CharacterControl2D>>().WithAll<SideScrollerCharacterTag>())
            {
                control.ValueRW.MoveX = moveX;
                // Latch (OR), never overwrite to false here: the physics system clears the latch after it applies the
                // jump, so a jump edge sampled this frame survives until a fixed step consumes it.
                if (jumpEdge)
                {
                    control.ValueRW.JumpPressed = true;
                }
            }
        }
    }
}
