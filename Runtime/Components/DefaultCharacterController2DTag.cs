using Unity.Entities;

namespace Zori.Entities.CharacterController2D
{
    /// <summary>
    /// Marks a character entity as driven by the package's built-in
    /// <c>KinematicCharacterPhysicsSolveSystem2D</c> using the <see cref="DefaultKinematicCharacterProcessor2D"/>.
    /// This resolves design open-question D4 the SAFE way: the default solve system runs ONLY for entities carrying
    /// this tag, so a consumer who writes a custom processor and drives the solve steps from their own system simply
    /// omits the tag and is never double-stepped by the package's default system.
    ///
    /// <para>NEW component (no 3D origin): the 3D package ships no default solve system at all — the Standard
    /// Characters sample owns the solve loop. The 2D package ships a working default (D4), and a concrete tag is the
    /// negative-space guard that keeps that default from silently running on a custom-driven character. Add it via
    /// the authoring component (a "use default controller" toggle) when a consumer wants the out-of-box behaviour.</para>
    /// </summary>
    public struct DefaultCharacterController2DTag : IComponentData { }
}
