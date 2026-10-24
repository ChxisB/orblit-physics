namespace Orblit.Physics.Scene;

/// <summary>
/// What a character asks for, once a step, before the step is taken.
/// </summary>
/// <remarks>
/// <see cref="ScenePhysics.Advance"/> is the only caller. A frame that owes three steps asks three
/// times, so a walk that is driven by a clip's playhead follows the world's own clock rather than the
/// frame's.
/// </remarks>
public interface ICharacterWalk
{
    /// <summary>
    /// The velocity the character would like for the next <paramref name="seconds"/>, in the world's axes
    /// and relative to whatever it stands on, with gravity in it.
    /// </summary>
    /// <remarks>
    /// <paramref name="footing"/> is what the last step made of the last request, or null before the
    /// character's first step. A character that is standing reads back no fall, so asking for gravity on
    /// top of that is how it keeps standing.
    /// </remarks>
    IReadOnlyList<double> Ask(double seconds, PhysicsFooting? footing);
}
