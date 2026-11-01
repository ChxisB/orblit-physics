using Orblit.Motion;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

/// <summary>
/// A character that goes where its clip's root goes.
/// </summary>
/// <remarks>
/// A <see cref="ClipPlayer"/> plays the clip, and each step its root motion is turned by the way the
/// character faces and asked for as a velocity. The world still decides how much of it the character
/// gets, so a walk into a wall stops at the wall and a walk up a step climbs it, whatever the clip says.
/// Which way the character faces is kept here as <see cref="Heading"/>, turned by the clip's own turns,
/// because a standing capsule is the same shape whichever way it looks and the world keeps no facing for
/// it. The game writes it to the entity.
/// </remarks>
public sealed class RootMotionWalk(
    ClipPlayer player,
    Quaternion? heading = null,
    IReadOnlyList<double>? gravity = null,
    double scale = 1.0) : ICharacterWalk
{
    /// <summary>
    /// Plays the clip. Started and stopped by the game, which is how a walk becomes an idle without the
    /// world hearing of it.
    /// </summary>
    public ClipPlayer Player { get; } = player;

    /// <summary>Which way the character faces in the world.</summary>
    public Quaternion Heading { get; set; } = heading ?? Quaternion.Identity();

    /// <summary>
    /// Metres per second squared, added to the fall while the character is not standing. The world's own
    /// is not asked, so a game that changes it tells the walk too.
    /// </summary>
    public IReadOnlyList<double> Gravity { get; } = gravity ?? [0.0, -9.81, 0.0];

    /// <summary>
    /// How much the clip's distances are multiplied by, for a model scaled away from the size its clip
    /// was made at.
    /// </summary>
    public double Scale { get; } = scale;

    public IReadOnlyList<double> Ask(double seconds, PhysicsFooting? footing)
    {
        var moved = Player.Advance(seconds).Moved;

        // Through the matrix: turning by the quaternion itself would turn by the inverse.
        var along = Heading.AsRotationMatrix().Transformed(moved.Position);
        along.Scale(Scale / seconds);
        var turned = Heading * moved.Rotation;
        turned.Normalize();
        Heading = turned;

        var fall = footing?.Velocity ?? [0.0, 0.0, 0.0];
        return [along.X, along.Y + fall[1] + (Gravity[1] * seconds), along.Z];
    }
}
