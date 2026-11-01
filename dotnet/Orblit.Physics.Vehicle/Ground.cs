using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// Where the wheel stands on the ground: its two directions along it, and how
/// fast the chassis moves over it that way.
/// </summary>
// F2: five fields, all required.
internal sealed record Ground(Vector3 Heading, Vector3 Side, double Along, double Across, double Speed)
{
    /// <summary>
    /// Null when the wheel points straight into the ground and has no way along
    /// it.
    /// </summary>
    public static Ground? Of(Patch patch)
    {
        var normal = patch.Normal;
        var heading = patch.Heading - (normal * patch.Heading.Dot(normal));
        if (heading.Length2 < 1e-6)
        {
            return null;
        }

        heading.Normalize();
        var side = normal.Cross(heading);
        var moving = patch.Velocity - (normal * patch.Velocity.Dot(normal));
        return new Ground(heading, side, moving.Dot(heading), moving.Dot(side), moving.Length);
    }
}
