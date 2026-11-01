using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// A joint that bends about one axis: a knee, an elbow, a finger.
/// </summary>
/// <remarks>
/// <see cref="Axis"/> is in the bone's own space, whose Y runs head to tail.
/// <see cref="Low"/> and <see cref="High"/> are how far it may turn from the
/// pose the rig rests in, in radians, so a knee that only bends one way is
/// low 0, high 2.4.
/// </remarks>
public sealed class RagdollHinge : RagdollJoint
{
    public RagdollHinge(Vector3? axis = null, double low = 0, double high = 2.4)
    {
        Axis = (axis ?? new Vector3(1, 0, 0)).Normalized();
        if (Axis.Length2 < 0.5)
        {
            throw new ArgumentOutOfRangeException(nameof(axis), axis, "A hinge needs an axis.");
        }

        if (!double.IsFinite(low) || !double.IsFinite(high) || low > high)
        {
            throw new ArgumentException("A hinge goes from low to high, with low first.", nameof(low));
        }

        Low = low;
        High = high;
    }

    public Vector3 Axis { get; }

    public double Low { get; }

    public double High { get; }
}
