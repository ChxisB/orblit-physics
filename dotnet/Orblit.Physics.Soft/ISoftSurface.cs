using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>What a soft body asks of the world around it.</summary>
public interface ISoftSurface
{
    /// <summary>
    /// Moves a ball of <paramref name="radius"/> from <paramref name="from"/>
    /// along <paramref name="direction"/> for <paramref name="distance"/>
    /// metres and answers the first surface it meets, or null.
    /// </summary>
    /// <remarks>
    /// The direction need not be a unit vector. A ball that starts clear of a
    /// surface answers a point on it. One that starts inside is met at once,
    /// and the point is only near the surface.
    /// </remarks>
    SoftContact? Sweep(Vector3 from, Vector3 direction, double distance, double radius);
}
