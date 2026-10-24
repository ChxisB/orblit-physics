using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>
/// The moving air a soft body sits in.
/// </summary>
/// <remarks>
/// Declared here, next to the body that asks it, so a body can be blown by a
/// physics world, a noise field or a constant without knowing which.
/// </remarks>
public interface ISoftAir
{
    /// <summary>
    /// The velocity of the air at <paramref name="point"/>, in metres a second.
    /// Asked once a step, at the middle of the body.
    /// </summary>
    Vector3 WindAt(Vector3 point);
}
