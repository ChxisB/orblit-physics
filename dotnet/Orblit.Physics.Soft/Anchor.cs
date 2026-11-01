using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>A particle held to a point that moves, and how hard it may be pulled.</summary>
internal sealed class Anchor(int particle, Func<Vector3> where, double? limit, IReadOnlyList<int> links)
{
    public int Particle { get; } = particle;

    /// <summary>Where the particle is held, asked once at the start of each step.</summary>
    public Func<Vector3> Where { get; } = where;

    /// <summary>
    /// The force in newtons, averaged over a step, past which the particle lets
    /// go. Null holds it whatever the pull.
    /// </summary>
    public double? Limit { get; } = limit;

    /// <summary>The links that end at the particle.</summary>
    public IReadOnlyList<int> Links { get; } = links;

    /// <summary>
    /// The force the body puts on the anchor. While a step is under way it is
    /// the sum over the sub-steps so far, and after it the mean.
    /// </summary>
    public Vector3 Pull { get; } = Vector3.Zero();
}
