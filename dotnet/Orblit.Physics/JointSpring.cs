namespace Orblit.Physics;

/// <summary>
/// A spring on one way of a joint: it pulls that way to <see cref="Rest"/> as
/// a spring of <see cref="Hertz"/> would, with damping <see cref="Ratio"/>.
/// </summary>
/// <remarks>
/// On a way the joint locks, a spring takes the lock's place, so the joint
/// gives there. On a way it leaves free, the spring is the only thing that
/// holds it. Hertz is the way's own frequency under the load it moves: a turn
/// is measured against how hard the body is to swing about the joint, a body
/// hanging from a slider by its mass. A stiffer spring than a step can follow
/// is as stiff as one can.
/// </remarks>
public sealed record JointSpring
{
    /// <param name="hertz">Cycles a second, undamped, of the body on the end of it.</param>
    /// <param name="ratio">
    /// Nought rings for ever. One settles as quickly as it can without
    /// overshooting. Above one is sluggish.
    /// </param>
    /// <param name="rest">
    /// Where it pulls the way to, in metres or radians from where the joint
    /// was made. Null is where it was made, which for a distance joint is the
    /// length it was made at.
    /// </param>
    public JointSpring(double hertz, double ratio = 1.0, double? rest = null)
    {
        SpringRules.Check(hertz, ratio, "A spring");
        Hertz = hertz;
        Ratio = ratio;
        Rest = rest;
    }

    public double Hertz { get; }

    public double Ratio { get; }

    public double? Rest { get; }
}
