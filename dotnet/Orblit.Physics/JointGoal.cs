namespace Orblit.Physics;

/// <summary>
/// A motor that holds its way at <see cref="At"/>, in metres or radians, as a
/// spring of <see cref="Hertz"/> and damping <see cref="Ratio"/> would, and
/// never pushes harder than the joint's strength. It has no use for the joint's
/// speed.
/// </summary>
/// <remarks>
/// Where a speed motor says how fast to go, this says where to be: an arm held
/// up, a door held shut, a limb driven to the pose an animation asks for.
/// </remarks>
public sealed record JointGoal
{
    public JointGoal(double at, double hertz, double ratio = 1.0)
    {
        SpringRules.Check(hertz, ratio, "A goal");
        At = at;
        Hertz = hertz;
        Ratio = ratio;
    }

    public double At { get; }

    public double Hertz { get; }

    public double Ratio { get; }
}
