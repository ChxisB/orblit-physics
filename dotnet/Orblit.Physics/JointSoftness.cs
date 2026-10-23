namespace Orblit.Physics;

/// <summary>
/// How soft the ends of a joint's limits are. With one, a limit is a spring
/// about its end and not a wall: pushed past, the joint gives and is pulled
/// back, and what stops there is held at the end. Locks stay hard.
/// </summary>
public sealed record JointSoftness
{
    public JointSoftness(double hertz, double ratio = 1.0)
    {
        SpringRules.Check(hertz, ratio, "A spring");
        Hertz = hertz;
        Ratio = ratio;
    }

    public double Hertz { get; }

    public double Ratio { get; }
}
