namespace Orblit.Physics.Vehicle;

/// <summary>
/// The bristles of one axis as a spring and a damper, taken over a step.
/// </summary>
/// <remarks>
/// The damper acts on how fast the bristles bend, not on how fast the patch
/// slips, so it quiets a parked wheel and has nothing to say to one that is
/// rolling. Both are solved at the end of the step. Whatever mass the force
/// falls on, the answer is stable.
/// </remarks>
/// <param name="Stiffness">Newtons a metre.</param>
/// <param name="Keep">The share of the bend that stays after the wheel has rolled for a step.</param>
/// <param name="Dt">Seconds in the step.</param>
internal sealed record Bristles(double Stiffness, double Keep, double Dt)
{
    /// <summary>
    /// The force that holds a patch with <paramref name="slip"/> metres a second
    /// over bristles bent by <paramref name="bent"/> metres, falling on
    /// <paramref name="mass"/> kilograms.
    /// </summary>
    public double Force(double bent, double slip, double mass)
    {
        var damper = Damper(mass);
        var both = Stiffness + damper;
        return ((damper * bent) - (both * Keep * (bent + (Dt * slip))))
            / (1 + (both * Keep * Dt * Dt / mass));
    }

    /// <summary>How far the bristles are bent once <paramref name="force"/> has acted.</summary>
    public double Bent(double bent, double force, double mass)
    {
        var damper = Damper(mass);
        return ((damper * bent) - force) / (Stiffness + damper);
    }

    /// <summary>Critical damping for <paramref name="mass"/>, as a force for a bend that goes in one step.</summary>
    private double Damper(double mass) => 2 * Math.Sqrt(Stiffness * mass) / Dt;
}
