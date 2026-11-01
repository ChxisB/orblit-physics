namespace Orblit.Physics.Vehicle;

/// <summary>
/// An engine: the torque it gives at each speed with the pedal down.
/// </summary>
public sealed class EngineSpec
{
    public EngineSpec(
        IReadOnlyList<(double Rpm, double NewtonMetres)> torque,
        double idle = 900.0,
        double inertia = 0.25,
        double drag = 40.0)
    {
        Torque = Array.AsReadOnly(torque.ToArray());
        if (Torque.Count < 2)
        {
            throw new ArgumentOutOfRangeException(nameof(torque), Torque.Count, "Needs two points at least.");
        }

        CheckCurve(Torque);
        if (!(idle > 0 && idle < Redline))
        {
            throw new ArgumentOutOfRangeException(nameof(idle), idle, "Must be above 0, to the redline.");
        }

        if (!(inertia > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(inertia), inertia, "Must be above zero.");
        }

        if (!(drag >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(drag), drag, "Must not be negative.");
        }

        Idle = idle;
        Inertia = inertia;
        Drag = drag;
    }

    /// <summary>
    /// Revolutions a minute and the newton metres at the crank with the pedal
    /// down, in order of speed. Between two points it is a straight line.
    /// </summary>
    public IReadOnlyList<(double Rpm, double NewtonMetres)> Torque { get; }

    /// <summary>Revolutions a minute it ticks over at with the pedal up.</summary>
    public double Idle { get; }

    /// <summary>Kilogram square metres the engine turns with.</summary>
    public double Inertia { get; }

    /// <summary>
    /// Newton metres the engine drags at the redline with the pedal up. It falls
    /// away in a straight line to nothing at rest.
    /// </summary>
    public double Drag { get; }

    /// <summary>The top speed on the curve. The engine gives nothing a little past it.</summary>
    public double Redline => Torque[^1].Rpm;

    /// <summary>The most newton metres on the curve.</summary>
    public double Peak
    {
        get
        {
            var most = 0.0;
            foreach (var (_, newtonMetres) in Torque)
            {
                most = newtonMetres > most ? newtonMetres : most;
            }

            return most;
        }
    }

    /// <summary>Newton metres at <paramref name="rpm"/> with the pedal down.</summary>
    public double Full(double rpm)
    {
        if (rpm <= Torque[0].Rpm)
        {
            return Torque[0].NewtonMetres;
        }

        for (var i = 1; i < Torque.Count; i++)
        {
            var (high, top) = Torque[i];
            if (rpm > high)
            {
                continue;
            }

            var (low, bottom) = Torque[i - 1];
            return bottom + ((top - bottom) * (rpm - low) / (high - low));
        }

        return Torque[^1].NewtonMetres;
    }

    private static void CheckCurve(IReadOnlyList<(double Rpm, double NewtonMetres)> torque)
    {
        for (var i = 0; i < torque.Count; i++)
        {
            var (rpm, newtonMetres) = torque[i];
            if (!(rpm > 0 && newtonMetres >= 0))
            {
                throw new ArgumentOutOfRangeException(nameof(torque), torque[i], $"Point {i} is out of range.");
            }

            if (i > 0 && !(rpm > torque[i - 1].Rpm))
            {
                throw new ArgumentOutOfRangeException(nameof(torque), torque[i], "Must rise in rpm.");
            }
        }
    }
}
