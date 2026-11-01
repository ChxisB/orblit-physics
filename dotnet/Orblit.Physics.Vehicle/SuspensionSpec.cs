namespace Orblit.Physics.Vehicle;

/// <summary>
/// How a wheel is held to the chassis.
/// </summary>
/// <remarks>
/// The spring is written as a frequency so that a kart and a truck use the
/// same numbers. The stiffness follows from the weight the wheel carries.
/// </remarks>
public sealed class SuspensionSpec
{
    /// <summary>
    /// The fastest spring a step of <see cref="Vehicle.MaxStep"/> holds. Past it
    /// the spring overshoots in a single step and the car buzzes.
    /// </summary>
    public const double MaxFrequency = 8.0;

    public SuspensionSpec(double travel = 0.3, double frequency = 1.5, double damping = 0.5)
    {
        if (!(travel > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(travel), travel, "Must be above zero.");
        }

        if (!(frequency > 0 && frequency <= MaxFrequency))
        {
            throw new ArgumentOutOfRangeException(
                nameof(frequency),
                frequency,
                $"Must be above zero and at most {MaxFrequency} Hz.");
        }

        if (!(damping >= 0 && damping <= 2))
        {
            throw new ArgumentOutOfRangeException(nameof(damping), damping, "Must be 0 to 2.");
        }

        Travel = travel;
        Frequency = frequency;
        Damping = damping;
    }

    /// <summary>
    /// How far the wheel can move, in metres, from hanging free to the mount.
    /// The spring is at rest when the wheel hangs free.
    /// </summary>
    public double Travel { get; }

    /// <summary>Cycles a second of the car's weight on this spring.</summary>
    public double Frequency { get; }

    /// <summary>Of critical damping. Zero rings, one settles in a single swing.</summary>
    public double Damping { get; }

    /// <summary>
    /// Metres the spring is squeezed once it holds the weight it was tuned for,
    /// under <paramref name="gravity"/> metres a second squared. The stiffness
    /// follows the weight, so the squeeze does not depend on how heavy the
    /// wheel's share is. Never more than <see cref="Travel"/>.
    /// </summary>
    public double Sag(double gravity = 9.81)
    {
        var omega = 2 * Math.PI * Frequency;
        return Math.Min(Travel, gravity / (omega * omega));
    }
}
