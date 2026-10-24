namespace Orblit.Physics.Vehicle;

/// <summary>
/// What a tyre holds the ground with.
/// </summary>
/// <remarks>
/// The force a tyre gives rises with slip up to <see cref="Grip"/> times the
/// load on it, and falls to <see cref="Sliding"/> of that once the patch is
/// slipping twice as much as <see cref="PeakSlip"/>. Slip is the speed of the
/// patch over the ground as a share of the speed the wheel is rolling at, or
/// of one metre a second below that.
/// </remarks>
public sealed class TyreSpec
{
    public TyreSpec(double grip = 1.0, double peakSlip = 0.1, double sliding = 0.8, double relaxation = 0.2)
    {
        if (!(grip > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(grip), grip, "Must be above zero.");
        }

        if (!(peakSlip > 0 && peakSlip <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(peakSlip), peakSlip, "Must be above 0, to 1.");
        }

        if (!(sliding > 0 && sliding <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(sliding), sliding, "Must be above 0, to 1.");
        }

        if (!(relaxation > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(relaxation), relaxation, "Must be above zero.");
        }

        Grip = grip;
        PeakSlip = peakSlip;
        Sliding = sliding;
        Relaxation = relaxation;
    }

    /// <summary>
    /// The most force the rubber gives for each newton of load, on a surface
    /// whose grip is one.
    /// </summary>
    public double Grip { get; }

    /// <summary>The slip, as a share, at which the force peaks.</summary>
    public double PeakSlip { get; }

    /// <summary>
    /// What is left of <see cref="Grip"/> when the patch slides, from zero to
    /// one. One is a tyre that does not fall off.
    /// </summary>
    public double Sliding { get; }

    /// <summary>
    /// Metres the wheel rolls for the force to build to its steady value. A
    /// short one answers quickly, a long one is soft.
    /// </summary>
    public double Relaxation { get; }
}
