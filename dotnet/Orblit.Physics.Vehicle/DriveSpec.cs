namespace Orblit.Physics.Vehicle;

/// <summary>
/// What carries an engine's torque to the driven wheels: a clutch, a gearbox
/// and a differential, shifting by itself unless the driver picks a gear.
/// </summary>
/// <remarks>
/// Ratios are turns of the engine for a turn of the wheel. The final drive
/// multiplies every gear. The clutch is a centrifugal one: it holds nothing at
/// idle and grips harder as the engine speeds up, so a car pulls away without
/// stalling.
/// </remarks>
public sealed class DriveSpec
{
    // F2: ten parameters, all but the first two optional and called by name.
    public DriveSpec(
        EngineSpec engine,
        IReadOnlyList<double> ratios,
        double reverse = 3.2,
        double finalDrive = 3.7,
        double? clutch = null,
        double? engage = null,
        double diffLock = 0.0,
        double shiftTime = 0.3,
        double upshift = 0.85,
        double downshift = 0.35)
    {
        Engine = engine;
        Ratios = Array.AsReadOnly(ratios.ToArray());
        Clutch = clutch ?? (engine.Peak * 1.5);
        Engage = engage ?? (engine.Idle + ((engine.Redline - engine.Idle) * 0.25));
        CheckGears(Ratios, reverse, finalDrive);
        CheckClutch(Clutch, Engage, engine);
        if (!(diffLock >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(diffLock), diffLock, "Must not be negative.");
        }

        if (!(shiftTime >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(shiftTime), shiftTime, "Must not be negative.");
        }

        CheckShifts(Ratios, upshift, downshift);
        Reverse = reverse;
        FinalDrive = finalDrive;
        Lock = diffLock;
        ShiftTime = shiftTime;
        Upshift = upshift;
        Downshift = downshift;
    }

    public EngineSpec Engine { get; }

    /// <summary>The forward gears, first to last.</summary>
    public IReadOnlyList<double> Ratios { get; }

    public double Reverse { get; }

    public double FinalDrive { get; }

    /// <summary>The most newton metres the clutch holds once it has taken hold.</summary>
    public double Clutch { get; }

    /// <summary>
    /// Revolutions a minute the clutch takes hold at in full. It starts to bite a
    /// tenth above idle.
    /// </summary>
    public double Engage { get; }

    /// <summary>
    /// Newton metres of difference the differential holds between its two wheels.
    /// Zero is an open one, which gives each wheel the same torque. Infinity is a
    /// locked one. Between is a limited slip.
    /// </summary>
    public double Lock { get; }

    /// <summary>Seconds the clutch takes to come back after a shift.</summary>
    public double ShiftTime { get; }

    /// <summary>The share of the redline at which the box changes up.</summary>
    public double Upshift { get; }

    /// <summary>The share of the redline at which the box changes down.</summary>
    public double Downshift { get; }

    private static void CheckGears(IReadOnlyList<double> ratios, double reverse, double finalDrive)
    {
        if (ratios.Count == 0 || ratios.Any(ratio => !(ratio > 0)))
        {
            throw new ArgumentOutOfRangeException(nameof(ratios), string.Join(", ", ratios), "Needs gears above zero.");
        }

        if (!(reverse > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(reverse), reverse, "Must be above zero.");
        }

        if (!(finalDrive > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(finalDrive), finalDrive, "Must be above zero.");
        }
    }

    private static void CheckClutch(double clutch, double engage, EngineSpec engine)
    {
        if (!(clutch > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(clutch), clutch, "Must be above zero.");
        }

        if (!(engage > engine.Idle * 1.1 && engage < engine.Redline))
        {
            throw new ArgumentOutOfRangeException(
                nameof(engage),
                engage,
                "Must be above idle by a tenth, to the redline.");
        }
    }

    private static void CheckShifts(IReadOnlyList<double> ratios, double upshift, double downshift)
    {
        if (!(downshift > 0 && downshift < upshift && upshift <= 1))
        {
            throw new ArgumentOutOfRangeException(
                nameof(upshift),
                upshift,
                "Must be above downshift, and at most one.");
        }

        for (var i = 1; i < ratios.Count; i++)
        {
            if (downshift * ratios[i - 1] / ratios[i] >= upshift)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ratios),
                    string.Join(", ", ratios),
                    $"A downshift from gear {i + 1} would land above the upshift.");
            }
        }
    }
}
