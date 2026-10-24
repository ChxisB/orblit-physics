namespace Orblit.Physics.Vehicle;

/// <summary>
/// A car as data.
/// </summary>
/// <remarks>
/// The chassis' own front is along its negative z, as a camera's is. Wheels
/// that share a mount z are an axle. With <see cref="AntiRoll"/> above zero
/// every axle has exactly two.
/// </remarks>
public sealed class VehicleSpec
{
    // F2: seven parameters, all but the first two optional and called by name.
    public VehicleSpec(
        double mass,
        IReadOnlyList<WheelSpec> wheels,
        IReadOnlyList<double>? centre = null,
        double antiRoll = 0.0,
        double maxSteer = 0.5,
        DriveSpec? drive = null,
        Layers? ground = null)
    {
        Wheels = Array.AsReadOnly(wheels.ToArray());
        Centre = Array.AsReadOnly(centre?.ToArray() ?? [0.0, 0.0, 0.0]);
        Axles = AxlesOf(Wheels, antiRoll);
        Sprung = SprungMass(Wheels, mass, Centre);
        CheckBody(mass, Wheels, drive, maxSteer);

        Mass = mass;
        AntiRoll = antiRoll;
        MaxSteer = maxSteer;
        Drive = drive;
        Ground = ground ?? Layers.Everything;
    }

    /// <summary>Kilograms.</summary>
    public double Mass { get; }

    public IReadOnlyList<WheelSpec> Wheels { get; }

    /// <summary>Where its weight is, in the chassis frame.</summary>
    public IReadOnlyList<double> Centre { get; }

    /// <summary>
    /// Newtons a metre of difference in compression across an axle. Zero is no
    /// bar.
    /// </summary>
    public double AntiRoll { get; }

    /// <summary>Radians the middle of the steering axle turns at full lock.</summary>
    public double MaxSteer { get; }

    /// <summary>
    /// The engine and what joins it to the wheels that drive. Null is a car that
    /// is pushed.
    /// </summary>
    public DriveSpec? Drive { get; }

    /// <summary>
    /// The layers a wheel sees as ground, so a car does not stand on a
    /// character.
    /// </summary>
    public Layers Ground { get; }

    /// <summary>The pairs of wheels, by index, that an anti-roll bar joins.</summary>
    public IReadOnlyList<(int A, int B)> Axles { get; }

    /// <summary>The kilograms each wheel carries at rest, in the order of <see cref="Wheels"/>.</summary>
    public IReadOnlyList<double> Sprung { get; }

    private static void CheckBody(double mass, IReadOnlyList<WheelSpec> wheels, DriveSpec? drive, double maxSteer)
    {
        if (!(mass > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(mass), mass, "Must be above zero.");
        }

        if (wheels.Count == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(wheels), wheels.Count, "Needs at least one.");
        }

        if (drive == null && wheels.Any(w => w.Drives))
        {
            throw new ArgumentOutOfRangeException(nameof(wheels), wheels.Count, "A driven wheel needs a drive.");
        }

        if (drive != null && !wheels.Any(w => w.Drives))
        {
            throw new ArgumentOutOfRangeException(nameof(drive), drive, "Needs a wheel that drives.");
        }

        if (!(maxSteer >= 0 && maxSteer < Math.PI / 2))
        {
            throw new ArgumentOutOfRangeException(nameof(maxSteer), maxSteer, "Must be 0 to a quarter turn.");
        }
    }

    private static (int A, int B)[] AxlesOf(IReadOnlyList<WheelSpec> wheels, double antiRoll)
    {
        if (!(antiRoll >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(antiRoll), antiRoll, "Must not be negative.");
        }

        if (antiRoll == 0)
        {
            return [];
        }

        var byZ = new OrderedDictionary<double, List<int>>();
        for (var i = 0; i < wheels.Count; i++)
        {
            var z = wheels[i].Mount.Z;
            if (!byZ.TryGetValue(z, out var axle))
            {
                axle = [];
                byZ[z] = axle;
            }

            axle.Add(i);
        }

        if (byZ.Values.Any(axle => axle.Count != 2))
        {
            throw new ArgumentOutOfRangeException(
                nameof(wheels),
                wheels.Count,
                "An anti-roll bar needs exactly two wheels on each axle.");
        }

        return [.. byZ.Values.Select(axle => (axle[0], axle[1]))];
    }

    /// <summary>
    /// Shares the mass out so that the wheels' loads balance about the centre of
    /// mass: an even share, tilted across the wheels as far as the centre is
    /// from their middle. Wheels in a line share it evenly.
    /// </summary>
    private static double[] SprungMass(IReadOnlyList<WheelSpec> wheels, double mass, IReadOnlyList<double> centre)
    {
        var count = wheels.Count;

        // The constructor refuses these with a clearer message once it is in.
        if (count == 0 || !(mass > 0))
        {
            return [];
        }

        var midX = Mean(wheels, w => w.Mount.X);
        var midZ = Mean(wheels, w => w.Mount.Z);
        var (xx, xz, zz) = (0.0, 0.0, 0.0);
        foreach (var w in wheels)
        {
            var (x, z) = (w.Mount.X - midX, w.Mount.Z - midZ);
            xx += x * x;
            xz += x * z;
            zz += z * z;
        }

        var det = (xx * zz) - (xz * xz);
        var (wantX, wantZ) = (mass * (centre[0] - midX), mass * (centre[2] - midZ));
        var (a, b) = Math.Abs(det) < 1e-9
            ? (0.0, 0.0)
            : (((wantX * zz) - (wantZ * xz)) / det, ((wantZ * xx) - (wantX * xz)) / det);

        var loads = wheels
            .Select(w => (mass / count) + (a * (w.Mount.X - midX)) + (b * (w.Mount.Z - midZ)))
            .ToArray();
        if (loads.Any(load => !(load > 0)))
        {
            throw new ArgumentOutOfRangeException(
                nameof(centre),
                string.Join(", ", centre),
                "The centre of mass must lie inside the wheels.");
        }

        return loads;
    }

    private static double Mean(IReadOnlyList<WheelSpec> wheels, Func<WheelSpec, double> of)
    {
        var sum = 0.0;
        foreach (var w in wheels)
        {
            sum += of(w);
        }

        return sum / wheels.Count;
    }
}
