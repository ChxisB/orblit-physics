using System.Globalization;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// The three cars of <see cref="VehicleReferenceAddedTests"/> written as the
/// raw bits of the chassis and of every wheel, so a line here can be compared
/// with one from an independent build of the original solver word for word.
/// </summary>
internal static class SteppedCarScenes
{
    public static IReadOnlyList<string> All()
    {
        List<string> lines = [];
        Settle(lines);
        Steer(lines);
        Drive(lines);
        return lines;
    }

    private static void Settle(List<string> lines)
    {
        var (physics, car) = VehicleReferenceAddedTests.Make(VehicleReferenceAddedTests.Car());
        using var _ = physics;
        VehicleReferenceAddedTests.Run(physics, car, 3);
        Report(lines, "settle", physics, car);
    }

    private static void Steer(List<string> lines)
    {
        var (physics, car) = VehicleReferenceAddedTests.Make(VehicleReferenceAddedTests.Car());
        using var _ = physics;
        VehicleReferenceAddedTests.Run(physics, car, 2);
        physics.Drive(1, velocity: [0, 0, -3]);
        VehicleReferenceAddedTests.Run(physics, car, 1);
        car.SetInput(new VehicleInput(steer: 0.6));
        VehicleReferenceAddedTests.Run(physics, car, 2);
        Report(lines, "steer", physics, car);
    }

    private static void Drive(List<string> lines)
    {
        var (physics, car) = VehicleReferenceAddedTests.Make(VehicleReferenceAddedTests.Car(VehicleReferenceAddedTests.Drive()));
        using var _ = physics;
        VehicleReferenceAddedTests.Run(physics, car, 2);
        car.SetInput(new VehicleInput(throttle: 1));
        VehicleReferenceAddedTests.Run(physics, car, 4);
        Report(lines, "drive", physics, car);
    }

    private static void Report(List<string> lines, string scene, Physics physics, Vehicle car)
    {
        lines.Add($"car {scene} chassis T {Bits(physics.TransformOf(1)!)} V {Bits(physics.VelocityOf(1)!)}");
        for (var i = 0; i < car.Wheels.Count; i++)
        {
            var wheel = car.Wheels[i];
            var grounded = wheel.Grounded ? "true" : "false";
            lines.Add($"car {scene} wheel{i} {grounded} {Bits([wheel.Compression, wheel.Force, wheel.Spin, wheel.Steer, wheel.Roll])}");
        }

        lines.Add($"car {scene} rpm {Bits([car.Rpm])} gear {car.Gear.ToString(CultureInfo.InvariantCulture)}");
    }

    private static string Bits(IEnumerable<float> words) =>
        string.Join(' ', words.Select(w => BitConverter.SingleToUInt32Bits(w).ToString("x8", CultureInfo.InvariantCulture)));

    private static string Bits(IEnumerable<double> words) =>
        string.Join(' ', words.Select(w => BitConverter.DoubleToUInt64Bits(w).ToString("x16", CultureInfo.InvariantCulture)));
}
