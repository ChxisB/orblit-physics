using Orblit.Physics.Tests;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// The three cars of <see cref="VehicleReferenceAddedTests"/> compared with a
/// run of the same cars on an independent build of the original solver, taken
/// on a Mac. The chassis and every wheel must be the same bit for bit there,
/// and close to it on another system.
/// </summary>
public class SteppedCarAddedTests
{
    private static string[] Pinned(string scene) => File
        .ReadAllLines(Path.Combine(AppContext.BaseDirectory, "Reference", "stepped_cars.txt"))
        .Where(line => line.StartsWith($"car {scene} ", StringComparison.Ordinal))
        .ToArray();

    private static string[] Ran(string scene) => SteppedCarScenes
        .All()
        .Where(line => line.StartsWith($"car {scene} ", StringComparison.Ordinal))
        .ToArray();

    [Theory]
    [InlineData("settle")]
    [InlineData("steer")]
    [InlineData("drive")]
    public void ACarEndsAsTheOriginalSolverLeavesItBitForBit(string scene)
    {
        Assert.NotEmpty(Pinned(scene));
        SteppedDump.AssertSame(Pinned(scene), Ran(scene));
    }
}
