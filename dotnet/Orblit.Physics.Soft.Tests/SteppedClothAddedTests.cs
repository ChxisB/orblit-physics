namespace Orblit.Physics.Soft.Tests;

/// <summary>
/// A sheet hung from a corner and let fall onto a floor, compared particle for
/// particle with a run of the same sheet on an independent build of the
/// original solver, taken on this machine. Every word must be the same bit
/// for bit.
/// </summary>
public class SteppedClothAddedTests
{
    [Fact]
    public void ASheetFallingOntoAFloorLandsAsTheOriginalSolverLandsIt()
    {
        var pinned = File.ReadAllLines(Path.Combine(AppContext.BaseDirectory, "Reference", "stepped_cloth.txt"));

        Assert.Equal(pinned, SteppedClothScene.Run());
    }
}
