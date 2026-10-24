namespace Orblit.Physics.Tests;

/// <summary>
/// Three worlds stepped for a few seconds, compared body for body with a run
/// of the same worlds on an independent build of the original solver, taken on
/// this machine. Every word of every body must be the same bit for bit.
/// </summary>
public class SteppedWorldAddedTests
{
    private static string[] Pinned(string scene) => File
        .ReadAllLines(Path.Combine(AppContext.BaseDirectory, "Reference", "stepped_world.txt"))
        .Where(line => line.StartsWith(scene + " ", StringComparison.Ordinal))
        .ToArray();

    [Fact]
    public void AWallOfBoxesHitByASphereFallsAsTheOriginalSolverDropsIt()
    {
        Assert.Equal(Pinned("wall"), SteppedScenes.Wall());
    }

    [Fact]
    public void ACharacterThatClimbsALedgeAndPushesACrateEndsWhereTheOriginalSolverPutsIt()
    {
        Assert.Equal(Pinned("character"), SteppedScenes.Character());
    }

    [Fact]
    public void AChainOfJointsSwingsAsTheOriginalSolverSwingsIt()
    {
        Assert.Equal(Pinned("chain"), SteppedScenes.Chain());
    }
}
