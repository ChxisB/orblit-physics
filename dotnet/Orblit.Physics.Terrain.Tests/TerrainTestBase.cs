namespace Orblit.Physics.Terrain.Tests;

/// <summary>
/// The world every scenario runs in, and the helpers they share. It holds no
/// tests of its own.
/// </summary>
public abstract class TerrainTestBase : IDisposable
{
    protected Physics World { get; } = new();

    public void Dispose()
    {
        World.Dispose();
        GC.SuppressFinalize(this);
    }

    /// <summary>
    /// Runs the given seconds of simulation at a fixed sixtieth, which is what
    /// the solver is tuned for.
    /// </summary>
    protected void Run(double seconds)
    {
        for (var i = 0; i < (int)Math.Round(seconds * 60, MidpointRounding.AwayFromZero); i++)
        {
            World.Step(1.0 / 60);
        }
    }

    protected double HeightOf(ulong id) => World.TransformOf(id)![1];

    /// <summary>
    /// Where a ray straight down through world (<paramref name="x"/>,
    /// <paramref name="z"/>) meets the ground laid, or null where it meets none.
    /// </summary>
    protected double? CastAt(double x, double z)
    {
        var hit = World.Cast([x, 50.0, z], [0.0, -1.0, 0.0], 100.0);
        return hit is null ? null : 50.0 - hit.Distance;
    }
}
