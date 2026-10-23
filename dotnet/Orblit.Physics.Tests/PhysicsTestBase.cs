namespace Orblit.Physics.Tests;

/// <summary>
/// The world every scenario runs in, and the helpers they share. It holds no
/// tests of its own.
/// </summary>
public abstract class PhysicsTestBase : IDisposable
{
    /// <summary>Ground at y = 0, facing up, never moving.</summary>
    protected const ulong Ground = 1;

    protected Physics World { get; private set; } = new();

    public void Dispose()
    {
        World.Dispose();
        GC.SuppressFinalize(this);
    }

    /// <summary>Replaces the world with a fresh one, for a scenario that runs twice.</summary>
    protected void Restart()
    {
        World.Dispose();
        World = new Physics();
    }

    /// <summary>Lays the floor every scenario here stands on.</summary>
    protected void Floor(double friction = 0.5) => World.Add(
        Ground,
        Shape.Plane(0.0, 1.0, 0.0),
        PhysicsMotion.Fixed,
        friction: friction);

    /// <summary>
    /// Runs the given seconds of simulation at a fixed sixtieth, which is what
    /// the solver is tuned for.
    /// </summary>
    protected void Run(double seconds)
    {
        for (var i = 0; i < Steps(seconds); i++)
        {
            World.Step(1.0 / 60);
        }
    }

    /// <summary>
    /// Walks character <paramref name="id"/> for <paramref name="seconds"/>,
    /// asking each step for what it was left with plus a step of gravity: the
    /// loop every game with a character runs. Answers how many steps it spent
    /// off the ground.
    /// </summary>
    protected int Walk(ulong id, double vx, double vz, double seconds)
    {
        var airborne = 0;
        for (var i = 0; i < Steps(seconds); i++)
        {
            var left = World.FootingOf(id)!.Velocity;
            World.Drive(id, [vx, left[1] - (9.81 / 60), vz]);
            World.Step(1.0 / 60);
            if (!World.FootingOf(id)!.Grounded)
            {
                airborne++;
            }
        }

        return airborne;
    }

    protected double HeightOf(ulong id) => World.TransformOf(id)![1];

    protected double FallSpeedOf(ulong id) => World.VelocityOf(id)![1];

    /// <summary>
    /// A square of ground this many samples wide, at <paramref name="height"/>
    /// everywhere unless <paramref name="heightAt"/> says otherwise.
    /// </summary>
    protected static double[] HeightsOf(int samples, double height = 0.0, Func<int, int, double>? heightAt = null)
    {
        var heights = new double[samples * samples];
        for (var r = 0; r < samples; r++)
        {
            for (var c = 0; c < samples; c++)
            {
                heights[(r * samples) + c] = heightAt?.Invoke(c, r) ?? height;
            }
        }

        return heights;
    }

    private static int Steps(double seconds) => (int)Math.Round(seconds * 60, MidpointRounding.AwayFromZero);
}
