using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftWindTests : SoftTestBase
{
    /// <summary>A wind the test can switch.</summary>
    private sealed class Gust : ISoftAir
    {
        public Vector3 Wind { get; set; } = Vector3.Zero();

        public Vector3 WindAt(Vector3 point) => Wind;
    }

    /// <summary>A flag on a pole: the left column pinned, the sheet hanging along x.</summary>
    private static SoftBody Flag(Vector3? at = null, ISoftAir? air = null, SoftSpec? spec = null)
    {
        var flag = SoftBody.Cloth(
            Columns,
            Columns,
            at ?? new Vector3(0, 2, 0),
            new Vector3(Spacing, 0, 0),
            new Vector3(0, -Spacing, 0),
            spec: spec,
            air: air);
        for (var j = 0; j < Columns; j++)
        {
            flag.Pin(j * Columns);
        }

        return flag;
    }

    /// <summary>
    /// A world with one trigger box blowing <paramref name="wind"/>, centred at
    /// <paramref name="at"/>. Its top is below the flag's pole and above the
    /// flag's middle, so only a body that asks the wind at its middle is blown.
    /// </summary>
    private static (Physics Physics, PhysicsWind Air) Zone(Vector3 at, Vector3 wind)
    {
        var physics = new Physics();
        physics.Add(7, Shape.Box(2, 0.65, 3), PhysicsMotion.Fixed, at: [at.X, at.Y, at.Z], trigger: true);
        physics.Step(Dt);
        return (physics, new PhysicsWind(physics, new Dictionary<ulong, Vector3> { [7] = wind }));
    }

    [Fact]
    public void AFlagMovesDownwindInAZoneAndNotOutsideIt()
    {
        var (physics, air) = Zone(new Vector3(0.5, 1.15, 0), new Vector3(0, 0, 10));
        using (physics)
        {
            var inside = Flag(air: air);
            var outside = Flag(at: new Vector3(20, 2, 0), air: air);
            Run(inside, 3);
            Run(outside, 3);
            Greater(MeanZ(inside), 0.2);
            Less(Math.Abs(MeanZ(outside)), 1e-9);
        }
    }

    [Fact]
    public void AFreeSheetComesUpToTheSpeedOfTheWindAndNoMore()
    {
        var sheet = SoftBody.Cloth(
            Columns,
            Columns,
            new Vector3(0, 2, 0),
            new Vector3(Spacing, 0, 0),
            new Vector3(0, -Spacing, 0),
            spec: new SoftSpec(gravity: Vector3.Zero(), damping: 0, windDrag: 2),
            air: new Gust { Wind = new Vector3(0, 0, 5) });
        Run(sheet, 5);
        for (var i = 0; i < sheet.Count; i++)
        {
            Assert.Equal(5, sheet.VelocityOf(i).Z, 0.05);
        }
    }

    [Fact]
    public void AFlagHangsAsItDidWithoutAnyAirWhenNoneBlows()
    {
        var still = Flag();
        var calm = Flag(air: new Gust());
        Run(still, 3);
        Run(calm, 3);

        // Air drag on a body at rest in still air is only a little extra damping.
        var last = (Columns * Columns) - 1;
        Less(calm.PositionOf(last).DistanceTo(still.PositionOf(last)), 0.02);
    }

    [Fact]
    public void AFlagIsBlownHarderByAStrongerWindAndALargerDrag()
    {
        static double Blown(double wind, double drag)
        {
            var gust = new Gust { Wind = new Vector3(0, 0, wind) };
            var flag = Flag(air: gust, spec: new SoftSpec(windDrag: drag));
            Run(flag, 2);
            return MeanZ(flag);
        }

        Greater(Blown(10, 1), Blown(5, 1));
        Greater(Blown(10, 2), Blown(10, 0.5));
    }

    [Fact]
    public void AWindThatStartsWakesABodyThatHadSettled()
    {
        var gust = new Gust();
        var flag = Flag(air: gust);
        Run(flag, 12);
        Assert.True(flag.Asleep);
        gust.Wind = new Vector3(0, 0, 10);
        flag.Step(Dt);
        Assert.False(flag.Asleep);
        Run(flag, 2);
        Greater(MeanZ(flag), 0.2);
    }

    [Fact]
    public void ASteadyWindLetsItSettleAndSleep()
    {
        var flag = Flag(air: new Gust { Wind = new Vector3(0, 0, 3) });
        Run(flag, 20);
        Assert.True(flag.Asleep);
    }

    [Fact]
    public void OverlappingZonesAddAndATriggerThatIsNotWindBlowsNothing()
    {
        using var physics = new Physics();
        foreach (var id in new ulong[] { 1, 2, 3 })
        {
            physics.Add(id, Shape.Box(1, 1, 1), PhysicsMotion.Fixed, trigger: true);
        }

        physics.Step(Dt);
        var air = new PhysicsWind(
            physics,
            new Dictionary<ulong, Vector3> { [1] = new Vector3(1, 0, 0), [2] = new Vector3(0, 2, 0) });
        var wind = air.WindAt(Vector3.Zero());
        Assert.Equal([1.0, 2.0, 0.0], [wind.X, wind.Y, wind.Z]);
        var away = air.WindAt(new Vector3(9, 9, 9));
        Assert.Equal([0.0, 0.0, 0.0], [away.X, away.Y, away.Z]);
    }

    [Fact]
    public void ADragBelowNothingIsRefused()
    {
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(windDrag: -1));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(windDrag: double.PositiveInfinity));
    }
}
