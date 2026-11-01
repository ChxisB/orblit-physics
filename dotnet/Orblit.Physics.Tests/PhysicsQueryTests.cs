using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class PhysicsQueryTests : PhysicsTestBase
{
    /// <summary>A cube trigger of half-size <paramref name="half"/>, kept still.</summary>
    private void Trigger(ulong id, double[] at, double half = 1.0, bool stay = false) => World.Add(
        id,
        Shape.Box(half, half, half),
        PhysicsMotion.Fixed,
        at,
        trigger: true,
        stay: stay);

    /// <summary>Steps this many seconds and keeps every event, since a step drops the last one's.</summary>
    private List<PhysicsEvent> Watch(double seconds)
    {
        var seen = new List<PhysicsEvent>();
        for (var i = 0; i < Steps(seconds); i++)
        {
            World.Step(1.0 / 60);
            seen.AddRange(World.Events);
        }

        return seen;
    }

    private static int Count(List<PhysicsEvent> seen, PhysicsEventKind kind, ulong a, ulong b) =>
        seen.Count(it => it.Kind == kind && it.A == a && it.B == b);

    /// <summary>
    /// Three boxes along +x, made in the reverse of the order a ray meets them,
    /// so that meeting them in order is not an accident of storage.
    /// </summary>
    private void Row()
    {
        for (ulong i = 3; i >= 1; i--)
        {
            World.Add(i, Shape.Box(0.5, 0.5, 0.5), PhysicsMotion.Fixed, [3.0 * i, 0.0, 0.0]);
        }
    }

    private IReadOnlyList<PhysicsHit> AlongRow(ulong ignore = 0, int limit = 32, double x = 0.0) =>
        World.CastAll([x, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, ignore: ignore, limit: limit);

    [Fact]
    public void ItsStructsAreTheSizeTheEngineReads()
    {
        Assert.Equal(48, Marshal.SizeOf<NativeZone>());
        Assert.Equal(48, Marshal.SizeOf<NativeRule>());
    }

    [Fact]
    public void ABodyFallingThroughATriggerEntersAndLeavesIt()
    {
        World.Add(9, Shape.Plane(0.0, 1.0, 0.0, offset: -6.0), PhysicsMotion.Fixed);
        Trigger(1, [0.0, 0.0, 0.0]);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 3.0, 0.0]);

        var seen = Watch(2.0);

        Assert.Equal(1, Count(seen, PhysicsEventKind.Entered, 1, 2));
        Assert.Equal(1, Count(seen, PhysicsEventKind.Exited, 1, 2));
        Assert.Equal(0, Count(seen, PhysicsEventKind.TouchBegan, 1, 2));
        Assert.Equal(-5.75, HeightOf(2), 0.1);
    }

    [Fact]
    public void InsideAndTouchStayAreHeardOnlyByABodyThatAsks()
    {
        Floor();
        Trigger(5, [0.0, 0.0, 0.0], stay: true);
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [5.0, 0.49, 0.0], stay: true);
        World.Add(3, Shape.Box(0.5, 0.5, 0.5), at: [8.0, 0.49, 0.0]);
        World.Add(4, Shape.Sphere(0.25), PhysicsMotion.Driven, [0.0, 0.0, 0.0]);

        var seen = Watch(0.3);

        Assert.True(Count(seen, PhysicsEventKind.Inside, 5, 4) > 10);
        Assert.True(Count(seen, PhysicsEventKind.TouchStay, Ground, 2) > 10);
        Assert.Equal(1, Count(seen, PhysicsEventKind.TouchBegan, Ground, 3));
        Assert.Equal(0, Count(seen, PhysicsEventKind.TouchStay, Ground, 3));
    }

    [Fact]
    public void EnteringNamesTheTriggerFirstAndGivesAUnitNormal()
    {
        Trigger(5, [0.0, 0.0, 0.0]);
        World.Add(2, Shape.Sphere(0.25), PhysicsMotion.Driven, [0.9, 0.0, 0.0]);

        var entered = Watch(0.1).Single(it => it.Kind == PhysicsEventKind.Entered);

        Assert.Equal(new ulong[] { 5, 2 }, new[] { entered.A, entered.B });
        var n = entered.Normal;
        Assert.Equal(1.0, (n[0] * n[0]) + (n[1] * n[1]) + (n[2] * n[2]), 0.01);
    }

    [Fact]
    public void AZoneWithNoGravityHoldsABodyAndRemovingItLetsGo()
    {
        Trigger(1, [0.0, 10.0, 0.0], half: 20.0);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0]);

        Assert.True(World.SetZone(1, new PhysicsZone(Gravity: [0.0, 0.0, 0.0])));
        Run(1.0);
        Assert.Equal(10.0, HeightOf(2), 0.05);

        Assert.True(World.RemoveZone(1));
        Run(1.0);
        Assert.True(HeightOf(2) < 6.0);
        Assert.False(World.RemoveZone(1));
    }

    [Fact]
    public void ZonesAgreeByPriorityThenByLowestId()
    {
        Assert.True(HeightUnder((1, 0, 0.0), (3, 5, 9.81)) > 12.0);
        Assert.True(HeightUnder((1, 5, 9.81), (3, 0, 0.0)) > 12.0);
        Assert.True(HeightUnder((3, 3, -9.81), (1, 3, 9.81)) > 12.0);
    }

    /// <summary>Each zone is an id, a priority and an upward gravity, all over the same ball.</summary>
    private static double HeightUnder(params (ulong Id, int Priority, double Gravity)[] zones)
    {
        using var world = new Physics();
        world.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0]);
        foreach (var (id, priority, gravity) in zones)
        {
            world.Add(id, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
            world.SetZone(id, new PhysicsZone(Gravity: [0.0, gravity, 0.0], Priority: priority));
        }

        for (var i = 0; i < 60; i++)
        {
            world.Step(1.0 / 60);
        }

        return world.TransformOf(2)![1];
    }

    [Fact]
    public void AZoneCanHoldDampingAndLeaveGravityAlone()
    {
        Trigger(1, [0.0, 10.0, 0.0], half: 20.0);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0], linearDamping: 0.0);
        World.SetZone(1, new PhysicsZone(LinearDamping: 8.0));
        World.Drive(2, [5.0, 0.0, 0.0]);

        Run(1.0);

        Assert.True(World.VelocityOf(2)![0] < 0.1);
        Assert.True(HeightOf(2) < 10.0);
    }

    [Fact]
    public void AFluidZoneFloatsALightBoxAndSinksAHeavyOne()
    {
        Trigger(1, [0.0, -20.0, 0.0], half: 20.0);
        Assert.True(World.SetZone(1, new PhysicsZone(FluidDensity: 1000.0, FluidDrag: 3.0)));
        // A box a metre across holds a cubic metre, so its mass is its density.
        foreach (var (id, mass) in new[] { (2UL, 500.0), (3UL, 2000.0) })
        {
            World.Add(id, Shape.Box(0.5, 0.5, 0.5), at: [id * 3.0, 0.3, 0.0], mass: mass);
        }

        Run(12.0);

        Assert.Equal(0.0, HeightOf(2), 0.03);
        Assert.True(HeightOf(3) < -3.0);
        Assert.False(World.SetZone(1, new PhysicsZone(FluidDensity: -1.0)));
    }

    [Fact]
    public void AZoneIsRefusedOnWhatIsNotATriggerAndForNaN()
    {
        World.Add(1, Shape.Sphere(0.5), at: [0.0, 2.0, 0.0]);
        Trigger(2, [0.0, 0.0, 0.0]);
        var weightless = new PhysicsZone(Gravity: [0.0, 0.0, 0.0]);

        Assert.False(World.SetZone(1, weightless));
        Assert.False(World.SetZone(99, weightless));
        Assert.False(World.SetZone(2, new PhysicsZone(Gravity: [0.0, double.NaN, 0.0])));
    }

    [Fact]
    public void ABeltCarriesABoxAtItsOwnSpeedAndReversingItTurnsIt()
    {
        World.Add(1, Shape.Box(8.0, 0.25, 1.0), PhysicsMotion.Fixed, [0.0, 0.25, 0.0]);
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 1.0, 0.0]);

        World.SetSurface(1, [2.0, 0.0, 0.0]);
        Run(2.0);
        Assert.Equal(2.0, World.VelocityOf(2)![0], 0.2);
        Assert.Equal(1.0, HeightOf(2), 0.02);

        World.SetSurface(1, [-2.0, 0.0, 0.0]);
        Run(2.0);
        Assert.Equal(-2.0, World.VelocityOf(2)![0], 0.2);

        World.SetSurface(1, [0.0, 0.0, 0.0]);
        Run(2.0);
        Assert.True(Math.Abs(World.VelocityOf(2)![0]) < 0.1);
    }

    [Fact]
    public void ARuleWithNoFrictionLetsOneBoxSlideOnGrippyGround()
    {
        Floor(friction: 1.0);
        foreach (var (id, z) in new[] { (2UL, 0.0), (3UL, 5.0) })
        {
            World.Add(id, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 0.5, z], friction: 1.0);
            World.Drive(id, [4.0, 0.0, 0.0]);
        }

        World.SetRule(2, Ground, new PhysicsRule(friction: 0.0));

        Run(2.0);

        Assert.True(World.TransformOf(3)![0] < 1.0);
        Assert.True(World.TransformOf(2)![0] > 2.5);
    }

    [Fact]
    public void ARuleIsRemovedByNamingItAndEndsWithEitherBody()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 0.5, 0.0]);
        var bouncy = new PhysicsRule(restitution: 0.9);

        Assert.True(World.SetRule(2, Ground, bouncy));
        Assert.True(World.RemoveRule(Ground, 2));
        Assert.False(World.RemoveRule(Ground, 2));

        World.SetRule(2, Ground, bouncy);
        World.Remove(2);
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 0.5, 0.0]);
        Assert.False(World.RemoveRule(2, Ground));
    }

    [Fact]
    public void ARuleIsRefusedForABodyAgainstItselfAndForNonsense()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 0.5, 0.0]);

        Assert.False(World.SetRule(2, 2, new PhysicsRule(friction: 0.0)));
        Assert.False(World.SetRule(2, 55, new PhysicsRule(friction: 0.0)));
        Assert.False(World.SetRule(2, Ground, new PhysicsRule(friction: double.NaN)));
        Assert.False(World.SetRule(2, Ground, new PhysicsRule(moveScaleA: -1.0)));
    }

    [Fact]
    public void CastAllMeetsEveryBodyAlongTheWayNearestFirst()
    {
        Row();

        var all = AlongRow();

        Assert.Equal(new ulong[] { 1, 2, 3 }, all.Select(it => it.Body));
        Assert.Equal(2.5, all[0].Distance, 0.01);
        Assert.Equal(-1.0, all[1].Normal[0], 0.01);
    }

    [Fact]
    public void CastAllKeepsTheNearestWhenTheLimitIsShort()
    {
        Row();

        Assert.Equal(new ulong[] { 1, 2 }, AlongRow(limit: 2).Select(it => it.Body));
        Assert.Empty(AlongRow(limit: 0));
    }

    [Fact]
    public void CastAllSkipsWhatItIsToldToIgnoreAndStartsInside()
    {
        Row();

        var inside = AlongRow(x: 3.0);

        Assert.Equal(new ulong[] { 1, 3 }, AlongRow(ignore: 2).Select(it => it.Body));
        Assert.True(inside[0].Started);
        Assert.Equal(0.0, inside[0].Distance);
        Assert.False(inside[1].Started);
    }

    [Fact]
    public void CastAnyAnswersWhetherThereIsAnythingInTheWay()
    {
        Row();
        Trigger(4, [30.0, 0.0, 0.0], half: 0.5);

        bool Any(double distance = 20.0, double x = 0.0, double direction = 1.0, bool triggers = false) =>
            World.CastAny([x, 0.0, 0.0], [direction, 0.0, 0.0], distance, triggers: triggers);

        Assert.True(Any());
        Assert.False(Any(direction: -1.0));
        Assert.False(Any(distance: 2.0));
        Assert.False(Any(x: 25.0));
        Assert.True(Any(x: 25.0, triggers: true));
    }

    [Fact]
    public void CastSeesATriggerOnlyWhenAskedForTriggers()
    {
        Trigger(1, [3.0, 0.0, 0.0], half: 0.5);

        PhysicsHit? Shoot(bool triggers) =>
            World.Cast([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, triggers: triggers);

        Assert.Null(Shoot(triggers: false));
        Assert.Equal(1UL, Shoot(triggers: true)?.Body);
    }

    [Fact]
    public void OverlapWithNoShapeAsksAboutAPoint()
    {
        World.Add(1, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed);
        Trigger(2, [10.0, 0.0, 0.0]);

        Assert.Equal(new ulong[] { 1 }, World.Overlap([0.0, 0.0, 0.0]));
        Assert.Empty(World.Overlap([0.0, 1.2, 0.0]));
        Assert.Empty(World.Overlap([10.0, 0.0, 0.0]));
        Assert.Equal(new ulong[] { 2 }, World.Overlap([10.0, 0.0, 0.0], triggers: true));
    }

    [Fact]
    public void OverlapByAShapeReportsWhatItTouchesUpToALimit()
    {
        World.Add(1, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed);
        World.Add(2, Shape.Sphere(0.5), PhysicsMotion.Fixed, [1.4, 0.0, 0.0]);

        IReadOnlyList<ulong> Around(ulong ignore = 0, int limit = 32) =>
            World.Overlap([1.2, 0.0, 0.0], Shape.Sphere(0.3), ignore: ignore, limit: limit);

        Assert.Equal(new ulong[] { 1, 2 }, Around().Order());
        Assert.Single(Around(limit: 1));
        Assert.Equal(new ulong[] { 2 }, Around(ignore: 1));
        Assert.Empty(Around(limit: 0));
    }

    [Fact]
    public void ARampATriggerAZoneAndABeltAnswerAllFiveQuestions()
    {
        const double angle = 0.5;
        Floor();
        World.Add(
            2,
            Shape.Plane(-Math.Sin(angle), Math.Cos(angle), 0.0, offset: -10.0 * Math.Sin(angle)),
            PhysicsMotion.Fixed);
        Trigger(3, [4.0, 1.0, 0.0]);
        Trigger(4, [-20.0, 2.0, 0.0], half: 2.0);
        World.SetZone(4, new PhysicsZone(Gravity: [0.0, 0.0, 0.0]));
        World.Add(5, Shape.Box(4.0, 0.25, 1.0), PhysicsMotion.Fixed, [-5.0, 0.25, 0.0]);
        World.Add(6, Shape.Box(0.5, 0.5, 0.5), at: [-5.0, 1.0, 0.0]);
        World.Add(7, Shape.Sphere(0.25), at: [-20.0, 2.0, 0.0]);
        World.SetSurface(5, [2.0, 0.0, 0.0]);

        // Point and overlap.
        Assert.Empty(World.Overlap([4.0, 1.0, 0.0]));
        Assert.Equal(new ulong[] { 3 }, World.Overlap([4.0, 1.0, 0.0], triggers: true));
        Assert.Equal(new ulong[] { 4 }, World.Overlap([-20.0, 2.0, 0.0], triggers: true));
        Assert.Equal(new ulong[] { 6 }, World.Overlap([-5.0, 1.0, 0.0], Shape.Sphere(0.1)));

        // Closest, all and any, down onto the ramp, reporting its normal.
        double[] down = [0.0, -1.0, 0.0];
        double[] origin = [13.0, 10.0, 0.0];
        var hit = World.Cast(origin, down, 100.0)!;
        Assert.Equal(2UL, hit.Body);
        Assert.Equal(-Math.Sin(angle), hit.Normal[0], 0.01);
        Assert.Equal(Math.Cos(angle), hit.Normal[1], 0.01);
        var both = World.CastAll(origin, down, 100.0);
        Assert.Equal(new ulong[] { 2, 1 }, both.Select(it => it.Body));
        Assert.Equal(10.0, both[1].Distance, 0.01);
        Assert.False(World.CastAny(origin, down, 5.0));
        Assert.True(World.CastAny(origin, down, 100.0));

        // The world, run: the belt carries its box and the zone holds its ball.
        Run(2.0);
        Assert.Equal(2.0, World.VelocityOf(6)![0], 0.2);
        Assert.Equal(2.0, HeightOf(7), 0.05);
    }

    [Fact]
    public void TheNewEventKindsFollowTheEnginesNumbers()
    {
        Assert.Equal(6U, (uint)PhysicsEventKind.Entered);
        Assert.Equal(7U, (uint)PhysicsEventKind.Exited);
        Assert.Equal(8U, (uint)PhysicsEventKind.TouchStay);
        Assert.Equal(9U, (uint)PhysicsEventKind.Inside);
    }
}
