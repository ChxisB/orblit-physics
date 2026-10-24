namespace Orblit.Physics.Tests;

public class PhysicsQueryAddedTests : PhysicsTestBase
{
    /// <summary>A quarter turn about z, which takes the x axis to straight up.</summary>
    private static readonly double[] Upright = [0.0, 0.0, 0.7071068, 0.7071068];

    private static readonly Layers Seen = new(Is: 4, Cares: 0);
    private static readonly Layers Unseen = new(Is: 8, Cares: 0);

    private void Trigger(ulong id, double[] at) =>
        World.Add(id, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed, at, trigger: true);

    /// <summary>Body 2 stands in the way of everything the queries below fire, on layer 1 caring about 4.</summary>
    private void Mark() =>
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), PhysicsMotion.Fixed, [5.0, 0.0, 0.0], layers: new Layers(Is: 1, Cares: 4));

    private IReadOnlyList<PhysicsHit> CastAllAt(Layers? layers = null, ulong ignore = 0, bool triggers = false) =>
        World.CastAll([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, layers: layers, ignore: ignore, triggers: triggers);

    private bool CastAnyAt(Layers? layers = null, ulong ignore = 0) =>
        World.CastAny([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, layers: layers, ignore: ignore);

    private IReadOnlyList<ulong> OverlapAt(Layers? layers = null, ulong ignore = 0) =>
        World.Overlap([5.0, 0.0, 0.0], layers: layers, ignore: ignore);

    private double SinkSpeed(double? drag)
    {
        Restart();
        World.Add(1, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, -20.0, 0.0], trigger: true);
        World.SetZone(1, drag is { } held
            ? new PhysicsZone(FluidDensity: 1000.0, FluidDrag: held)
            : new PhysicsZone(FluidDensity: 1000.0));
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, -5.0, 0.0], mass: 2000.0);
        Run(1.5);
        return World.VelocityOf(2)![1];
    }

    /// <summary>
    /// Pushes box 2 at box 3 across smooth ground under the given rule and
    /// answers where box 3 and box 2 end up along x.
    /// </summary>
    private (double Pushed, double Pusher) Shove(Action<Physics>? rule)
    {
        Restart();
        Floor(friction: 0.0);
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 0.5, 0.0], friction: 0.0);
        World.Add(3, Shape.Box(0.5, 0.5, 0.5), at: [2.0, 0.5, 0.0], friction: 0.0);
        rule?.Invoke(World);
        World.Drive(2, [3.0, 0.0, 0.0]);
        Run(1.5);
        return (World.TransformOf(3)![0], World.TransformOf(2)![0]);
    }

    [Fact]
    public void EveryQueryReadsTheLayersItIsGiven()
    {
        Mark();

        Assert.Equal(new ulong[] { 2 }, CastAllAt(Seen).Select(it => it.Body));
        Assert.Empty(CastAllAt(Unseen));
        Assert.True(CastAnyAt(Seen));
        Assert.False(CastAnyAt(Unseen));
        Assert.Equal(new ulong[] { 2 }, OverlapAt(Seen));
        Assert.Empty(OverlapAt(Unseen));
    }

    [Fact]
    public void EveryQuerySkipsTheBodyItIsToldToIgnore()
    {
        Mark();

        Assert.Single(CastAllAt());
        Assert.Empty(CastAllAt(ignore: 2));
        Assert.True(CastAnyAt());
        Assert.False(CastAnyAt(ignore: 2));
        Assert.Single(OverlapAt());
        Assert.Empty(OverlapAt(ignore: 2));
    }

    [Fact]
    public void CastAllSeesATriggerOnlyWhenAskedForTriggers()
    {
        Trigger(1, [5.0, 0.0, 0.0]);

        Assert.Empty(CastAllAt());
        Assert.Equal(new ulong[] { 1 }, CastAllAt(triggers: true).Select(it => it.Body));
    }

    [Fact]
    public void CastAllAndCastAnyTurnTheirShapeByTheRotationTheyAreGiven()
    {
        World.Add(2, Shape.Sphere(0.25), PhysicsMotion.Fixed, [5.0, 1.5, 0.0]);
        var tall = Shape.Box(0.1, 2.0, 0.1);
        double[] along = [1.0, 0.0, 0.0];
        double[] origin = [0.0, 0.0, 0.0];

        Assert.Single(World.CastAll(origin, along, 10.0, tall));
        Assert.True(World.CastAny(origin, along, 10.0, tall));
        Assert.Empty(World.CastAll(origin, along, 10.0, tall, Upright));
        Assert.False(World.CastAny(origin, along, 10.0, tall, Upright));
    }

    [Fact]
    public void OverlapTurnsItsShapeByTheRotationItIsGiven()
    {
        World.Add(2, Shape.Sphere(0.25), PhysicsMotion.Fixed, [0.0, 1.5, 0.0]);
        var wide = Shape.Box(2.0, 0.1, 0.1);

        Assert.Empty(World.Overlap([0.0, 0.0, 0.0], wide));
        Assert.Equal(new ulong[] { 2 }, World.Overlap([0.0, 0.0, 0.0], wide, Upright));
    }

    [Fact]
    public void AZoneOfGravityAloneLeavesDampingAlone()
    {
        World.Add(1, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0], linearDamping: 0.0);
        World.SetZone(1, new PhysicsZone(Gravity: [0.0, 0.0, 0.0]));
        World.Drive(2, [5.0, 0.0, 0.0]);

        Run(1.0);

        Assert.Equal(5.0, World.VelocityOf(2)![0], 0.1);
    }

    [Fact]
    public void AZoneOfAngularDampingSlowsTheSpinAndNotTheSpeed()
    {
        World.Add(1, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0], linearDamping: 0.0, angularDamping: 0.0);
        World.SetZone(1, new PhysicsZone(AngularDamping: 8.0));
        World.Drive(2, [5.0, 0.0, 0.0], [0.0, 6.0, 0.0]);

        Run(1.0);

        var velocity = World.VelocityOf(2)!;
        Assert.Equal(5.0, velocity[0], 0.1);
        Assert.True(velocity[4] < 0.1);
    }

    [Fact]
    public void AFluidsDragSetsTheSpeedABodySinksAt()
    {
        var slowed = SinkSpeed(3.0);
        var free = SinkSpeed(null);

        Assert.InRange(slowed, -2.0, -1.0);
        Assert.True(free < -5.0);
    }

    [Fact]
    public void AZoneWithNoPriorityYieldsToOneWithPriorityOneAtAHigherId()
    {
        World.Add(1, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.Add(3, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0]);
        World.SetZone(1, new PhysicsZone(Gravity: [0.0, 0.0, 0.0]));
        World.SetZone(3, new PhysicsZone(Gravity: [0.0, 9.81, 0.0], Priority: 1));
        Run(1.0);

        Assert.True(HeightOf(2) > 12.0);
    }

    [Fact]
    public void AZoneIsReplacedByTheNextOneSetOnTheSameTrigger()
    {
        World.Add(1, Shape.Box(20.0, 20.0, 20.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.Add(2, Shape.Sphere(0.25), at: [0.0, 10.0, 0.0]);

        Assert.True(World.SetZone(1, new PhysicsZone(Gravity: [0.0, 0.0, 0.0])));
        Assert.True(World.SetZone(1, new PhysicsZone(Gravity: [0.0, 9.81, 0.0])));
        Run(1.0);

        Assert.True(HeightOf(2) > 10.5);
        Assert.False(World.RemoveZone(99));
    }

    [Fact]
    public void ARuleOfRestitutionAloneMakesABallBounce()
    {
        double Height(PhysicsRule? rule)
        {
            Restart();
            Floor();
            World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);
            if (rule is not null)
            {
                World.SetRule(2, Ground, rule);
            }

            Run(1.0);
            return HeightOf(2);
        }

        Assert.True(Height(null) < 0.6);
        Assert.True(Height(new PhysicsRule(restitution: 0.9)) > 1.0);
    }

    [Fact]
    public void AnIgnoringRuleLetsABodyFallThroughWhatItNamesAndNoOtherBody()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 2.0, 0.0]);
        World.Add(3, Shape.Sphere(0.5), at: [5.0, 2.0, 0.0]);

        Assert.True(World.SetRule(2, Ground, new PhysicsRule(ignore: true)));
        Run(2.0);

        Assert.True(HeightOf(2) < -1.0);
        Assert.Equal(0.5, HeightOf(3), 0.05);
    }

    [Fact]
    public void AMoveScaleFollowsTheNamesInTheOrderTheyAreGiven()
    {
        var (freePushed, freePusher) = Shove(null);
        var (heldB, _) = Shove(world => world.SetRule(2, 3, new PhysicsRule(moveScaleB: 0.0)));
        var (heldA, _) = Shove(world => world.SetRule(3, 2, new PhysicsRule(moveScaleA: 0.0)));
        var (_, carrying) = Shove(world => world.SetRule(2, 3, new PhysicsRule(moveScaleA: 0.0)));

        Assert.True(freePushed > 2.5);
        Assert.Equal(2.0, heldB, 0.05);
        Assert.Equal(2.0, heldA, 0.05);
        Assert.True(carrying > freePusher + 0.5);
    }

    [Fact]
    public void AOneWayRuleLetsABodyThrownUpPassAndHoldsItComingDown()
    {
        double LandsAt(bool oneWay)
        {
            Restart();
            Floor();
            World.Add(3, Shape.Box(2.0, 0.1, 2.0), PhysicsMotion.Fixed, [0.0, 2.0, 0.0]);
            World.Add(2, Shape.Sphere(0.25), at: [0.0, 0.5, 0.0]);
            if (oneWay)
            {
                Assert.True(World.SetRule(3, 2, new PhysicsRule(oneWay: [0.0, 1.0, 0.0])));
            }

            World.Drive(2, [0.0, 9.0, 0.0]);
            Run(4.0);
            return HeightOf(2);
        }

        Assert.Equal(0.25, LandsAt(oneWay: false), 0.05);
        Assert.Equal(2.35, LandsAt(oneWay: true), 0.05);
    }

    [Fact]
    public void ABeltCarriesABoxAlongZAsWellAsX()
    {
        World.Add(1, Shape.Box(1.0, 0.25, 8.0), PhysicsMotion.Fixed, [0.0, 0.25, 0.0]);
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 1.0, 0.0]);

        World.SetSurface(1, [0.0, 0.0, 2.0]);
        Run(2.0);

        Assert.Equal(2.0, World.VelocityOf(2)![2], 0.2);
        Assert.Equal(0.0, World.VelocityOf(2)![0], 0.1);
    }

    [Fact]
    public void ARuleThatChangesNothingIsNotARule()
    {
        Assert.Throws<ArgumentException>(() => new PhysicsRule());
        Assert.Throws<ArgumentException>(() => new PhysicsRule(moveScaleA: 1.0, moveScaleB: 1.0, ignore: false));
    }

    [Fact]
    public void AnyOneChangeMakesARule()
    {
        PhysicsRule[] rules =
        [
            new(friction: 0.0),
            new(restitution: 0.0),
            new(moveScaleA: 2.0),
            new(moveScaleB: 0.5),
            new(ignore: true),
            new(oneWay: [0.0, 1.0, 0.0]),
        ];

        Assert.Equal(6, rules.Length);
        Assert.Equal(0.0, rules[0].Friction);
        Assert.Equal(2.0, rules[2].MoveScaleA);
        Assert.Equal(0.5, rules[3].MoveScaleB);
        Assert.True(rules[4].Ignore);
        Assert.Equal(new[] { 0.0, 1.0, 0.0 }, rules[5].OneWay);
    }

    [Fact]
    public void ADisposedWorldRefusesZonesRulesAndTheMoreCasts()
    {
        var gone = new Physics();
        gone.Dispose();

        Assert.Throws<InvalidOperationException>(() => gone.SetZone(1, new PhysicsZone()));
        Assert.Throws<InvalidOperationException>(() => gone.RemoveZone(1));
        Assert.Throws<InvalidOperationException>(() => gone.SetRule(1, 2, new PhysicsRule(ignore: true)));
        Assert.Throws<InvalidOperationException>(() => gone.RemoveRule(1, 2));
        Assert.Throws<InvalidOperationException>(() => gone.SetSurface(1, [0.0, 0.0, 0.0]));
        Assert.Throws<InvalidOperationException>(() => gone.CastAll([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0));
        Assert.Throws<InvalidOperationException>(() => gone.CastAny([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0));
        Assert.Throws<InvalidOperationException>(() => gone.Overlap([0.0, 0.0, 0.0]));
    }
}
