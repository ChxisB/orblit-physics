namespace Orblit.Physics.Tests;

/// <summary>What the ported joint tests leave unpinned.</summary>
public class PhysicsJointAddedTests : PhysicsTestBase
{
    private static int[] LimitedWays(Joint joint) =>
        joint.Limits.Select((limit, way) => (limit, way)).Where(it => it.limit is not null).Select(it => it.way).ToArray();

    [Fact]
    public void EachFactoryPutsItsLimitOnItsOwnWay()
    {
        var range = new JointLimit(-1.0, 2.0);
        Assert.Equal([3], LimitedWays(Joint.Hinge(range)));
        Assert.Equal([0], LimitedWays(Joint.Slider(range)));
        Assert.Equal([0], LimitedWays(Joint.Distance(range)));
        Assert.Equal([3], LimitedWays(Joint.Cone(0.5, range)));
        Assert.Equal([0], LimitedWays(Joint.Path(range)));
        Assert.Empty(LimitedWays(Joint.Fixed()));
        Assert.Empty(LimitedWays(Joint.Point()));
        Assert.Empty(LimitedWays(Joint.Gear(2.0)));
        Assert.Empty(LimitedWays(Joint.Rack(2.0)));
        Assert.Empty(LimitedWays(Joint.Pulley()));
    }

    [Fact]
    public void EachFactoryKeepsWhatItWasGiven()
    {
        var goal = new JointGoal(0.5, 4.0);
        var springs = new JointSprings(AlongX: new JointSpring(2.0));
        var soft = new JointSoftness(3.0);

        foreach (var joint in new[]
        {
            Joint.Hinge(speed: 1.5, strength: 20.0, goal: goal, springs: springs, softLimit: soft),
            Joint.Slider(speed: 1.5, strength: 20.0, goal: goal, springs: springs, softLimit: soft),
            Joint.Path(speed: 1.5, strength: 20.0, goal: goal, springs: springs, softLimit: soft),
        })
        {
            Assert.Equal(1.5, joint.Speed);
            Assert.Equal(20.0, joint.Strength);
            Assert.Same(goal, joint.Goal);
            Assert.Same(springs, joint.Springs);
            Assert.Same(soft, joint.SoftLimit);
        }

        Assert.Same(springs, Joint.Fixed(springs).Springs);
        Assert.Same(springs, Joint.Point(springs).Springs);
        Assert.Same(springs, Joint.Distance(springs: springs).Springs);
        Assert.Same(soft, Joint.Distance(softLimit: soft).SoftLimit);
        Assert.Same(springs, Joint.Cone(0.5, springs: springs).Springs);
        Assert.Same(soft, Joint.Cone(0.5, softLimit: soft).SoftLimit);
        Assert.Same(springs, Joint.SixAxis(springs: springs).Springs);
        Assert.Same(soft, Joint.SixAxis(softLimit: soft).SoftLimit);
        Assert.Equal(-0.5, Joint.Gear(-0.5).Ratio);
        Assert.Equal(-0.5, Joint.Rack(-0.5).Ratio);
        Assert.Equal(1.0, Joint.Pulley().Ratio);
        Assert.Same(springs, Joint.Gear(1.0, springs).Springs);
        Assert.Same(springs, Joint.Rack(1.0, springs).Springs);
        Assert.Same(springs, Joint.Pulley(springs: springs).Springs);
        Assert.True(Joint.Path(follow: true).Follow);
        Assert.False(Joint.Path().Follow);
        Assert.Same(JointSprings.None, Joint.Fixed().Springs);
    }

    [Fact]
    public void ALockedLimitHoldsOneValue()
    {
        Assert.Equal(new JointLimit(0.0, 0.0), JointLimit.Locked());
        Assert.Equal(new JointLimit(0.3, 0.3), JointLimit.Locked(0.3));
    }

    [Theory]
    [InlineData(0.0, 1.0)]
    [InlineData(-1.0, 1.0)]
    [InlineData(double.NaN, 1.0)]
    [InlineData(1.0, -0.1)]
    [InlineData(1.0, double.NaN)]
    public void ASpringNeedsAPositiveFrequencyAndANonNegativeRatio(double hertz, double ratio)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointSpring(hertz, ratio));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointSoftness(hertz, ratio));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointGoal(0.0, hertz, ratio));
    }

    [Fact]
    public void ASpringMayHaveNoDampingAtAll()
    {
        Assert.Equal(0.0, new JointSpring(1.0, 0.0).Ratio);
        Assert.Equal(0.0, new JointSoftness(1.0, 0.0).Ratio);
        Assert.Equal(0.0, new JointGoal(0.0, 1.0, 0.0).Ratio);
    }

    [Fact]
    public void AConeSwingHoldsTheSecondBodyAtItsWidestAngle()
    {
        World.Add(2, Shape.Sphere(0.1), at: [1.0, 3.0, 0.0]);
        Assert.True(World.Join(1, Joint.Cone(0.3), a: 0, b: 2, at: [0.0, 3.0, 0.0]));

        Run(3.0);
        Assert.Equal(3.0 - Math.Sin(0.3), HeightOf(2), 0.05);
    }

    [Theory]
    [InlineData(5.0, 0.0, true)]
    [InlineData(50.0, 0.0, false)]
    [InlineData(0.0, 5.0, true)]
    [InlineData(0.0, 50.0, false)]
    public void ATorquePastItsLimitBreaksAJointAsAForceDoes(double breakingForce, double breakingTorque, bool breaks)
    {
        // A kilogram a metre out from its weld pulls with 9.81 newtons and turns
        // it with 9.81 newton-metres, so each limit is crossed on its own.
        World.Add(2, Shape.Box(0.1, 0.1, 0.1), at: [1.0, 3.0, 0.0]);
        World.Join(1, Joint.Fixed(), a: 2, at: [0.0, 3.0, 0.0], breakingForce: breakingForce, breakingTorque: breakingTorque);

        Run(0.5);
        Assert.Equal(breaks, World.JointStateOf(1) is null);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TheBodiesOfAJointCollideOnlyWhenItSaysSo(bool collide)
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 5.0, 0.0]);
        World.Add(3, Shape.Sphere(0.1), at: [0.15, 5.0, 0.0]);
        World.Join(1, Joint.Point(), a: 2, b: 3, at: [0.075, 5.0, 0.0], collide: collide);

        var touched = false;
        for (var i = 0; i < 10; i++)
        {
            World.Step(1.0 / 60);
            touched |= World.Events.Any(it => it.Kind == PhysicsEventKind.TouchBegan);
        }

        Assert.Equal(collide, touched);
    }

    [Fact]
    public void AimLeavesAWayWithNoPlaceWhereItWasAndMovesTheOthers()
    {
        World.Add(2, Shape.Box(0.1, 0.1, 0.1), mass: 1.0, at: [0.0, 3.0, 0.0]);
        World.Join(
            1,
            Joint.Slider(springs: new JointSprings(AlongX: new JointSpring(1.0, 1.0, 0.5))),
            a: 2,
            at: [0.0, 3.0, 0.0],
            rotation: [0.0, 0.0, 0.70710678, 0.70710678]);
        const double stretch = 9.81 / (2 * Math.PI * 2 * Math.PI);

        Run(4.0);
        Assert.Equal(0.5 + stretch, World.JointStateOf(1)!.Offset[0], 0.01);

        Assert.True(World.Aim(1, rest: [null, null, null, null, null, null]));
        Assert.True(World.Aim(1, goal: 4.0));
        Run(4.0);
        Assert.Equal(0.5 + stretch, World.JointStateOf(1)!.Offset[0], 0.01);

        Assert.True(World.Aim(1, rest: [1.5, null, null, null, null, null]));
        Run(4.0);
        Assert.Equal(1.5 + stretch, World.JointStateOf(1)!.Offset[0], 0.01);
    }

    [Fact]
    public void APathOfNoPointsIsRefused()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        Assert.False(World.Join(1, Joint.Path(), a: 0, b: 2, at: [0.0, 3.0, 0.0], path: []));
    }

    [Fact]
    public void ADisposedWorldRefusesEveryJointCall()
    {
        var world = new Physics();
        world.Dispose();

        Assert.Throws<InvalidOperationException>(() => world.Join(1, Joint.Point(), a: 2, at: [0.0, 0.0, 0.0]));
        Assert.Throws<InvalidOperationException>(() => world.Unjoin(1));
        Assert.Throws<InvalidOperationException>(() => world.Aim(1, goal: 0.0));
        Assert.Throws<InvalidOperationException>(() => world.JointStateOf(1));
    }

    [Theory]
    [InlineData(false, 0.0)]
    [InlineData(true, -0.23)]
    public void APathTurnsItsBodyToFollowTheLineOnlyWhenAsked(bool follow, double turn)
    {
        World.Add(2, Shape.Box(0.1, 0.1, 0.1), at: [0.0, 3.0, 0.0]);
        World.Join(1, Joint.Path(follow: follow), a: 0, b: 2, at: [0.0, 3.0, 0.0], path: [[0.0, 3.0, 0.0], [3.0, 1.5, 0.0], [6.0, 1.5, 0.0]]);

        Run(0.5);
        // Along the slope the x axis is a turn of 0.4636 about z, whose
        // quaternion has a z part of sin(0.2318).
        Assert.Equal(turn, World.TransformOf(2)![5], 0.04);
    }

    [Fact]
    public void ASpringWithNoDampingKeepsSwinging()
    {
        World.Add(2, Shape.Box(0.1, 0.1, 0.1), mass: 1.0, at: [0.0, 3.0, 0.0]);
        World.Join(
            1,
            Joint.Slider(springs: new JointSprings(AlongX: new JointSpring(1.0, 0.0, 0.5))),
            a: 2,
            at: [0.0, 3.0, 0.0],
            rotation: [0.0, 0.0, 0.70710678, 0.70710678]);

        Run(4.0);
        double low = double.MaxValue, high = double.MinValue;
        for (var i = 0; i < 60; i++)
        {
            World.Step(1.0 / 60);
            low = Math.Min(low, HeightOf(2));
            high = Math.Max(high, HeightOf(2));
        }

        Assert.True(high - low > 0.3, "an undamped spring is still going");
    }

    [Fact]
    public void ADistanceJointWithNoSecondPointIsAZeroLengthRod()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        Assert.True(World.Join(1, Joint.Distance(springs: new JointSprings(AboutY: new JointSpring(2.0))), a: 2, at: [0.0, 3.0, 0.0]));
        Assert.NotNull(World.JointStateOf(1));
    }

    [Fact]
    public void AJointGoesWithItsBodyBeforeAnyStepSaysSo()
    {
        // Each call comes straight after the removal, so it is the one that
        // has to send what waits.
        Assert.False(JointWhoseBodyIsGone().Aim(1, goal: 0.0));
        Assert.False(JointWhoseBodyIsGone().Unjoin(1));
        Assert.Null(JointWhoseBodyIsGone().JointStateOf(1));
    }

    private Physics JointWhoseBodyIsGone()
    {
        Restart();
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        World.Join(1, Joint.Point(new JointSprings(AboutX: new JointSpring(2.0))), a: 2, at: [0.0, 3.0, 0.0]);
        World.Remove(2);
        return World;
    }

    [Fact]
    public void ASoftLimitDampsByItsRatio()
    {
        var loose = PhysicsJointTests.WidestSwing(new JointSoftness(1.0, 0.0));
        var damped = PhysicsJointTests.WidestSwing(new JointSoftness(1.0, 4.0));
        Assert.True(loose > damped + 0.01);
    }

    [Fact]
    public void AimWantsExactlySixPlaces()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        World.Join(1, Joint.Point(), a: 2, at: [0.0, 3.0, 0.0]);

        Assert.Throws<ArgumentException>(() => World.Aim(1, rest: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    }

    [Fact]
    public void APathPointIsExactlyXYAndZ()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);

        Assert.Throws<ArgumentException>(() =>
            World.Join(1, Joint.Path(), a: 0, b: 2, at: [0.0, 3.0, 0.0], path: [[0.0, 3.0, 0.0, 1.0], [1.0, 3.0, 0.0, 1.0]]));
    }
}
