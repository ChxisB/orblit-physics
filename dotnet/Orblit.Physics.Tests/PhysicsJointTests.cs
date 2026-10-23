using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class PhysicsJointTests : PhysicsTestBase
{
    /// <summary>A quarter turn about z, which takes a joint's x axis to straight up.</summary>
    private static readonly double[] Upright = [0.0, 0.0, 0.7071068, 0.7071068];

    /// <summary>A frame whose x axis is the world's z, so a hinge turns a wheel that faces the camera.</summary>
    private static readonly double[] AboutZ = [0.0, -0.70710678, 0.0, 0.70710678];

    /// <summary>A slope of one in two and then a flat, from (0, 3) to (6, 1.5).</summary>
    private static readonly double[][] Way = [[0.0, 3.0, 0.0], [3.0, 1.5, 0.0], [6.0, 1.5, 0.0]];

    /// <summary>
    /// A post that never moves, for a joint's first body, so what is measured
    /// is the second body as the post sees it.
    /// </summary>
    private void Post(ulong id, double[] at) =>
        World.Add(id, Shape.Box(0.1, 0.1, 0.1), PhysicsMotion.Fixed, at);

    private double DistanceFrom(ulong id, double[] point)
    {
        var at = World.TransformOf(id)!;
        var dx = at[0] - point[0];
        var dy = at[1] - point[1];
        var dz = at[2] - point[2];
        return Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
    }

    /// <summary>A wheel on the world with its centre on its axle, so only a motor or a gear turns it.</summary>
    private void Wheel(ulong id, double[] at, double speed = 0.0)
    {
        World.Add(id, Shape.Box(0.3, 0.3, 0.3), at: at);
        World.Join(
            id,
            Joint.Hinge(speed: speed, strength: speed == 0.0 ? 0.0 : 200.0),
            a: 0,
            b: id,
            at: at,
            rotation: AboutZ);
    }

    private void Step(int count)
    {
        for (var i = 0; i < count; i++)
        {
            World.Step(1.0 / 60);
        }
    }

    [Fact]
    public void EachKindSaysWhichOfTheSixWaysItHolds()
    {
        var cone = Joint.Cone(swing: 0.5, twist: new JointLimit(-0.2, 0.2));
        Assert.Equal(PhysicsJointKind.Cone, cone.Kind);
        Assert.Equal(0.5, cone.Swing);
        Assert.Equal(
            new double?[] { null, null, null, 0.2, null, null },
            cone.Limits.Select(limit => limit?.High).ToArray());

        var drawer = Joint.Slider(limit: new JointLimit(0.0, 0.4));
        Assert.Equal(0, drawer.Limits.ToList().FindIndex(limit => limit is not null));

        var rail = Joint.SixAxis(alongY: new JointLimit(-1.0, 1.0), aboutZ: JointLimit.Locked());
        Assert.Equal(
            new[] { true, false, true, true, true, false },
            rail.Limits.Select(limit => limit is null).ToArray());
        Assert.Equal(rail.AboutZ!.Low, rail.AboutZ!.High);
        Assert.Equal(
            new uint[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
            Enum.GetValues<PhysicsJointKind>().Select(kind => (uint)kind).ToArray());
    }

    [Fact]
    public void ItsStructsAreTheSizeTheEngineReads()
    {
        Assert.Equal(280, Marshal.SizeOf<NativeJoint>());
        Assert.Equal(32, Marshal.SizeOf<NativeJointState>());
    }

    [Fact]
    public void APendulumSwingsAndKeepsItsLength()
    {
        World.Add(2, Shape.Sphere(0.1), at: [1.0, 3.0, 0.0]);
        Assert.True(World.Join(1, Joint.Point(), a: 2, at: [0.0, 3.0, 0.0]));

        var lowest = 3.0;
        var longest = 0.0;
        for (var i = 0; i < 120; i++)
        {
            World.Step(1.0 / 60);
            lowest = Math.Min(lowest, HeightOf(2));
            longest = Math.Max(longest, DistanceFrom(2, [0.0, 3.0, 0.0]));
        }

        Assert.True(lowest < 2.1, "it swung down");
        Assert.Equal(1.0, longest, 0.02);
    }

    [Fact]
    public void AHingeTurnsUnderItsMotorAndStopsAtItsLimit()
    {
        Post(1, [0.0, 1.0, 0.0]);
        World.Add(2, Shape.Box(0.5, 1.0, 0.05), at: [0.5, 1.0, 0.0]);
        World.Join(
            1,
            Joint.Hinge(limit: new JointLimit(0.0, 1.2), speed: 1.0, strength: 50.0),
            a: 1,
            b: 2,
            at: [0.0, 1.0, 0.0],
            rotation: Upright);

        Run(0.5);
        Assert.Equal(0.5, World.JointStateOf(1)!.Angles[0], 0.05);
        Assert.Equal(1.0, World.VelocityOf(2)![4], 0.05);

        Run(1.5);
        var state = World.JointStateOf(1)!;
        Assert.Equal(1.2, state.Angles[0], 0.03);
        Assert.True(Math.Abs(World.VelocityOf(2)![4]) < 0.05);
        Assert.Equal(1.0, HeightOf(2), 0.01);
        Assert.Equal(9.81, state.Force, 0.5);
    }

    [Fact]
    public void TheWorldMayBeEitherEndAndWhichDecidesTheSense()
    {
        double Turned(ulong a, ulong b)
        {
            Restart();
            World.Add(2, Shape.Box(0.5, 1.0, 0.05), at: [0.5, 1.0, 0.0]);
            World.Join(1, Joint.Hinge(speed: 1.0, strength: 50.0), a: a, b: b, at: [0.0, 1.0, 0.0], rotation: Upright);
            Run(1.0);
            Assert.Equal(1.0, World.JointStateOf(1)!.Angles[0], 0.05);
            return World.TransformOf(2)![2];
        }

        Assert.True(Turned(0, 2) < -0.3, "the door's own angle");
        Assert.True(Turned(2, 0) > 0.3, "the world's, reversed");
    }

    [Fact]
    public void ASliderKeepsToItsRailAsFarAsItsLimit()
    {
        Post(1, [0.0, 3.0, 0.0]);
        World.Add(2, Shape.Box(0.2, 0.2, 0.2), at: [0.0, 3.0, 0.0]);
        World.Join(
            1,
            Joint.Slider(limit: new JointLimit(0.0, 0.5), speed: 1.0, strength: 100.0),
            a: 1,
            b: 2,
            at: [0.0, 3.0, 0.0]);

        Run(0.25);
        Assert.Equal(0.25, World.JointStateOf(1)!.Offset[0], 0.03);

        Run(1.0);
        var at = World.TransformOf(2)!;
        Assert.Equal(0.5, at[0], 0.01);
        Assert.Equal(3.0, at[1], 0.01);
        Assert.Equal(0.0, at[2], 0.01);
    }

    [Fact]
    public void ASixAxisJointLimitsEachWayInTheOrderItNamesThem()
    {
        Post(1, [0.0, 3.0, 0.0]);
        World.Add(2, Shape.Box(0.2, 0.2, 0.2), at: [0.0, 3.0, 0.0]);
        World.Join(
            1,
            Joint.SixAxis(
                alongX: JointLimit.Locked(),
                alongY: new JointLimit(-0.5, 0.0),
                alongZ: JointLimit.Locked(),
                aboutX: JointLimit.Locked(),
                aboutY: JointLimit.Locked(),
                aboutZ: JointLimit.Locked()),
            a: 1,
            b: 2,
            at: [0.0, 3.0, 0.0]);
        World.Drive(2, [1.0, 0.0, 1.0]);

        Run(1.0);
        var at = World.TransformOf(2)!;
        Assert.Equal(2.5, at[1], 0.02);
        Assert.Equal(0.0, at[0], 0.01);
        Assert.Equal(0.0, at[2], 0.01);
        Assert.Equal(-0.5, World.JointStateOf(1)!.Offset[1], 0.02);
    }

    [Fact]
    public void ARopeIsSlackUntilItIsTaut()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        World.Join(1, Joint.Distance(new JointLimit(0.0, 3.0)), a: 2, at: [0.0, 3.0, 0.0], to: [0.0, 5.0, 0.0]);

        Run(0.3);
        // Two metres of a three metre rope does not hold anything.
        Assert.Equal(-9.81 * 0.3, FallSpeedOf(2), 0.1);

        Run(3.0);
        Assert.Equal(2.0, HeightOf(2), 0.03);
        Assert.Equal(3.0, World.JointStateOf(1)!.Offset[0], 0.03);
    }

    [Fact]
    public void AJointBreaksPastItsStrengthAndSaysSo()
    {
        World.Add(2, Shape.Box(0.2, 0.2, 0.2), at: [0.0, 3.0, 0.0]);
        World.Join(7, Joint.Fixed(), a: 2, at: [0.0, 3.0, 0.0], breakingForce: 5.0);

        World.Step(1.0 / 60);
        var broke = World.Events.Where(it => it.Kind == PhysicsEventKind.Broke).ToList();
        var event_ = Assert.Single(broke);
        Assert.Equal(7UL, event_.A);
        Assert.Equal(0UL, event_.B);
        Assert.True(event_.Force > 5.0);
        Assert.Equal(3.0, event_.At[1], 0.01);
        Assert.Null(World.JointStateOf(7));

        Run(0.5);
        Assert.True(HeightOf(2) < 2.0, "nothing holds it now");
    }

    [Fact]
    public void AChainSleepsAsOneAndLosesItsJointsWithABody()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 2.0, 0.0]);
        World.Add(3, Shape.Sphere(0.1), at: [0.0, 1.0, 0.0]);
        World.Join(1, Joint.Point(), a: 2, at: [0.0, 3.0, 0.0]);
        World.Join(2, Joint.Point(), a: 3, b: 2, at: [0.0, 1.5, 0.0]);

        Run(1.5);
        Assert.True(World.Asleep(2));
        Assert.True(World.Asleep(3));

        World.Remove(2);
        Assert.Null(World.JointStateOf(1));
        Assert.Null(World.JointStateOf(2));
        Assert.False(World.Asleep(3), "what it held is woken");
        Assert.DoesNotContain(PhysicsEventKind.Broke, World.Events.Select(it => it.Kind));

        Run(0.5);
        Assert.True(HeightOf(3) < 0.5);
    }

    [Fact]
    public void UnjoiningLetsGo()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 2.0, 0.0]);
        World.Join(1, Joint.Fixed(), a: 2, at: [0.0, 2.0, 0.0]);
        Run(1.0);
        Assert.Equal(2.0, HeightOf(2), 0.01);
        Assert.True(World.Asleep(2));

        Assert.True(World.Unjoin(1));
        Assert.False(World.Asleep(2));
        Assert.False(World.Unjoin(1));
        Run(0.5);
        Assert.True(HeightOf(2) < 1.0);
    }

    [Fact]
    public void MorePassesOverTheJointsHoldAHeavyTipNearerItsWeld()
    {
        // How far the root of a chain of three welds sags with ten kilograms on
        // the tip, in a world that makes this many passes over its joints.
        static double SagWith(uint passes)
        {
            using var world = new Physics(new PhysicsSettings { JointSteps = passes });
            for (ulong i = 0; i < 3; i++)
            {
                world.Add(10 + i, Shape.Box(0.2, 0.2, 0.2), mass: i == 2 ? 10.0 : 1.0, at: [0.5 * (i + 1), 3.0, 0.0]);
            }

            for (ulong i = 0; i < 3; i++)
            {
                world.Join(100 + i, Joint.Fixed(), a: i == 0 ? 10 : 10 + i - 1, b: i == 0 ? 0 : 10 + i, at: [0.5 * i, 3.0, 0.0]);
            }

            for (var i = 0; i < 240; i++)
            {
                world.Step(1.0 / 60);
            }

            var root = world.TransformOf(10)!;
            var dx = root[0] - 0.5;
            var dy = root[1] - 3.0;
            return Math.Sqrt((dx * dx) + (dy * dy));
        }

        var one = SagWith(1);
        var eight = SagWith(8);
        Assert.True(eight < 0.003);
        Assert.True(eight < one);
    }

    [Fact]
    public void ABoxOnASpringSettlesAStretchOfGOverOmegaSquared()
    {
        using var world = new Physics();
        world.Add(2, Shape.Box(0.1, 0.1, 0.1), mass: 1.0, at: [0.0, 3.0, 0.0]);
        // The slider's frame points up and the box is a, so the offset it reads
        // grows as the box falls.
        world.Join(
            1,
            Joint.Slider(springs: new JointSprings(AlongX: new JointSpring(1.0, 1.0, 0.5))),
            a: 2,
            at: [0.0, 3.0, 0.0],
            rotation: [0.0, 0.0, 0.70710678, 0.70710678]);
        for (var i = 0; i < 240; i++)
        {
            world.Step(1.0 / 60);
        }

        const double omega = 2 * Math.PI;
        Assert.Equal(3.0 - 0.5 - (9.81 / (omega * omega)), world.TransformOf(2)![1], 0.01);
    }

    [Fact]
    public void AMotorWithAGoalHoldsAHingeAtAnAngleAgainstALoad()
    {
        using var world = new Physics();
        world.Add(2, Shape.Box(0.25, 0.25, 0.25), mass: 2.0, at: [0.0, 3.0, 0.5]);
        // The world is a, so the angle is the arm's own, and a positive turn
        // takes its far end down.
        world.Join(1, Joint.Hinge(strength: 50.0, goal: new JointGoal(-0.5, 4.0)), a: 0, b: 2, at: [0.0, 3.0, 0.0]);
        for (var i = 0; i < 180; i++)
        {
            world.Step(1.0 / 60);
        }

        var state = world.JointStateOf(1)!;
        Assert.Equal(-0.5, state.Angles[0], 0.06);
        Assert.True(state.Torque <= 51.0);
        Assert.True(world.TransformOf(2)![1] > 3.0);
    }

    [Fact]
    public void AimMovesAGoalAndASpringWithoutMakingTheJointAgain()
    {
        using var world = new Physics();
        world.Add(2, Shape.Box(0.25, 0.25, 0.25), mass: 2.0, at: [0.0, 3.0, 0.5]);
        world.Join(1, Joint.Hinge(strength: 50.0, goal: new JointGoal(-0.5, 4.0)), a: 0, b: 2, at: [0.0, 3.0, 0.0]);

        void Seconds(double seconds)
        {
            for (var i = 0; i < Math.Round(seconds * 60, MidpointRounding.AwayFromZero); i++)
            {
                world.Step(1.0 / 60);
            }
        }

        Seconds(2);
        Assert.Equal(-0.5, world.JointStateOf(1)!.Angles[0], 0.06);

        Assert.True(world.Aim(1, goal: 0.5));
        Seconds(2);
        Assert.Equal(0.5, world.JointStateOf(1)!.Angles[0], 0.06);

        Assert.True(world.Aim(1, rest: [null, 1.0, null, null, null, null]));
        Seconds(1);
        Assert.Equal(0.5, world.JointStateOf(1)!.Angles[0], 0.06);
    }

    [Fact]
    public void AimRefusesAJointThatIsNotThereAndAPlaceAtInfinity()
    {
        using var world = new Physics();
        world.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        world.Join(1, Joint.Point(new JointSprings(AboutX: new JointSpring(2.0))), a: 2, at: [0.0, 3.0, 0.0]);

        Assert.False(world.Aim(9, goal: 0.0));
        Assert.False(world.Aim(1, goal: double.PositiveInfinity));
        Assert.False(world.Aim(1, rest: [0.0, 0.0, 0.0, double.PositiveInfinity, 0.0, 0.0]));
        Assert.Throws<ArgumentException>(() => world.Aim(1, rest: [0.0, 0.0]));
    }

    [Fact]
    public void ASpringOnADistanceJointRestsAtTheLengthItWasMade()
    {
        using var world = new Physics();
        world.Add(2, Shape.Sphere(0.1), mass: 1.0, at: [0.0, 2.0, 0.0]);
        world.Join(
            1,
            Joint.Distance(springs: new JointSprings(AlongX: new JointSpring(2.0))),
            a: 2,
            at: [0.0, 2.0, 0.0],
            to: [0.0, 3.0, 0.0]);
        for (var i = 0; i < 240; i++)
        {
            world.Step(1.0 / 60);
        }

        const double omega = 4 * Math.PI;
        Assert.Equal(1.0 + (9.81 / (omega * omega)), world.JointStateOf(1)!.Offset[0], 0.01);
    }

    /// <summary>
    /// The widest angle a hinge swings to, driven hard into a limit of half a
    /// radian that may be soft.
    /// </summary>
    internal static double WidestSwing(JointSoftness? soft)
    {
        using var world = new Physics();
        world.Add(2, Shape.Box(0.5, 1.0, 0.05), at: [0.5, 1.0, 0.0]);
        world.Join(
            1,
            Joint.Hinge(limit: new JointLimit(-0.5, 0.5), softLimit: soft),
            a: 2,
            at: [0.0, 1.0, 0.0],
            rotation: [0.0, 0.0, 0.70710678, 0.70710678]);
        world.Drive(2, spin: [0.0, -6.0, 0.0]);
        var most = 0.0;
        for (var i = 0; i < 240; i++)
        {
            world.Step(1.0 / 60);
            most = Math.Max(most, world.JointStateOf(1)!.Angles[0]);
        }

        return most;
    }

    [Fact]
    public void ASoftLimitGivesPastItsEndAndBringsTheJointBack()
    {
        var hard = WidestSwing(null);
        var soft = WidestSwing(new JointSoftness(1.0));
        Assert.True(hard < 0.53);
        Assert.True(soft > hard + 0.05);
    }

    [Fact]
    public void AGearTurnsTheSecondWheelAsFarTheOtherWayAsItSays()
    {
        Wheel(2, [0.0, 3.0, 0.0], speed: 4.0);
        Wheel(3, [1.0, 3.0, 0.0]);
        Assert.True(World.Join(9, Joint.Gear(-0.5), a: 2, b: 3, at: [0.5, 3.0, 0.0], rotation: AboutZ));
        Step(30);
        var left = World.JointStateOf(2)!.Angles[0];
        var right = World.JointStateOf(3)!.Angles[0];
        Assert.True(left > 0.8);
        Assert.Equal(-0.5 * left, right, 0.03);

        Step(150);
        // The ratio holds over turns, which a pose cannot count.
        Assert.True(Math.Abs(World.JointStateOf(9)!.Offset[0]) < 0.05);
    }

    [Fact]
    public void WithoutTheGearTheSecondWheelStaysWhereItWas()
    {
        Wheel(2, [0.0, 3.0, 0.0], speed: 4.0);
        Wheel(3, [1.0, 3.0, 0.0]);
        Step(30);
        Assert.True(Math.Abs(World.JointStateOf(3)!.Angles[0]) < 0.01);
    }

    [Fact]
    public void ARackMovesByItsRatioForEachRadianItsPinionTurns()
    {
        World.Add(2, Shape.Box(0.2, 0.2, 0.2), at: [0.0, 2.2, 0.0]);
        World.Add(3, Shape.Box(0.2, 0.2, 0.2), at: [0.0, 3.0, 0.0]);
        World.Join(1, Joint.Slider(), a: 0, b: 2, at: [0.0, 2.2, 0.0]);
        World.Join(2, Joint.Hinge(speed: 2.0, strength: 100.0), a: 0, b: 3, at: [0.0, 3.0, 0.0], rotation: AboutZ);
        Assert.True(World.Join(3, Joint.Rack(0.25), a: 2, b: 3, at: [0.0, 2.6, 0.0], axisB: [0.0, 0.0, 1.0]));
        Step(60);
        var turned = World.JointStateOf(2)!.Angles[0];
        Assert.True(turned > 1.5);
        Assert.Equal(0.25 * turned, World.JointStateOf(1)!.Offset[0], 0.02);
    }

    [Fact]
    public void WeightsOverAPulleyMoveAtAThirdOfGravity()
    {
        World.Add(2, Shape.Box(0.1, 0.1, 0.1), mass: 2.0, at: [-1.0, 1.0, 0.0]);
        World.Add(3, Shape.Box(0.1, 0.1, 0.1), at: [1.0, 1.0, 0.0]);
        Assert.True(World.Join(
            1,
            Joint.Pulley(),
            a: 2,
            b: 3,
            at: [-1.0, 1.0, 0.0],
            to: [1.0, 1.0, 0.0],
            over: [[-1.0, 3.0, 0.0], [1.0, 3.0, 0.0]]));
        Step(30);
        var fall = 1.0 - HeightOf(2);
        Assert.Equal(0.41, fall, 0.06);
        Assert.Equal(fall, HeightOf(3) - 1.0, 0.02);
    }

    private void Ball(Joint joint)
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        Assert.True(World.Join(1, joint, a: 0, b: 2, at: [0.0, 3.0, 0.0], path: Way));
    }

    [Fact]
    public void APathHoldsABallOnItsLineAsItSlidesDown()
    {
        Ball(Joint.Path());
        Step(60);
        var arc = World.JointStateOf(1)!.Offset[0];
        Assert.Equal(2.15, arc, 0.15);
        var at = World.TransformOf(2)!;
        Assert.Equal(0.894427 * arc, at[0], 0.02);
        Assert.Equal(3.0 - (0.447214 * arc), at[1], 0.02);
    }

    [Fact]
    public void APathGoesRoundACorner()
    {
        Ball(Joint.Path());
        Step(96);
        Assert.Equal(1.5, HeightOf(2), 0.02);
    }

    [Fact]
    public void APathStopsAtALimit()
    {
        Ball(Joint.Path(new JointLimit(0.0, 2.0)));
        Step(120);
        Assert.Equal(2.0, World.JointStateOf(1)!.Offset[0], 0.05);
    }

    [Fact]
    public void APathKeepsCountingPastItsEndOnTheLineGoneOnStraight()
    {
        Ball(Joint.Path());
        Step(240);
        Assert.True(World.JointStateOf(1)!.Offset[0] > 7.0);
        Assert.Equal(1.5, World.TransformOf(2)![1], 0.02);
    }

    [Fact]
    public void APathStopsAtALimitSetAtItsEndWhichABallRanOff()
    {
        // The line is 3.354 and then 3 long.
        Ball(Joint.Path(new JointLimit(0.0, 6.354)));
        Step(240);
        Assert.Equal(6.354, World.JointStateOf(1)!.Offset[0], 0.05);
        Assert.Equal(6.0, World.TransformOf(2)![0], 0.05);
    }

    [Fact]
    public void APathIsDrivenAlongByAMotor()
    {
        Ball(Joint.Path(speed: 1.0, strength: 100.0));
        Step(60);
        Assert.Equal(1.0, World.JointStateOf(1)!.Offset[0], 0.1);
    }

    [Fact]
    public void APathIsRefusedWithoutALineToFollow()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        bool Path(IReadOnlyList<IReadOnlyList<double>>? points) =>
            World.Join(1, Joint.Path(), a: 0, b: 2, at: [0.0, 3.0, 0.0], path: points);

        Assert.False(Path(null));
        Assert.False(Path([[0.0, 3.0, 0.0]]));
        Assert.Throws<ArgumentException>(() => Path([[0.0, 3.0], [1.0, 3.0]]));
        Assert.True(Path(Way));
    }

    [Fact]
    public void ACouplingWithNoRatioIsRefused()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 3.0, 0.0]);
        Assert.False(World.Join(1, Joint.Gear(double.NaN), a: 2, at: [0.0, 3.0, 0.0]));
        Assert.Throws<ArgumentException>(
            () => World.Join(1, Joint.Pulley(), a: 2, at: [0.0, 3.0, 0.0], over: [[0.0, 4.0, 0.0]]));
    }

    [Fact]
    public void TheWorldRefusesAJointItCannotMake()
    {
        World.Add(2, Shape.Sphere(0.1), at: [0.0, 2.0, 0.0]);
        World.Add(3, Shape.Sphere(0.1), at: [1.0, 2.0, 0.0]);
        bool Join(ulong id, ulong a = 2, ulong b = 0, double[]? at = null) =>
            World.Join(id, Joint.Point(), a: a, b: b, at: at ?? [0.0, 2.0, 0.0]);

        Assert.False(Join(0), "zero is never a joint");
        Assert.False(Join(1, a: 9), "no such body");
        Assert.False(Join(1, b: 9), "no such body");
        Assert.False(Join(1, b: 2), "a body joined to itself");
        Assert.False(Join(1, a: 0), "the world joined to itself");
        Assert.False(Join(1, at: [double.NaN, 2.0, 0.0]));
        Assert.True(Join(1));
        Assert.False(Join(1, b: 3), "already a joint");
        Assert.True(Join(3, b: 3), "joints are named apart");
        Assert.Null(World.JointStateOf(99));
        Assert.False(World.Unjoin(99));
    }
}
