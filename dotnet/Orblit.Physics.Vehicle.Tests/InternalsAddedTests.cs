using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// The parts of a car the public type hides, asked one at a time.
/// </summary>
public class InternalsAddedTests
{
    private const double Dt = 1.0 / 60;

    private static WheelSpec Wheel(double x, double z, bool steers = false, bool drives = false) =>
        new(new Vector3(x, 0, z), 0.35, steers: steers, drives: drives);

    /// <summary>Front left, rear left, front right, rear right. The front is along negative z.</summary>
    private static WheelSpec[] Four(bool frontSteers = true, params int[] driven) =>
        [
            Wheel(-0.8, -1.2, frontSteers, driven.Contains(0)),
            Wheel(-0.8, 1.2, false, driven.Contains(1)),
            Wheel(0.8, -1.2, frontSteers, driven.Contains(2)),
            Wheel(0.8, 1.2, false, driven.Contains(3)),
        ];

    private static void AssertVector(Vector3 expected, Vector3 actual, int precision = 9)
    {
        Assert.Equal(expected.X, actual.X, precision);
        Assert.Equal(expected.Y, actual.Y, precision);
        Assert.Equal(expected.Z, actual.Z, precision);
    }

    [Fact]
    public void SteeringTurnsTheInnerWheelFurtherThanTheOuter()
    {
        var left = Steering.Angles(Four(), 0.3);
        var right = Steering.Angles(Four(), -0.3);

        Assert.Equal(new[] { 0.332123793, 0.0, 0.273399437, 0.0 }, left.Select(angle => Math.Round(angle, 9)));
        Assert.Equal(new[] { -0.273399437, 0.0, -0.332123793, 0.0 }, right.Select(angle => Math.Round(angle, 9)));
    }

    [Fact]
    public void SteeringAtZeroTurnsNothing() =>
        Assert.All(Steering.Angles(Four(), 0.0), angle => Assert.Equal(0.0, angle));

    [Fact]
    public void SteeringACarWhoseRearSteersMirrorsTheFront()
    {
        var wheels = new[]
        {
            Wheel(-0.8, -1.2),
            Wheel(-0.8, 1.2, steers: true),
            Wheel(0.8, -1.2),
            Wheel(0.8, 1.2, steers: true),
        };

        var angles = Steering.Angles(wheels, 0.3);

        Assert.Equal(new[] { 0.0, 0.273399437, 0.0, 0.332123793 }, angles.Select(angle => Math.Round(angle, 9)));
    }

    [Fact]
    public void SteeringGivesEveryWheelTheSameAngleWhenNoneIsFixed()
    {
        var wheels = new[] { Wheel(-0.8, -1.2, steers: true), Wheel(0.8, -1.2, steers: true) };

        Assert.Equal(new[] { 0.3, 0.3 }, Steering.Angles(wheels, 0.3));
    }

    [Fact]
    public void SteeringAnAllFixedCarTurnsNothing() =>
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 0.0 }, Steering.Angles(Four(frontSteers: false), 0.3));

    [Fact]
    public void SteeringGivesTheAngleWhenTheSteeredAxleSitsOnTheFixedOne()
    {
        var wheels = new[]
        {
            Wheel(-0.8, 0.0000001, steers: true),
            Wheel(0.8, 0.0000001, steers: true),
            Wheel(0.0, 0.0),
        };

        Assert.Equal(new[] { 0.3, 0.3, 0.0 }, Steering.Angles(wheels, 0.3));
    }

    [Fact]
    public void SteeringKeepsTheGeometryOnceTheAxlesAreMoreThanAMicrometreApart()
    {
        var wheels = new[]
        {
            Wheel(-0.8, -0.000002, steers: true),
            Wheel(0.8, -0.000002, steers: true),
            Wheel(0.0, 0.0),
        };

        var angles = Steering.Angles(wheels, 0.3);

        Assert.InRange(Math.Abs(angles[0]), 0.0, 0.00001);
        Assert.InRange(Math.Abs(angles[1]), 0.0, 0.00001);
    }

    [Fact]
    public void GroundOfAPatchFindsTheWayAlongAndAcross()
    {
        var patch = new Patch(
            new Vector3(2, 3, -5),
            new Vector3(0, 1, 0),
            new Vector3(0, 0, -3),
            Load: 1,
            Surface: 1,
            Mass: 1);

        var ground = Ground.Of(patch)!;

        AssertVector(new Vector3(0, 0, -1), ground.Heading, 12);
        AssertVector(new Vector3(-1, 0, 0), ground.Side, 12);
        Assert.Equal(5, ground.Along, 12);
        Assert.Equal(-2, ground.Across, 12);
        Assert.Equal(Math.Sqrt(29), ground.Speed, 12);
    }

    [Fact]
    public void GroundOfAPatchDropsTheHeadingsPartAlongTheNormal()
    {
        var patch = new Patch(
            new Vector3(0, 0, -4),
            new Vector3(0, 1, 0),
            new Vector3(0, 1, -1),
            Load: 1,
            Surface: 1,
            Mass: 1);

        var ground = Ground.Of(patch)!;

        AssertVector(new Vector3(0, 0, -1), ground.Heading, 12);
        Assert.Equal(4, ground.Along, 12);
    }

    [Fact]
    public void GroundOfAHeadingWithNothingLeftAlongTheSurfaceIsNothing()
    {
        Assert.Null(Ground.Of(Road(heading: new Vector3(0, 1, 0))));
        Assert.Null(Ground.Of(Road(heading: new Vector3(0.0009, 0, 0))));
        Assert.Null(Ground.Of(Road(heading: new Vector3(0, 0, 0))));
        AssertVector(new Vector3(1, 0, 0), Ground.Of(Road(heading: new Vector3(0.0011, 0, 0)))!.Heading, 12);
    }

    [Fact]
    public void BristlesWorkOutTheForceAndTheBendFromTheirStiffness()
    {
        var bristles = new Bristles(Stiffness: 100, Keep: 0.5, Dt: 0.1);

        Assert.Equal(-25.0 / 1.625, bristles.Force(0, 1, 4), 9);
        Assert.Equal(-11.0 / 1.625, bristles.Force(0.01, 0.5, 4), 9);
        Assert.Equal(0.019, bristles.Bent(0.02, -1.5, 4), 12);
    }

    [Fact]
    public void ATyreWithNoRoadSpinsUpByItsDriveAndSlowsByItsBrake()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(null, 0.1, drive: 10, brake: 0);
        Assert.Equal(1.0, tyre.Spin);
        Assert.Equal(0.0, tyre.Force.Length);

        tyre.Step(null, 0.1, drive: 0, brake: 5);
        Assert.Equal(0.5, tyre.Spin);

        tyre.Step(null, 0.1, drive: 0, brake: 5);
        Assert.Equal(0.0, tyre.Spin);
    }

    [Fact]
    public void ATyreWithNoRoadSlowsAReversedWheelTheOtherWay()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(null, 0.1, drive: -10, brake: 0);
        tyre.Step(null, 0.1, drive: 0, brake: 5);

        Assert.Equal(-0.5, tyre.Spin);
    }

    [Fact]
    public void ATyreSpinsUpInProportionToItsInertia()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 2.0);

        tyre.Step(null, 0.1, drive: 10, brake: 0);

        Assert.Equal(0.5, tyre.Spin);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-5.0)]
    [InlineData(double.NaN)]
    public void ATyreWithNoLoadHasNoRoad(double load)
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0, 0, -10), load: load), Dt, drive: 0, brake: 0);

        Assert.Equal(0.0, tyre.Force.Length);
        Assert.Equal(0.0, tyre.Spin);
    }

    [Fact]
    public void ATyreWithAHeadingThatPointsStraightAtTheNormalHasNoRoad()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0, 0, -10), heading: new Vector3(0, 1, 0)), Dt, drive: 0, brake: 0);

        Assert.Equal(0.0, tyre.Force.Length);
    }

    [Fact]
    public void ATyreOnTheRoadDragsBackAndSpinsUpToMatchIt()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0, 0, -10)), Dt, drive: 0, brake: 0);

        Assert.True(tyre.Force.Z > 0, $"{tyre.Force.Z} does not drag against the way the road goes.");
        Assert.Equal(0.0, tyre.Force.X, 9);
        Assert.Equal(0.0, tyre.Force.Y, 9);
        Assert.True(tyre.Spin > 0);
    }

    [Theory]
    [InlineData(1.0, 1.0, 0.8, 3200.0)]
    [InlineData(1.0, 0.5, 0.8, 1600.0)]
    [InlineData(2.0, 1.0, 0.8, 6400.0)]
    [InlineData(1.0, 1.0, 0.5, 2000.0)]
    public void ATyreThatSlidesHoldsAtTheSlidingFractionOfLoadTimesGripTimesSurface(
        double grip,
        double surface,
        double sliding,
        double expected)
    {
        var tyre = new Tyre(new TyreSpec(grip: grip, sliding: sliding), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0, 0, -50), surface: surface), Dt, drive: 0, brake: 0);

        Assert.Equal(expected, tyre.Force.Length, 6);
        Assert.Equal(expected, tyre.Force.Z, 6);
    }

    [Fact]
    public void ATyreThatSlidesSidewaysPushesBackAgainstTheSlide()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(50, 0, 0)), Dt, drive: 0, brake: 0);

        Assert.Equal(-3200.0, tyre.Force.X, 6);
        Assert.Equal(0.0, tyre.Force.Z, 9);
    }

    [Fact]
    public void ATyreWithItsBrakeOnHardLocksAndStillHoldsTheRoad()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0, 0, -50)), Dt, drive: 0, brake: 1e6);

        Assert.Equal(0.0, tyre.Spin);
        Assert.Equal(3200.0, tyre.Force.Length, 6);
    }

    [Fact]
    public void ATyreWithAGentleBrakeLosesJustTheImpulseOfIt()
    {
        var free = new Tyre(new TyreSpec(), 0.35, 1.0);
        var braked = new Tyre(new TyreSpec(), 0.35, 1.0);
        var patch = Road(velocity: new Vector3(0, 0, -10));

        free.Step(patch, Dt, drive: 0, brake: 0);
        braked.Step(patch, Dt, drive: 0, brake: 1);

        Assert.Equal(free.Spin - Dt, braked.Spin, 12);
    }

    [Fact]
    public void ATyresSpinAfterIsAQueryThatAgreesWithTheStepThatFollows()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);
        var patch = Road(velocity: new Vector3(0, 0, -10));

        var after = tyre.SpinAfter(patch, Dt, drive: 20, brake: 0);

        Assert.Equal(0.0, tyre.Spin);
        Assert.Equal(0.0, tyre.Force.Length);
        tyre.Step(patch, Dt, drive: 20, brake: 0);
        Assert.Equal(after, tyre.Spin);
    }

    [Fact]
    public void ATyreRemembersTheBendOfItsBristlesFromOneStepToTheNext()
    {
        var fresh = new Tyre(new TyreSpec(), 0.35, 1.0);
        var used = new Tyre(new TyreSpec(), 0.35, 1.0);
        var patch = Road(velocity: new Vector3(0, 0, -2));

        fresh.Step(patch, Dt, drive: 0, brake: 0);
        used.Step(patch, Dt, drive: 0, brake: 0);
        used.Step(patch, Dt, drive: 0, brake: 0);

        Assert.NotEqual(fresh.Force.Z, used.Force.Z);
    }

    [Fact]
    public void ATyreForgetsTheBendOnceItLeavesTheRoad()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);
        var patch = Road(velocity: new Vector3(0, 0, -2));
        var fresh = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(patch, Dt, drive: 0, brake: 0);
        tyre.Step(null, Dt, drive: 0, brake: 1e6);
        tyre.Step(patch, Dt, drive: 0, brake: 0);
        fresh.Step(patch, Dt, drive: 0, brake: 0);

        Assert.Equal(0.0, fresh.Force.Z - tyre.Force.Z, 12);
    }

    [Fact]
    public void ATyreWithNoRoadSlowsAHeavyWheelByLessThanALightOne()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 2.0);

        tyre.Step(null, 0.1, drive: 20, brake: 0);
        tyre.Step(null, 0.1, drive: 0, brake: 5);

        Assert.Equal(0.75, tyre.Spin, 12);
    }

    [Fact]
    public void ATyreHeldByAGentleBrakeLocksOnlyOnceTheBrakeBeatsTheWheelsInertia()
    {
        var free = new Tyre(new TyreSpec(), 0.35, 2.0);
        var braked = new Tyre(new TyreSpec(), 0.35, 2.0);
        var patch = Road(velocity: new Vector3(0, 0, -10));
        free.Step(patch, Dt, drive: 0, brake: 0);

        braked.Step(patch, Dt, drive: 0, brake: 1.5 * Math.Abs(free.Spin) / Dt);

        Assert.Equal(free.Spin * 0.25, braked.Spin, 9);
    }

    [Fact]
    public void ATyreWithNoLoadForgetsWhereItWasLastStep()
    {
        var used = new Tyre(new TyreSpec(), 0.35, 1.0);
        var fresh = new Tyre(new TyreSpec(), 0.35, 1.0);

        used.Step(Road(velocity: new Vector3(0, 0, -10), load: 0), Dt, drive: 0, brake: 0);
        used.Step(Road(velocity: new Vector3(0, 0, -2)), Dt, drive: 0, brake: 0);
        fresh.Step(Road(velocity: new Vector3(0, 0, -2)), Dt, drive: 0, brake: 0);

        Assert.Equal(fresh.Force.Z, used.Force.Z, 12);
    }

    [Fact]
    public void ATyreThatLocksHoldsTheRoadWhateverItsWheelWasDoing()
    {
        var spun = new Tyre(new TyreSpec(), 0.35, 1.0);
        var still = new Tyre(new TyreSpec(), 0.35, 1.0);
        spun.Step(null, 0.1, drive: 100, brake: 0);
        var patch = Road(velocity: new Vector3(0, 0, -2));

        spun.Step(patch, Dt, drive: 0, brake: 1e6);
        still.Step(patch, Dt, drive: 0, brake: 1e6);

        Assert.Equal(0.0, spun.Spin);
        Assert.True(still.Force.Z > 0, $"{still.Force.Z} does not drag against the way the road goes.");
        Assert.Equal(still.Force.Z, spun.Force.Z, 12);
    }

    [Fact]
    public void ATyreRollingBackwardsGivesTheMirrorOfOneRollingForwards()
    {
        var forward = new Tyre(new TyreSpec(), 0.35, 1.0);
        var backward = new Tyre(new TyreSpec(), 0.35, 1.0);
        forward.Step(null, 0.1, drive: 100, brake: 0);
        backward.Step(null, 0.1, drive: -100, brake: 0);

        forward.Step(Road(velocity: new Vector3(0, 0, -3.3)), Dt, drive: 0, brake: 0);
        backward.Step(Road(velocity: new Vector3(0, 0, 3.3)), Dt, drive: 0, brake: 0);

        Assert.Equal(-forward.Force.Z, backward.Force.Z, 12);
        Assert.Equal(-forward.Spin, backward.Spin, 12);
    }

    [Fact]
    public void ATyreMeasuresSlipAgainstAMetreASecondWhenTheGroundMovesSlower()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0.15, 0, 0), mass: 3000), Dt, drive: 0, brake: 0);

        Assert.Equal(-3600.0, tyre.Force.X, 6);
    }

    [Fact]
    public void ATyreBelowItsPeakSlipNeverGivesMoreThanItsGrip()
    {
        var tyre = new Tyre(new TyreSpec(), 0.35, 1.0);

        tyre.Step(Road(velocity: new Vector3(0.05, 0, 0), mass: 1e5), Dt, drive: 0, brake: 0);

        Assert.Equal(-4000.0, tyre.Force.X, 6);
    }

    [Fact]
    public void ADrivelineStartsAtIdleInFirstGearWithNothingPushingTheWheels()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        Assert.Equal(900.0, line.Rpm, 9);
        Assert.Equal(1, line.Gear);
        Assert.All(Enumerable.Range(0, 4), i => Assert.Equal(0.0, line.Torque(i)));
    }

    [Fact]
    public void ADrivelineHoldsTheEngineNearIdleInNeutralWithTheThrottleOff()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        for (var i = 0; i < 180; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(gear: 0), Dt);
        }

        Assert.InRange(line.Rpm, 850, 1000);
        Assert.Equal(0, line.Gear);
    }

    [Fact]
    public void ADrivelineRevsAnEngineThatIsHeldInNeutralWithTheThrottleDown()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        for (var i = 0; i < 60; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(throttle: 1, gear: 0), Dt);
        }

        Assert.InRange(line.Rpm, 5000, 6500 * 1.06);
    }

    [Fact]
    public void ADrivelineSharesTheTorqueAmongTheAxlesThenAmongTheirWheels()
    {
        var line = new Driveline(Drive(), Four(driven: [0, 1, 3]));

        for (var i = 0; i < 60; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(throttle: 1), Dt);
        }

        Assert.True(line.Torque(0) > 0, $"{line.Torque(0)} does not push.");
        Assert.Equal(0.0, line.Torque(2));
        Assert.Equal(line.Torque(0), 2 * line.Torque(1), 9);
        Assert.Equal(line.Torque(1), line.Torque(3), 9);
    }

    [Fact]
    public void ADrivelineInReversePushesTheWheelsTheOtherWay()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        for (var i = 0; i < 60; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(throttle: 1, gear: -1), Dt);
        }

        Assert.True(line.Torque(1) < 0, $"{line.Torque(1)} does not push back.");
        Assert.Equal(-1, line.Gear);
    }

    [Theory]
    [InlineData(4.0, 20.0)]
    [InlineData(-4.0, -20.0)]
    [InlineData(40.0, 50.0)]
    [InlineData(-40.0, -50.0)]
    public void ADrivelineLetsAnAxleCatchUpUnderItsDiffLockAndNoFurther(double spinGap, double expected)
    {
        var line = new Driveline(Drive(diffLock: 50), Four(driven: [1, 3]));

        line.Step([0, 2, 0, 2 + spinGap], Free(), new VehicleInput(gear: 0), 0.1);

        Assert.Equal(expected, line.Torque(1), 9);
        Assert.Equal(-expected, line.Torque(3), 9);
    }

    [Fact]
    public void ADrivelineWithAnOpenDifferentialLeavesTheWheelsAlone()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        line.Step([0, 2, 0, 30], Free(), new VehicleInput(gear: 0), 0.1);

        Assert.Equal(0.0, line.Torque(1));
        Assert.Equal(0.0, line.Torque(3));
    }

    [Fact]
    public void ADrivelineScalesTheCatchUpByTheInertiaOfTheWheel()
    {
        var heavy = new[]
        {
            Wheel(-0.8, -1.2),
            new WheelSpec(new Vector3(-0.8, 0, 1.2), 0.35, drives: true, inertia: 2.0),
            Wheel(0.8, -1.2),
            new WheelSpec(new Vector3(0.8, 0, 1.2), 0.35, drives: true, inertia: 2.0),
        };
        var line = new Driveline(Drive(diffLock: 50), heavy);

        line.Step([0, 2, 0, 6], Free(), new VehicleInput(gear: 0), 0.1);

        Assert.Equal(40.0, line.Torque(1), 9);
    }

    [Fact]
    public void ADrivelineShiftsUpOnItsOwnWhenTheWheelsRunFastWithThrottleOn()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        var gears = new List<int>();

        for (var i = 0; i < 4; i++)
        {
            line.Step(Spins(300), Free(), new VehicleInput(throttle: 0.5), 0.1);
            gears.Add(line.Gear);
        }

        Assert.Equal(new[] { 2, 2, 2, 3 }, gears);
    }

    [Fact]
    public void ADrivelineDoesNotShiftUpWithTheThrottleOff()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        for (var i = 0; i < 8; i++)
        {
            line.Step(Spins(300), Free(), new VehicleInput(), 0.1);
        }

        Assert.Equal(1, line.Gear);
    }

    [Fact]
    public void ADrivelineStopsShiftingUpAtItsTopGear()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        line.Step(Spins(300), Free(), new VehicleInput(throttle: 0.5, gear: 5), 0.1);

        for (var i = 0; i < 8; i++)
        {
            line.Step(Spins(300), Free(), new VehicleInput(throttle: 0.5), 0.1);
        }

        Assert.Equal(5, line.Gear);
    }

    [Fact]
    public void ADrivelineShiftsDownOnItsOwnOnceTheWheelsSlow()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        var gears = new List<int>();

        line.Step(Spins(0), Free(), new VehicleInput(gear: 3), 0.1);
        gears.Add(line.Gear);
        for (var i = 0; i < 3; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(), 0.1);
            gears.Add(line.Gear);
        }

        Assert.Equal(new[] { 3, 3, 3, 2 }, gears);
    }

    [Fact]
    public void ADrivelineNeverShiftsDownBelowFirst()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        for (var i = 0; i < 20; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(), 0.1);
        }

        Assert.Equal(1, line.Gear);
    }

    [Fact]
    public void ADrivelineKeepsAGearWhileTheWheelsStillTurnFastEnoughToHoldIt()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        line.Step(Spins(300), Free(), new VehicleInput(gear: 3), 0.1);

        for (var i = 0; i < 8; i++)
        {
            line.Step(Spins(300), Free(), new VehicleInput(), 0.1);
        }

        Assert.Equal(3, line.Gear);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    public void ADrivelineLeavesReverseAndNeutralForFirstWhenToldNothing(int from)
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        line.Step(Spins(0), Free(), new VehicleInput(gear: from), 0.1);

        line.Step(Spins(0), Free(), new VehicleInput(), 0.1);

        Assert.Equal(1, line.Gear);
    }

    [Fact]
    public void ADrivelineShiftsUpWhenTheWheelsRunBackwardsFastEnough()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        line.Step(Spins(-300), Free(), new VehicleInput(gear: 3), 0.1);

        for (var i = 0; i < 8; i++)
        {
            line.Step(Spins(-300), Free(), new VehicleInput(throttle: 0.5), 0.1);
        }

        Assert.True(line.Gear > 3, $"Gear {line.Gear} did not go up.");
    }

    [Fact]
    public void ADrivelineBringsTheEngineUpToTheSpeedTheWheelsGiveWhateverWayTheyTurn()
    {
        var line = new Driveline(Drive(), Four(driven: [1, 3]));

        line.Step(Spins(-300), Free(), new VehicleInput(gear: 3), 0.1);

        Assert.InRange(line.Rpm, 6000, 7500);
    }

    [Fact]
    public void ADrivelineEasesTheThrottleOffInTheFiveSharesAboveTheRedline()
    {
        const double Redline = 6500;
        const double Dt3 = 0.3;
        var line = new Driveline(Drive(), Four(driven: [1, 3]));
        var rpmPerRadian = 60 / (2 * Math.PI);
        var target = Redline * 1.025 / rpmPerRadian;

        line.Step(Spins(target / (1.5 * 4.5)), Free(), new VehicleInput(throttle: 1, gear: 3), Dt3);

        Assert.Equal((target + (0.5 * 200 * Dt3 / 0.25)) * rpmPerRadian, line.Rpm, 6);
    }

    [Fact]
    public void ADrivelineNeverTurnsItsEngineBackwards()
    {
        var light = new DriveSpec(
            new EngineSpec([(1000, 160), (3000, 220), (5000, 240), (6500, 200)], inertia: 0.001),
            [3.6, 2.2, 1.5, 1.1, 0.9],
            finalDrive: 4.5,
            clutch: 1000,
            shiftTime: 0);
        var line = new Driveline(light, Four(driven: [1, 3]));
        for (var i = 0; i < 30; i++)
        {
            line.Step(Spins(0), Free(), new VehicleInput(throttle: 1, gear: 0), Dt);
        }

        Func<double, double>[] backwards = [.. Enumerable.Repeat<Func<double, double>>(_ => -5000.0, 4)];
        line.Step(Spins(-5000), backwards, new VehicleInput(throttle: 1, gear: 1), Dt);

        Assert.Equal(0.0, line.Rpm);
    }

    [Fact]
    public void ADrivelineLeavesAnAxleOfThreeWheelsWithoutADifferential()
    {
        WheelSpec[] wide = [Wheel(-0.8, 1.2, drives: true), Wheel(0, 1.2, drives: true), Wheel(0.8, 1.2, drives: true)];
        var line = new Driveline(Drive(diffLock: 50), wide);

        line.Step([0, 5, 20], Free(), new VehicleInput(gear: 0), 0.1);

        Assert.All(Enumerable.Range(0, 3), i => Assert.Equal(0.0, line.Torque(i)));
    }

    [Fact]
    public void ADrivelineSendsTheSameTotalToTheWheelsWhicheverAxlesDrive()
    {
        var rear = new Driveline(Drive(), Four(driven: [1, 3]));
        var three = new Driveline(Drive(), Four(driven: [0, 1, 3]));

        for (var i = 0; i < 60; i++)
        {
            rear.Step(Spins(0), Free(), new VehicleInput(throttle: 1), Dt);
            three.Step(Spins(0), Free(), new VehicleInput(throttle: 1), Dt);
        }

        Assert.True(rear.Torque(1) > 0, $"{rear.Torque(1)} does not push.");
        Assert.Equal(2 * rear.Torque(1), three.Torque(0) + three.Torque(1) + three.Torque(3), 9);
    }

    [Fact]
    public void AChassisReadFromAMissingBodyIsNothing()
    {
        using var physics = new Physics();

        Assert.Null(Chassis.Read(physics, 7, [0, 0, 0]));
    }

    [Fact]
    public void AChassisReadFromABodyKnowsItsPlaceItsTurnAndItsCentre()
    {
        const double Tilt = 0.5;
        using var physics = new Physics();
        physics.Add(
            1,
            Shape.Box(1, 1, 1),
            at: [1, 2, 3],
            rotation: [Math.Sin(Tilt / 2), 0, 0, Math.Cos(Tilt / 2)],
            mass: 10);

        var chassis = Chassis.Read(physics, 1, [0, 0, -1])!;

        AssertVector(new Vector3(1, 2, 3), chassis.Origin, 6);
        AssertVector(new Vector3(1, 2 + Math.Sin(Tilt), 3 - Math.Cos(Tilt)), chassis.Centre, 6);
        AssertVector(new Vector3(0, -Math.Cos(Tilt), -Math.Sin(Tilt)), chassis.Down, 6);
        AssertVector(new Vector3(1, 2 + Math.Cos(Tilt), 3 + Math.Sin(Tilt)), chassis.ToWorld(new Vector3(0, 1, 0)), 6);
        AssertVector(new Vector3(0, Math.Cos(Tilt), Math.Sin(Tilt)), chassis.Direct(new Vector3(0, 1, 0)), 6);
    }

    [Fact]
    public void AChassisReadFromAMovingBodyKnowsItsVelocityAndSpin()
    {
        using var physics = new Physics();
        physics.Add(1, Shape.Box(1, 1, 1), mass: 10);
        physics.Drive(1, velocity: [1, 2, 3], spin: [4, 5, 6]);

        var chassis = Chassis.Read(physics, 1, [0, 0, 0])!;

        AssertVector(new Vector3(1, 2, 3), chassis.Velocity);
        AssertVector(new Vector3(4, 5, 6), chassis.Spin);
    }

    [Fact]
    public void AChassisMovesAPointByItsVelocityAndItsSpinAboutTheCentreWithNothingBefore()
    {
        var chassis = Still(velocity: new Vector3(1, 0, 0), spin: new Vector3(0, 2, 0), centre: new Vector3(0, 0, 1));

        var motion = chassis.MotionAt(new Vector3(0, 0, 0), null, Dt);

        AssertVector(new Vector3(1 - 2, 0, 0), motion, 12);
    }

    [Fact]
    public void AChassisMovesAPointByWhereItWasLastStep()
    {
        var before = Still();
        var now = Still(origin: new Vector3(0.1, 0, 0));

        var motion = now.MotionAt(new Vector3(0.1, 0, -1), before, 0.1);

        AssertVector(new Vector3(1, 0, 0), motion, 12);
    }

    [Fact]
    public void AChassisMovesAPointByTheTurnItMadeSinceLastStep()
    {
        const double Turn = 0.1;
        var now = Still(turn: new Quaternion(0, Math.Sin(Turn / 2), 0, Math.Cos(Turn / 2)));

        var motion = now.MotionAt(new Vector3(0, 0, -1), Still(), 0.1);

        AssertVector(new Vector3(-Math.Sin(Turn) / 0.1, 0, (Math.Cos(Turn) - 1) / 0.1), motion, 9);
    }

    private static Patch Road(
        Vector3? velocity = null,
        Vector3? heading = null,
        double load = 4000,
        double surface = 1,
        double mass = 300) =>
        new(
            velocity ?? new Vector3(0, 0, -10),
            new Vector3(0, 1, 0),
            heading ?? new Vector3(0, 0, -1),
            load,
            surface,
            mass);

    private static Chassis Still(
        Vector3? origin = null,
        Quaternion? turn = null,
        Vector3? velocity = null,
        Vector3? spin = null,
        Vector3? centre = null)
    {
        var quaternion = turn ?? new Quaternion(0, 0, 0, 1);
        return new Chassis(
            origin ?? Vector3.Zero(),
            quaternion,
            quaternion.AsRotationMatrix(),
            centre ?? Vector3.Zero(),
            velocity ?? Vector3.Zero(),
            spin ?? Vector3.Zero());
    }

    private static DriveSpec Drive(double diffLock = 0.0) =>
        new(
            new EngineSpec([(1000, 160), (3000, 220), (5000, 240), (6500, 200)]),
            [3.6, 2.2, 1.5, 1.1, 0.9],
            finalDrive: 4.5,
            diffLock: diffLock);

    private static double[] Spins(double spin) => [spin, spin, spin, spin];

    /// <summary>Wheels that do not answer to the torque, which is a car with no road under it.</summary>
    private static Func<double, double>[] Free() => [.. Enumerable.Repeat<Func<double, double>>(_ => 0.0, 4)];
}
