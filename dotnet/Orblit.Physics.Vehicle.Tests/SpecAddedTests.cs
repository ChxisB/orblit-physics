using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// The edge of every number a spec accepts, and what the specs derive from them.
/// </summary>
public class SpecAddedTests
{
    private static readonly (double Rpm, double NewtonMetres)[] Curve =
        [(1000, 100), (3000, 200), (5000, 150)];

    private static EngineSpec Engine() => new(Curve);

    private static WheelSpec Wheel(double x, double z, bool drives = false, bool steers = false) =>
        new(new Vector3(x, 0, z), 0.35, steers: steers, drives: drives);

    private static WheelSpec[] Four(bool rearDrives = false) =>
        [
            .. new[] { -0.8, 0.8 }.SelectMany(x =>
                new[] { -1.2, 1.2 }.Select(z => Wheel(x, z, drives: rearDrives && z > 0))),
        ];

    [Theory]
    [InlineData(0.0, 1.5, 0.5)]
    [InlineData(-0.1, 1.5, 0.5)]
    [InlineData(0.3, 0.0, 0.5)]
    [InlineData(0.3, -1.0, 0.5)]
    [InlineData(0.3, 8.0001, 0.5)]
    [InlineData(0.3, 1.5, -0.001)]
    [InlineData(0.3, 1.5, 2.001)]
    [InlineData(double.NaN, 1.5, 0.5)]
    [InlineData(0.3, double.NaN, 0.5)]
    [InlineData(0.3, 1.5, double.NaN)]
    public void ASuspensionOutOfRangeIsRefused(double travel, double frequency, double damping) =>
        Assert.ThrowsAny<ArgumentException>(() => new SuspensionSpec(travel, frequency, damping));

    [Theory]
    [InlineData(0.001, 0.001, 0.0)]
    [InlineData(0.3, 8.0, 2.0)]
    public void ASuspensionOnTheEdgeIsAccepted(double travel, double frequency, double damping)
    {
        var spec = new SuspensionSpec(travel, frequency, damping);
        Assert.Equal((travel, frequency, damping), (spec.Travel, spec.Frequency, spec.Damping));
    }

    [Fact]
    public void ASuspensionSagsByWhatWeightSquashesOrItsWholeTravel()
    {
        var omega = 2 * Math.PI * 1.5;
        Assert.Equal(9.81 / (omega * omega), new SuspensionSpec(0.3, 1.5).Sag(), 12);
        Assert.Equal(2.0 / (omega * omega), new SuspensionSpec(0.3, 1.5).Sag(2.0), 12);
        Assert.Equal(0.05, new SuspensionSpec(0.05, 1.5).Sag());
    }

    [Theory]
    [InlineData(0.0, 0.1, 0.8, 0.2)]
    [InlineData(1.0, 0.0, 0.8, 0.2)]
    [InlineData(1.0, 1.001, 0.8, 0.2)]
    [InlineData(1.0, 0.1, 0.0, 0.2)]
    [InlineData(1.0, 0.1, 1.001, 0.2)]
    [InlineData(1.0, 0.1, 0.8, 0.0)]
    [InlineData(double.NaN, 0.1, 0.8, 0.2)]
    [InlineData(1.0, double.NaN, 0.8, 0.2)]
    [InlineData(1.0, 0.1, double.NaN, 0.2)]
    [InlineData(1.0, 0.1, 0.8, double.NaN)]
    public void ATyreOutOfRangeIsRefused(double grip, double peakSlip, double sliding, double relaxation) =>
        Assert.ThrowsAny<ArgumentException>(() => new TyreSpec(grip, peakSlip, sliding, relaxation));

    [Fact]
    public void ATyreOnTheEdgeIsAccepted()
    {
        var spec = new TyreSpec(grip: 0.001, peakSlip: 1.0, sliding: 1.0, relaxation: 0.001);
        Assert.Equal((0.001, 1.0, 1.0, 0.001), (spec.Grip, spec.PeakSlip, spec.Sliding, spec.Relaxation));
    }

    [Theory]
    [InlineData(0.0, 0.0, false, 1.0)]
    [InlineData(-0.1, 0.0, false, 1.0)]
    [InlineData(0.3, -1.0, false, 1.0)]
    [InlineData(0.3, 0.0, true, 1.0)]
    [InlineData(0.3, 100.0, false, 0.0)]
    [InlineData(0.3, 100.0, false, -1.0)]
    [InlineData(double.NaN, 0.0, false, 1.0)]
    [InlineData(0.3, double.NaN, false, 1.0)]
    [InlineData(0.3, 100.0, false, double.NaN)]
    public void AWheelOutOfRangeIsRefused(double radius, double brakeTorque, bool handbrake, double inertia) =>
        Assert.ThrowsAny<ArgumentException>(() =>
            new WheelSpec(Vector3.Zero(), radius, brakeTorque: brakeTorque, handbrake: handbrake, inertia: inertia));

    [Fact]
    public void AWheelWithBrakesAndAHandbrakeIsAccepted()
    {
        var spec = new WheelSpec(Vector3.Zero(), 0.001, brakeTorque: 0.001, handbrake: true, inertia: 0.001);
        Assert.True(spec.Handbrake);
    }

    [Fact]
    public void AWheelKeepsItsOwnCopyOfItsMountAndDefaultsTheRest()
    {
        var mount = new Vector3(1, 2, 3);
        var spec = new WheelSpec(mount, 0.3);
        mount.X = 9;

        Assert.Equal(1, spec.Mount.X);
        Assert.False(spec.Steers);
        Assert.False(spec.Drives);
        Assert.False(spec.Handbrake);
        Assert.Equal(0, spec.BrakeTorque);
        Assert.Equal(1, spec.Inertia);
        Assert.Equal(new SuspensionSpec().Travel, spec.Suspension.Travel);
        Assert.Equal(new TyreSpec().Grip, spec.Tyre.Grip);
    }

    [Theory]
    [InlineData(-1.001, 0, 0, 0)]
    [InlineData(1.001, 0, 0, 0)]
    [InlineData(0, -0.001, 0, 0)]
    [InlineData(0, 1.001, 0, 0)]
    [InlineData(0, 0, -0.001, 0)]
    [InlineData(0, 0, 1.001, 0)]
    [InlineData(0, 0, 0, -2)]
    [InlineData(double.NaN, 0, 0, 0)]
    [InlineData(0, double.NaN, 0, 0)]
    [InlineData(0, 0, double.NaN, 0)]
    public void AnInputOutOfRangeIsRefused(double steer, double throttle, double brake, int gear) =>
        Assert.ThrowsAny<ArgumentException>(() => new VehicleInput(steer, throttle, brake, gear: gear));

    [Fact]
    public void AnInputOnTheEdgeIsAccepted()
    {
        var input = new VehicleInput(steer: -1, throttle: 1, brake: 1, handbrake: true, gear: -1);
        Assert.Equal((-1.0, 1.0, 1.0, true, -1), (input.Steer, input.Throttle, input.Brake, input.Handbrake, input.Gear));
        Assert.Null(new VehicleInput().Gear);
        Assert.Equal(1.0, new VehicleInput(steer: 1).Steer);
        Assert.Equal(0.0, new VehicleInput(throttle: 0, brake: 0).Throttle);
    }

    [Fact]
    public void AnEngineReadsItsCurve()
    {
        var engine = Engine();

        Assert.Equal(5000, engine.Redline);
        Assert.Equal(200, engine.Peak);
        Assert.Equal((900.0, 0.25, 40.0), (engine.Idle, engine.Inertia, engine.Drag));
    }

    [Theory]
    [InlineData(100.0, 100.0)]
    [InlineData(1000.0, 100.0)]
    [InlineData(2000.0, 150.0)]
    [InlineData(3000.0, 200.0)]
    [InlineData(4000.0, 175.0)]
    [InlineData(5000.0, 150.0)]
    [InlineData(9000.0, 150.0)]
    public void AnEngineGivesItsFullTorqueByStraightLinesBetweenThePoints(double rpm, double expected) =>
        Assert.Equal(expected, Engine().Full(rpm), 12);

    [Fact]
    public void AnEnginePeakIsTheMostAnywhereOnTheCurve()
    {
        Assert.Equal(300, new EngineSpec([(1000, 300), (2000, 100)], idle: 500).Peak);
        Assert.Equal(300, new EngineSpec([(1000, 100), (2000, 300)], idle: 500).Peak);
        Assert.Equal(0, new EngineSpec([(1000, 0), (2000, 0)], idle: 500).Peak);
    }

    [Fact]
    public void AnEngineKeepsItsOwnCopyOfItsCurve()
    {
        var points = new[] { (1000.0, 100.0), (2000.0, 200.0) };
        var engine = new EngineSpec(points, idle: 500);
        points[0] = (1.0, 1.0);

        Assert.Equal(100, engine.Torque[0].NewtonMetres);
    }

    [Fact]
    public void AnEngineOnTheEdgeIsAccepted()
    {
        var engine = new EngineSpec([(0.001, 0), (0.002, 0)], idle: 0.0015, inertia: 0.001, drag: 0);
        Assert.Equal(0.0015, engine.Idle);
    }

    [Fact]
    public void AnEngineWithoutTwoPointsIsRefused()
    {
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([], idle: 1));
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([(1000, 100)], idle: 500));
    }

    [Theory]
    [InlineData(0.0, 100.0, 2000.0, 100.0)]
    [InlineData(-5.0, 100.0, 2000.0, 100.0)]
    [InlineData(1000.0, -0.001, 2000.0, 100.0)]
    [InlineData(1000.0, 100.0, 0.0, 100.0)]
    [InlineData(1000.0, 100.0, 1000.0, 100.0)]
    [InlineData(1000.0, 100.0, 999.0, 100.0)]
    [InlineData(1000.0, 100.0, 2000.0, -1.0)]
    [InlineData(1000.0, 100.0, double.NaN, 100.0)]
    [InlineData(double.NaN, 100.0, 2000.0, 100.0)]
    [InlineData(1000.0, double.NaN, 2000.0, 100.0)]
    public void AnEngineCurveOutOfRangeIsRefused(double rpmA, double torqueA, double rpmB, double torqueB) =>
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([(rpmA, torqueA), (rpmB, torqueB)], idle: 0.5));

    [Fact]
    public void AnEngineCurveThatTurnsBackLaterIsRefused() =>
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([(1000, 1), (2000, 1), (1500, 1)], idle: 500));

    [Theory]
    [InlineData(0.0, 0.25, 40.0)]
    [InlineData(-1.0, 0.25, 40.0)]
    [InlineData(5000.0, 0.25, 40.0)]
    [InlineData(5001.0, 0.25, 40.0)]
    [InlineData(900.0, 0.0, 40.0)]
    [InlineData(900.0, -1.0, 40.0)]
    [InlineData(900.0, 0.25, -0.001)]
    [InlineData(double.NaN, 0.25, 40.0)]
    [InlineData(900.0, double.NaN, 40.0)]
    [InlineData(900.0, 0.25, double.NaN)]
    public void AnEngineIdleInertiaOrDragOutOfRangeIsRefused(double idle, double inertia, double drag) =>
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec(Curve, idle, inertia, drag));

    [Fact]
    public void ADriveDefaultsItsClutchAndEngageFromTheEngine()
    {
        var drive = new DriveSpec(Engine(), [3, 2, 1]);

        Assert.Equal(300, drive.Clutch);
        Assert.Equal(900 + ((5000 - 900) * 0.25), drive.Engage);
        Assert.Equal((3.2, 3.7, 0.0, 0.3, 0.85, 0.35), (drive.Reverse, drive.FinalDrive, drive.Lock, drive.ShiftTime, drive.Upshift, drive.Downshift));
    }

    [Fact]
    public void ADriveKeepsAClutchAndEngageItIsGiven()
    {
        var drive = new DriveSpec(Engine(), [3, 2, 1], clutch: 50, engage: 4999, diffLock: 7);

        Assert.Equal((50.0, 4999.0, 7.0), (drive.Clutch, drive.Engage, drive.Lock));
    }

    [Fact]
    public void ADriveKeepsItsOwnCopyOfItsRatios()
    {
        var ratios = new[] { 3.0, 2.0, 1.0 };
        var drive = new DriveSpec(Engine(), ratios);
        ratios[0] = 99;

        Assert.Equal(3, drive.Ratios[0]);
    }

    [Fact]
    public void ADriveOnTheEdgeIsAccepted()
    {
        var edge = new DriveSpec(
            Engine(),
            [2.0, 1.0],
            reverse: 0.001,
            finalDrive: 0.001,
            clutch: 0.001,
            engage: (900 * 1.1) + 0.001,
            diffLock: 0,
            shiftTime: 0,
            upshift: 1.0,
            downshift: 0.001);

        Assert.Equal((0.0, 0.0), (edge.ShiftTime, edge.Lock));
        Assert.Equal(1.0, edge.Upshift);
    }

    [Theory]
    [InlineData(0.0, 3.7, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(-1.0, 3.7, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 0.0, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 0.0, 2000.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 990.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 5000.0, 0.0, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, -0.001, 0.3, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, 0.0, -0.001, 0.85, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, 0.0, 0.3, 1.001, 0.35)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.85)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.0)]
    [InlineData(3.2, 3.7, 300.0, 2000.0, 0.0, 0.3, 0.85, 0.9)]
    public void ADriveOutOfRangeIsRefused(
        double reverse,
        double finalDrive,
        double clutch,
        double engage,
        double diffLock,
        double shiftTime,
        double upshift,
        double downshift) =>
        Assert.ThrowsAny<ArgumentException>(() =>
            new DriveSpec(Engine(), [3, 2, 1], reverse, finalDrive, clutch, engage, diffLock, shiftTime, upshift, downshift));

    [Fact]
    public void ADriveEngagingAtTenthAboveIdleExactlyIsRefused()
    {
        var idle = 900.0;
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(Engine(), [3, 2, 1], engage: idle * 1.1));
    }

    [Theory]
    [InlineData(new double[0])]
    [InlineData(new[] { 3.0, 0.0, 1.0 })]
    [InlineData(new[] { 3.0, -2.0, 1.0 })]
    [InlineData(new[] { 3.0, double.NaN, 1.0 })]
    public void ADriveWithoutGearsAboveZeroIsRefused(double[] ratios) =>
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(Engine(), ratios));

    [Fact]
    public void ADriveWhoseDownshiftLandsPastTheUpshiftIsRefused()
    {
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(Engine(), [3.0, 1.0]));
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(Engine(), [2.0, 1.0], upshift: 1.0, downshift: 0.5));
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(Engine(), [1.5, 1.2, 0.5], downshift: 0.4));
        _ = new DriveSpec(Engine(), [1.9, 1.0], upshift: 1.0, downshift: 0.5);
    }

    [Fact]
    public void AVehicleKeepsItsOwnCopyOfItsWheelsAndCentre()
    {
        var wheels = Four();
        var centre = new[] { 0.0, 0.0, 0.0 };
        var spec = new VehicleSpec(1200, wheels, centre);
        wheels[0] = Wheel(5, 5);
        centre[2] = 9;

        Assert.Equal(-0.8, spec.Wheels[0].Mount.X);
        Assert.Equal(0.0, spec.Centre[2]);
        Assert.Equal((0.5, 0.0), (spec.MaxSteer, spec.AntiRoll));
        Assert.Null(spec.Drive);
        Assert.Equal(Layers.Everything, spec.Ground);
    }

    [Fact]
    public void AVehicleSharesItsWeightByWhereTheCentreSits()
    {
        Assert.All(new VehicleSpec(1200, Four()).Sprung, load => Assert.Equal(300, load, 9));

        var forward = new VehicleSpec(1200, Four(), centre: [0, 0, -0.4]).Sprung;
        Assert.Equal(new[] { 400.0, 200.0, 400.0, 200.0 }, forward.Select(load => Math.Round(load, 9)));

        var left = new VehicleSpec(1200, Four(), centre: [-0.4, 0, 0]).Sprung;
        Assert.Equal(new[] { 450.0, 450.0, 150.0, 150.0 }, left.Select(load => Math.Round(load, 9)));
    }

    [Fact]
    public void AVehicleSharesItsWeightEvenlyWhenTheWheelsDoNotSpanTheGround()
    {
        var one = new VehicleSpec(1000, [Wheel(0, 0)]);
        var line = new VehicleSpec(1000, [Wheel(0, -1), Wheel(0, 1)], centre: [0, 0, 0.3]);

        Assert.Equal(new[] { 1000.0 }, one.Sprung);
        Assert.Equal(new[] { 500.0, 500.0 }, line.Sprung);
    }

    [Theory]
    [InlineData(-2.0)]
    [InlineData(-1.3)]
    [InlineData(1.3)]
    public void AVehicleWhoseCentreSitsOutsideItsWheelsIsRefused(double z) =>
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, Four(), centre: [0, 0, z]));

    [Fact]
    public void AVehicleWhoseCentreSitsJustInsideItsWheelsIsAccepted() =>
        Assert.True(new VehicleSpec(1200, Four(), centre: [0, 0, -1.19]).Sprung.All(load => load > 0));

    [Fact]
    public void AVehicleBalancesItsWheelsAboutTheCentreWhateverTheLayout()
    {
        WheelSpec[] skewed = [Wheel(-0.7, -1.0), Wheel(0.9, -1.2), Wheel(-0.6, 1.5), Wheel(1.0, 1.1)];
        double[] centre = [0.25, 0.0, 0.4];
        const double Mass = 1500;

        var loads = new VehicleSpec(Mass, skewed, centre).Sprung;

        Assert.Equal(Mass, loads.Sum(), 9);
        Assert.Equal(Mass * centre[0], loads.Select((load, i) => load * skewed[i].Mount.X).Sum(), 9);
        Assert.Equal(Mass * centre[2], loads.Select((load, i) => load * skewed[i].Mount.Z).Sum(), 9);
    }

    [Fact]
    public void AVehicleWhoseCentreLeavesAWheelWithNoLoadIsRefusedForItsCentre()
    {
        WheelSpec[] square = [Wheel(-1, -1), Wheel(1, -1), Wheel(-1, 1), Wheel(1, 1)];

        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new VehicleSpec(1200, square, centre: [0, 0, 1]));

        Assert.Equal("centre", error.ParamName);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    public void AVehicleWithNoMassIsRefusedForItsMassNotItsCentre(double mass)
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new VehicleSpec(mass, Four()));

        Assert.Equal("mass", error.ParamName);
    }

    [Fact]
    public void AnAntiRollBarNeedsTwoWheelsOnEveryAxleNotJustOneOnSome()
    {
        WheelSpec[] odd = [Wheel(-0.8, -1.2), Wheel(0.8, -1.2), Wheel(0, 1.2)];

        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new VehicleSpec(1200, odd, antiRoll: 100));

        Assert.Equal("wheels", error.ParamName);
    }

    [Fact]
    public void AVehicleThatSteersAFullQuarterTurnIsRefused()
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() =>
            new VehicleSpec(1200, Four(), maxSteer: Math.PI / 2));

        Assert.Equal("maxSteer", error.ParamName);
    }

    [Fact]
    public void ADriveWhoseOnlyGearIsNothingIsRefusedForItsRatios()
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new DriveSpec(Engine(), [0.0]));

        Assert.Equal("ratios", error.ParamName);
    }

    [Fact]
    public void ADriveWhoseDownshiftEqualsItsUpshiftIsRefused()
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() =>
            new DriveSpec(Engine(), [3.0], upshift: 0.5, downshift: 0.5));

        Assert.Equal("upshift", error.ParamName);
    }

    [Theory]
    [InlineData(0.0, 0.5, false)]
    [InlineData(-1.0, 0.5, false)]
    [InlineData(1200.0, -0.001, false)]
    [InlineData(1200.0, 1.5708, false)]
    [InlineData(1200.0, double.NaN, false)]
    [InlineData(double.NaN, 0.5, false)]
    [InlineData(1200.0, 0.5, true)]
    public void AVehicleOutOfRangeIsRefused(double mass, double maxSteer, bool drivenWithoutADrive) =>
        Assert.ThrowsAny<ArgumentException>(() =>
            new VehicleSpec(mass, Four(rearDrives: drivenWithoutADrive), maxSteer: maxSteer));

    [Fact]
    public void AVehicleWithNoWheelsIsRefused() =>
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, []));

    [Fact]
    public void AVehicleWithADriveNeedsAWheelThatDrives() =>
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, Four(), drive: new DriveSpec(Engine(), [3, 2, 1])));

    [Fact]
    public void AVehicleOnTheEdgeIsAccepted()
    {
        var spec = new VehicleSpec(
            0.001,
            Four(rearDrives: true),
            maxSteer: 0.0,
            drive: new DriveSpec(Engine(), [3, 2, 1]));
        var wide = new VehicleSpec(1200, Four(), maxSteer: 1.5707);

        Assert.Equal(0.0, spec.MaxSteer);
        Assert.Equal(1.5707, wide.MaxSteer);
    }

    [Fact]
    public void AnAntiRollBarPairsTheWheelsOfEachAxleInTheOrderTheyAreMet()
    {
        var spec = new VehicleSpec(1200, Four(), antiRoll: 1000);

        Assert.Equal(new[] { (0, 2), (1, 3) }, spec.Axles);
        Assert.Empty(new VehicleSpec(1200, Four()).Axles);
        Assert.Empty(new VehicleSpec(1200, [Wheel(0, 0), Wheel(1, 0), Wheel(2, 0)]).Axles);
    }

    [Fact]
    public void AnAntiRollBarNeedsExactlyTwoWheelsOnEachAxle()
    {
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, [Wheel(0, 0)], antiRoll: 1));
        Assert.ThrowsAny<ArgumentException>(() =>
            new VehicleSpec(1200, [Wheel(-1, 0), Wheel(0, 0), Wheel(1, 0)], antiRoll: 1));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, Four(), antiRoll: -0.001));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(1200, Four(), antiRoll: double.NaN));
    }
}
