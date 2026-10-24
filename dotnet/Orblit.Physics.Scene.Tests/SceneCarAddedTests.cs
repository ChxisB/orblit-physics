using Orblit.Physics.Vehicle;
using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneCarAddedTests : SceneTestBase
{
    private static double SagOf(double hertz) => 9.81 / Math.Pow(2 * Math.PI * hertz, 2);

    private static SceneEntity Wheel(string id, double x, double z, WheelComponent wheel, double y) => new(
        id,
        id,
        "car",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(x, y, z)),
            [SceneComponents.Wheel] = wheel,
        });

    private static SceneEntity Chassis(BodyComponent body, VehicleComponent car, Vector3? scale = null, double height = 0.7) => new(
        "car",
        "Car",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(0, height, 0), scale: scale),
            [SceneComponents.Body] = body,
            [SceneComponents.Vehicle] = car,
        });

    private static BodyComponent Body(Vector3? centre = null, Vector3? centreOfMass = null) =>
        new(size: new Vector3(1.8, 0.6, 4), mass: 1200, centre: centre, centreOfMass: centreOfMass);

    private static SceneDocument WithFourWheels(
        BodyComponent body,
        VehicleComponent car,
        WheelComponent front,
        WheelComponent rear,
        double rearRightZ = 1.2,
        Vector3? scale = null,
        double height = 0.7,
        double wheelY = 0.2) => SceneOf(
            Floor(),
            Chassis(body, car, scale, height),
            Wheel("fl", -0.8, -1.2, front, wheelY),
            Wheel("fr", 0.8, -1.2, front, wheelY),
            Wheel("rl", -0.8, 1.2, rear, wheelY),
            Wheel("rr", 0.8, rearRightZ, rear, wheelY));

    private static Vehicle.Vehicle? CarOf(ScenePhysics scene) => scene.VehicleOf("car");

    [Fact]
    public void ACarCarriesEveryFieldOfItsComponentsIntoTheVehicleSpec()
    {
        var front = new WheelComponent(steers: true, brake: 2500, handbrake: true, travel: 0.25, hertz: 2, grip: 0.8);
        var rear = new WheelComponent(drives: true, brake: 1800, grip: 1.3, travel: 0.25, hertz: 2);
        var car = new VehicleComponent(maxSteer: 35, antiRoll: 4000, idle: 800, reverse: 2.9, finalDrive: 4.1, @lock: 0.6, ground: 6);
        var scene = Make(WithFourWheels(
            Body(centre: new Vector3(0.1, 0.2, 0.3), centreOfMass: new Vector3(0, -0.1, 0.05)),
            car,
            front,
            rear));

        var spec = CarOf(scene)!.Spec;

        Assert.Equal(1200.0, spec.Mass);
        Assert.Equal([0.0, -0.1, 0.05], spec.Centre);
        Assert.Equal(4000.0, spec.AntiRoll);
        Assert.Equal(Doubles.Radians(35), spec.MaxSteer, 1e-12);
        Assert.Equal(6u, spec.Ground.Cares);
        Assert.Equal(0u, spec.Ground.Is);
        Assert.Equal(800.0, spec.Drive!.Engine.Idle);
        Assert.Equal(2.9, spec.Drive.Reverse);
        Assert.Equal(4.1, spec.Drive.FinalDrive);
        Assert.Equal(0.6, spec.Drive.Lock);
        var top = 0.2 + 0.25 - SagOf(2);
        Assert.Equal(-0.8 - 0.1, spec.Wheels[0].Mount.X, 1e-9);
        Assert.Equal(top - 0.2, spec.Wheels[0].Mount.Y, 1e-9);
        Assert.Equal(-1.2 - 0.3, spec.Wheels[0].Mount.Z, 1e-9);
        Assert.Equal(
            [(true, false, 2500.0, true, 0.8), (true, false, 2500.0, true, 0.8), (false, true, 1800.0, false, 1.3), (false, true, 1800.0, false, 1.3)],
            spec.Wheels.Select(w => (w.Steers, w.Drives, w.BrakeTorque, w.Handbrake, w.Tyre.Grip)).ToArray());
    }

    [Fact]
    public void ACarWithNoDrivingWheelHasNoDrive()
    {
        var wheel = new WheelComponent();
        var scene = Make(WithFourWheels(Body(), new VehicleComponent(), wheel, wheel));

        Assert.Null(CarOf(scene)!.Spec.Drive);
    }

    [Fact]
    public void ATorqueCurveWithAnOddNumberOfValuesDropsTheLastOne()
    {
        var rear = new WheelComponent(drives: true);
        var scene = Make(WithFourWheels(
            Body(),
            new VehicleComponent(torque: [1000.0, 100.0, 4000.0, 200.0, 6000.0]),
            new WheelComponent(),
            rear));

        var torque = CarOf(scene)!.Spec.Drive!.Engine.Torque;

        Assert.Equal([(1000.0, 100.0), (4000.0, 200.0)], torque.ToArray());
    }

    [Fact]
    public void WheelsAtTheSameDepthToATenthOfAMillimetreShareAnAxle()
    {
        var wheel = new WheelComponent(drives: true);
        var scene = Make(WithFourWheels(Body(), new VehicleComponent(antiRoll: 1000), wheel, wheel, rearRightZ: 1.20004));

        Assert.Equal(2, CarOf(scene)!.Spec.Axles.Count);
    }

    [Fact]
    public void ACarScaledBySomeHundredthsIsNotACar()
    {
        var wheel = new WheelComponent();

        Assert.Null(CarOf(Make(WithFourWheels(Body(), new VehicleComponent(), wheel, wheel, scale: Vector3.All(1.05)))));
    }

    [Fact]
    public void AWheelNeverRisesAboveTheTopOfItsSpring()
    {
        var wheel = new WheelComponent();
        var scene = Make(WithFourWheels(Body(), new VehicleComponent(), wheel, wheel, height: 0.31, wheelY: -0.3));
        var top = -0.3 + 0.3 - SagOf(1.5);
        var worst = 0.0;

        Run(scene, 1, () =>
        {
            worst = Math.Max(worst, CarOf(scene)!.Wheels.Max(w => w.Compression - 0.3));
            foreach (var id in new[] { "fl", "fr", "rl", "rr" })
            {
                Assert.True(TransformOf(scene.Document, id).Position.Y <= top + 1e-9, $"{id} went up through its mount");
            }
        });

        Assert.True(worst > 0, "the car never bottomed out, so this proves nothing");
    }

    [Fact]
    public void AWheelTurnsBackwardsAboutItsAxleByTheAmountItHasRolled()
    {
        var scene = Make(World());
        Run(scene, 2);
        scene.Pilot("car", new VehicleInput(throttle: 1, steer: 0.6));
        Run(scene, 1);

        var states = CarOf(scene)!.Wheels;
        Assert.True(states[2].Roll > 0.01 && states[0].Steer > 0.01);
        Near(-Doubles.Degrees(states[2].Roll), TransformOf(scene.Document, "rl").Rotation.X, 1e-9, "rl rolls");
        Near(Doubles.Degrees(states[0].Steer), TransformOf(scene.Document, "fl").Rotation.Y, 1e-9, "fl steers");
    }

    private static SceneDocument World()
    {
        var front = new WheelComponent(steers: true);
        var rear = new WheelComponent(drives: true);
        return WithFourWheels(Body(), new VehicleComponent(), front, rear);
    }

    [Fact]
    public void PosesNamesNoWheelThatIsAlreadyWhereTheCarWouldPutIt()
    {
        var (document, car, physics) = Direct();
        using var _ = physics;
        var poses = car.Poses(document);
        Assert.Equal(4, poses.Count);

        var settled = WithTransforms(poses, rotation: p => p.Rotation, position: p => p.Position);

        Assert.Empty(car.Poses(settled));
    }

    [Fact]
    public void PosesNamesAWheelThatIsInTheRightPlaceButTurnedWrong()
    {
        var (document, car, physics) = Direct();
        using var _ = physics;
        var poses = car.Poses(document);

        var wrong = WithTransforms(poses, rotation: _ => new Vector3(5, 0, 0), position: p => p.Position);

        Assert.Equal(4, car.Poses(wrong).Count);
    }

    [Fact]
    public void PosesNamesAWheelThatIsTurnedRightButInTheWrongPlace()
    {
        var (document, car, physics) = Direct();
        using var _ = physics;
        var poses = car.Poses(document);

        var wrong = WithTransforms(poses, rotation: p => p.Rotation, position: p => p.Position + new Vector3(0, 0.01, 0));

        Assert.Equal(4, car.Poses(wrong).Count);
    }

    private static (SceneDocument Document, Car Car, Physics Physics) Direct()
    {
        var wheel = new WheelComponent();
        var document = WithFourWheels(Body(), new VehicleComponent(), wheel, wheel);
        var physics = new Physics();
        physics.Add(900, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
        physics.Add(1, Shape.Box(0.9, 0.3, 2), mass: 1200, at: [0.0, 0.7, 0.0]);
        return (document, Car.Make(document, "car", physics, 1, 9.81)!, physics);
    }

    private static SceneDocument WithTransforms(
        Dictionary<string, TransformComponent> poses,
        Func<TransformComponent, Vector3> rotation,
        Func<TransformComponent, Vector3> position)
    {
        var wheel = new WheelComponent();
        var entities = new List<SceneEntity> { Floor(), Chassis(Body(), new VehicleComponent()) };
        foreach (var (id, pose) in poses.OrderBy(p => p.Key, StringComparer.Ordinal))
        {
            entities.Add(new SceneEntity(
                id,
                id,
                "car",
                components: new Dictionary<string, SceneComponent>
                {
                    [SceneComponents.Transform] = new TransformComponent(position(pose), rotation(pose)),
                    [SceneComponents.Wheel] = wheel,
                }));
        }

        return SceneOf([.. entities]);
    }
}
