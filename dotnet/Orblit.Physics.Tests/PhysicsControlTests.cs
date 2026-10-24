using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class PhysicsControlTests : PhysicsTestBase
{
    private static readonly Shape Crate = Shape.Box(0.5, 0.5, 0.5);

    private double SpeedOf(ulong id)
    {
        var v = World.VelocityOf(id)!;
        return Math.Sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
    }

    [Fact]
    public void ItsStructIsTheSizeTheEngineReads() =>
        Assert.Equal(56, Marshal.SizeOf<NativeControls>());

    [Fact]
    public void LocksAreTheBitsTheEngineNumbersThemBy()
    {
        var named = Enum.GetValues<PhysicsLock>().Where(it => it != PhysicsLock.None);

        Assert.Equal(new uint[] { 1, 2, 4, 8, 16, 32 }, named.Select(it => (uint)it));
        Assert.Equal(2u | 32u, (uint)(PhysicsLock.MoveY | PhysicsLock.TurnZ));
    }

    [Fact]
    public void ABodyLockedToAPlaneAtHalfGravityUnderACapBehavesSo()
    {
        World.Add(2, Crate, at: [0.0, 20.0, 0.0]);
        World.Add(3, Crate, at: [4.0, 20.0, 0.0]);
        var plane = new PhysicsControls(
            Locks: PhysicsLock.MoveZ | PhysicsLock.TurnX | PhysicsLock.TurnY,
            GravityScale: 0.5,
            MaxSpeed: 3.0);
        Assert.True(World.SetControls(2, plane));

        // Body 3 is the same box with nothing set, so what differs is the controls.
        foreach (ulong id in new ulong[] { 2, 3 })
        {
            World.Drive(id, velocity: [2.0, 0.0, 2.0], spin: [4.0, 4.0, 4.0]);
        }

        Run(1.0);

        var held = World.VelocityOf(2)!;
        Assert.Equal(0.0, World.TransformOf(2)![2], 1e-4);
        Assert.Equal(0.0, held[2]);
        Assert.Equal(0.0, held[3]);
        Assert.Equal(0.0, held[4]);
        Assert.True(held[5] > 1.0, "the turn left is still free");
        Assert.True(SpeedOf(2) <= 3.001);

        Assert.True(World.TransformOf(3)![2] > 1.0);
        Assert.True(20.0 - HeightOf(2) < 20.0 - HeightOf(3));
    }

    [Fact]
    public void HalfGravityFallsHalfAsFar()
    {
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 50.0, 0.0]);
        World.Add(3, Shape.Sphere(0.5), at: [4.0, 50.0, 0.0]);
        World.SetControls(2, new PhysicsControls(GravityScale: 0.5));

        Run(0.5);

        // Damping trims both falls a little, so the ratio is not exactly half.
        Assert.Equal(0.5, (50.0 - HeightOf(2)) / (50.0 - HeightOf(3)), 0.03);
    }

    [Fact]
    public void ContinuousHoldsASmallSlowBallAtAThinWall()
    {
        // 2 cm from the face at 2.9 m/s is 4.8 cm a step, which puts the ball
        // 1.3 cm past the middle of a wall 1 cm thick, and the contact pushes
        // it out the far side. That is under the speed contacts are made early.
        double XAfter(bool continuous)
        {
            Restart(new PhysicsSettings { Gravity = [0.0, 0.0, 0.0] });
            World.Add(2, Shape.Box(0.005, 2.0, 2.0), PhysicsMotion.Fixed);
            World.Add(3, Shape.Sphere(0.01), at: [0.035, 0.0, 0.0]);
            World.SetControls(3, new PhysicsControls(Continuous: continuous));
            World.Drive(3, velocity: [-2.9, 0.0, 0.0]);
            Run(40.0 / 60);
            return World.TransformOf(3)![0];
        }

        Assert.True(XAfter(continuous: false) < -0.02);
        Assert.True(XAfter(continuous: true) > 0.005);
    }

    [Fact]
    public void ASpeedCapHoldsAFallAndASpinCapHoldsASpin()
    {
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 500.0, 0.0]);
        World.SetControls(2, new PhysicsControls(MaxSpeed: 3.0, MaxSpin: 2.0));
        World.Drive(2, spin: [10.0, 0.0, 0.0]);

        // One step, since angular damping would trim a spin already at its cap.
        World.Step(1.0 / 60);
        Assert.Equal(2.0, World.VelocityOf(2)![3], 0.01);

        Run(1.0);
        Assert.True(Math.Abs(SpeedOf(2) - 3.0) <= 0.01, "held, not just slowed");
    }

    [Fact]
    public void APushGoesThroughTheCentreOfMassUnlessToldOtherwise()
    {
        var weighted = new PhysicsControls(Centre: [0.4, 0.0, 0.0]);
        World.Add(2, Crate, at: [10.0, 5.0, 0.0]);
        World.Add(3, Crate, at: [15.0, 5.0, 0.0]);
        World.SetControls(2, weighted);
        World.SetControls(3, weighted);

        World.Push(2, impulse: [0.0, 0.0, 1.0]);
        World.Push(3, impulse: [0.0, 0.0, 1.0], at: [15.0, 5.0, 0.0]);
        World.Step(1.0 / 60);

        var through = World.VelocityOf(2)!;
        Assert.Equal(1.0, through[2], 0.01);
        Assert.True(Math.Abs(through[4]) < 0.001);
        Assert.True(
            World.VelocityOf(3)![4] > 1.0,
            "through the origin, which is off to one side of the weight");
    }

    [Fact]
    public void AGivenInertiaIsTheOneTheBodyTurnsBy()
    {
        World.Add(2, Crate, at: [0.0, 5.0, 0.0]);
        World.Add(3, Crate, at: [5.0, 5.0, 0.0]);
        World.SetControls(3, new PhysicsControls(Inertia: [100.0, 100.0, 100.0]));

        World.Push(2, impulse: [0.0, 0.0, 2.0], at: [0.0, 5.5, 0.0]);
        World.Push(3, impulse: [0.0, 0.0, 2.0], at: [5.0, 5.5, 0.0]);
        World.Step(1.0 / 60);

        Assert.Equal(6.0, World.VelocityOf(2)![3], 0.05);
        Assert.Equal(0.01, World.VelocityOf(3)![3], 0.002);
    }

    [Fact]
    public void ControlsThatCannotBeUsedAreRefusedAndChangeNothing()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);

        var half = new PhysicsControls(GravityScale: 0.5);
        Assert.False(World.SetControls(2, new PhysicsControls(GravityScale: double.NaN)));
        Assert.False(World.SetControls(2, new PhysicsControls(MaxSpeed: -1.0)));
        Assert.False(World.SetControls(2, new PhysicsControls(Inertia: [-1.0, 1.0, 1.0])));
        Assert.False(World.SetControls(9, half), "no such body");
        Assert.False(World.SetControls(Ground, half), "not a body");

        Run(0.5);
        Assert.True(3.0 - HeightOf(2) > 1.0, "still full gravity");
        Assert.True(World.SetControls(2, half));
    }

    [Fact]
    public void TheWorldsGravityIsSetAndWakesWhatWasAsleep()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 1.0, 0.0]);
        Run(3.0);
        Assert.True(World.Asleep(2));

        Assert.True(World.SetGravity([0.0, 9.81, 0.0]));
        Run(1.0);

        Assert.False(World.Asleep(2));
        Assert.True(HeightOf(2) > 2.0);
        Assert.False(World.SetGravity([0.0, double.NaN, 0.0]));
    }

    [Fact]
    public void ABodyIsMadeFixedOrFreeAndAFixedOneStops()
    {
        World.Add(2, Crate, PhysicsMotion.Fixed, [0.0, 10.0, 0.0]);
        Run(0.5);
        Assert.Equal(10.0, HeightOf(2));

        World.SetMotion(2, PhysicsMotion.Free);
        Run(0.5);
        Assert.True(HeightOf(2) < 9.0);

        World.SetMotion(2, PhysicsMotion.Fixed);
        World.Step(1.0 / 60);
        var stopped = HeightOf(2);
        Run(0.5);
        Assert.Equal(stopped, HeightOf(2));
        Assert.True(World.VelocityOf(2)!.All(it => it == 0.0));
    }

    [Fact]
    public void ATriggerIsNeverMadeFree()
    {
        World.Add(2, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed, [0.0, 10.0, 0.0], trigger: true);
        World.SetMotion(2, PhysicsMotion.Free);
        Run(0.5);
        Assert.Equal(10.0, HeightOf(2));
    }

    [Fact]
    public void ControlsGivenQuietlyToABodyAddedAsleepLeaveItAsleep()
    {
        World.Add(2, Crate, at: [0.0, 10.0, 0.0], asleep: true);
        var took = World.SetControls(2, new PhysicsControls(GravityScale: 0.5), quiet: true);
        Run(0.5);

        Assert.True(took);
        Assert.True(World.Asleep(2));
        Assert.Equal(10.0, HeightOf(2));
    }

    [Fact]
    public void ControlsGivenToASleeperWakeItUnlessAskedNotTo()
    {
        World.Add(2, Crate, at: [0.0, 10.0, 0.0], asleep: true);
        World.SetControls(2, new PhysicsControls(GravityScale: 0.5));

        Assert.False(World.Asleep(2));
    }

    [Fact]
    public void AQuietBodyKeepsItsControlsForWhenItWakes()
    {
        World.Add(2, Crate, at: [0.0, 10.0, 0.0], asleep: true);
        World.SetControls(2, new PhysicsControls(GravityScale: 0.5), quiet: true);
        World.Wake(2);
        Run(0.5);

        // Half gravity for half a second falls about 0.61 m, not 1.2.
        Assert.Equal(0.61, 10.0 - HeightOf(2), 0.08);
    }

    [Fact]
    public void TwoBodiesThatIgnoreEachOtherPassThrough()
    {
        Floor();

        // Two columns: a box on the ground with another dropped on it. Only the
        // first column is told to ignore.
        void Column(ulong low, ulong high, double x)
        {
            World.Add(low, Crate, at: [x, 0.5, 0.0]);
            World.Add(high, Crate, at: [x, 3.0, 0.0]);
        }

        Column(2, 3, 0.0);
        Column(4, 5, 5.0);
        Assert.True(World.SetRule(2, 3, new PhysicsRule(ignore: true)));

        Run(3.0);

        Assert.Equal(0.5, HeightOf(3), 0.05);
        Assert.Equal(1.5, HeightOf(5), 0.05);
    }

    [Fact]
    public void AOneWayPlatformLetsABodyUpAndHoldsItComingDown()
    {
        Floor();
        World.Add(2, Shape.Box(1.0, 0.1, 1.0), PhysicsMotion.Fixed, [0.0, 2.0, 0.0]);
        World.Add(3, Crate, at: [0.0, 0.5, 0.0]);
        Assert.True(World.SetRule(2, 3, new PhysicsRule(oneWay: [0.0, 1.0, 0.0])));
        World.Drive(3, velocity: [0.0, 9.0, 0.0]);

        Run(4.0);

        Assert.Equal(2.6, HeightOf(3), 0.05);
        Assert.False(World.SetRule(2, 3, new PhysicsRule(oneWay: [0.0, 0.0, 0.0])));
    }
}
