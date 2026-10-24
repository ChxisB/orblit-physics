namespace Orblit.Physics.Tests;

public class PhysicsControlAddedTests : PhysicsTestBase
{
    private static readonly Shape Crate = Shape.Box(0.5, 0.5, 0.5);

    /// <summary>
    /// A crate high above any ground, pushed once at its own origin, answering
    /// the spin the push gave it. The origin is off its weight by the centre.
    /// </summary>
    private float[] SpinAfterPush(double[] centre, double[] impulse)
    {
        Restart();
        World.Add(2, Crate, at: [0.0, 50.0, 0.0]);
        World.SetControls(2, new PhysicsControls(Centre: centre));
        World.Push(2, impulse: impulse, at: [0.0, 50.0, 0.0]);
        World.Step(1.0 / 60);
        return World.VelocityOf(2)![3..];
    }

    /// <summary>The spin a crate is given by a push at height 0.5 above its weight, about x, y and z.</summary>
    private float[] SpinGivenInertia(double[] inertia)
    {
        Restart();
        World.Add(2, Crate, at: [0.0, 50.0, 0.0]);
        World.Add(3, Crate, at: [5.0, 50.0, 0.0]);
        World.Add(4, Crate, at: [10.0, 50.0, 0.0]);
        var spin = new float[3];
        (ulong Id, double[] Arm, double[] Impulse)[] pushes =
        [
            (2, [0.0, 0.5, 0.0], [0.0, 0.0, 2.0]),
            (3, [0.0, 0.0, 0.5], [2.0, 0.0, 0.0]),
            (4, [0.5, 0.0, 0.0], [0.0, 2.0, 0.0]),
        ];
        foreach (var (id, arm, impulse) in pushes)
        {
            World.SetControls(id, new PhysicsControls(Inertia: inertia));
            var at = World.TransformOf(id)!;
            World.Push(id, impulse, [at[0] + arm[0], at[1] + arm[1], at[2] + arm[2]]);
        }

        World.Step(1.0 / 60);
        for (var i = 0; i < 3; i++)
        {
            spin[i] = World.VelocityOf(pushes[i].Id)![3 + i];
        }

        return spin;
    }

    [Theory]
    [InlineData(PhysicsLock.MoveX, 0)]
    [InlineData(PhysicsLock.MoveY, 1)]
    [InlineData(PhysicsLock.MoveZ, 2)]
    [InlineData(PhysicsLock.TurnX, 3)]
    [InlineData(PhysicsLock.TurnY, 4)]
    [InlineData(PhysicsLock.TurnZ, 5)]
    public void EachLockHoldsItsOwnWayAndNoOther(PhysicsLock locked, int index)
    {
        World.Add(2, Crate, at: [0.0, 50.0, 0.0]);
        World.SetControls(2, new PhysicsControls(Locks: locked));
        World.Drive(2, velocity: [2.0, 2.0, 2.0], spin: [4.0, 4.0, 4.0]);

        World.Step(1.0 / 60);

        var v = World.VelocityOf(2)!;
        for (var i = 0; i < 6; i++)
        {
            Assert.Equal(i == index, v[i] == 0.0);
        }
    }

    [Fact]
    public void ACentreAboveTheOriginTurnsAPushAlongZAboutXOnly()
    {
        var spin = SpinAfterPush([0.0, 0.4, 0.0], [0.0, 0.0, 1.0]);

        Assert.True(spin[0] < -1.0);
        Assert.True(Math.Abs(spin[1]) < 0.05 && Math.Abs(spin[2]) < 0.05);
    }

    [Fact]
    public void ACentreBesideTheOriginTurnsAPushAlongXAboutYOnly()
    {
        var spin = SpinAfterPush([0.0, 0.0, 0.4], [1.0, 0.0, 0.0]);

        Assert.True(spin[1] < -1.0);
        Assert.True(Math.Abs(spin[0]) < 0.05 && Math.Abs(spin[2]) < 0.05);
    }

    [Fact]
    public void AnInertiaIsReadOnePerAxis()
    {
        var spin = SpinGivenInertia([100.0, 200.0, 400.0]);

        Assert.Equal(0.01, spin[0], 0.0005);
        Assert.Equal(0.005, spin[1], 0.0005);
        Assert.Equal(0.0025, spin[2], 0.0005);
    }

    [Fact]
    public void AZeroInTheInertiaLeavesTheBodyWithTheShapes()
    {
        World.Add(2, Crate, at: [0.0, 5.0, 0.0]);
        Assert.True(World.SetControls(2, new PhysicsControls(Inertia: [0.0, 100.0, 100.0])));

        World.Push(2, impulse: [0.0, 0.0, 2.0], at: [0.0, 5.5, 0.0]);
        World.Step(1.0 / 60);

        Assert.Equal(6.0, World.VelocityOf(2)![3], 0.05);
    }

    [Fact]
    public void EveryFieldIsCheckedForANumber()
    {
        World.Add(2, Crate, at: [0.0, 5.0, 0.0]);

        Assert.False(World.SetControls(2, new PhysicsControls(GravityScale: double.PositiveInfinity)));
        Assert.False(World.SetControls(2, new PhysicsControls(MaxSpin: -1.0)));
        Assert.False(World.SetControls(2, new PhysicsControls(Centre: [0.0, double.NaN, 0.0])));
        Assert.False(World.SetControls(2, new PhysicsControls(Inertia: [1.0, double.NaN, 1.0])));
        Assert.True(World.SetControls(2, new PhysicsControls()));
    }

    [Fact]
    public void ControlsLeftAtTheirDefaultsDoNotStopABodyAtAThinWall()
    {
        Restart(new PhysicsSettings { Gravity = [0.0, 0.0, 0.0] });
        World.Add(2, Shape.Box(0.005, 2.0, 2.0), PhysicsMotion.Fixed);
        World.Add(3, Shape.Sphere(0.01), at: [0.035, 0.0, 0.0]);
        World.SetControls(3, new PhysicsControls());
        World.Drive(3, velocity: [-2.9, 0.0, 0.0]);

        Run(40.0 / 60);

        Assert.True(World.TransformOf(3)![0] < -0.02);
    }

    [Fact]
    public void ABodyMadeDrivenKeepsTheVelocityItHad()
    {
        World.Add(2, Crate, at: [0.0, 10.0, 0.0]);
        World.Drive(2, velocity: [2.0, 0.0, 0.0]);
        World.SetMotion(2, PhysicsMotion.Driven);

        Run(1.0);

        Assert.Equal(2.0, World.TransformOf(2)![0], 0.1);
        Assert.Equal(10.0, HeightOf(2));
    }

    [Fact]
    public void TheWorldsGravityPullsAlongEachAxisItIsGiven()
    {
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 50.0, 0.0]);
        Assert.True(World.SetGravity([0.0, 0.0, 4.0]));

        Run(1.0);

        Assert.Equal(2.0, World.TransformOf(2)![2], 0.2);
        Assert.Equal(50.0, HeightOf(2), 0.01);
        Assert.Equal(0.0, World.TransformOf(2)![0], 0.01);
    }

    [Fact]
    public void SettingGravityWakesABodyAddedAsleepJustBefore()
    {
        World.Add(2, Crate, at: [0.0, 10.0, 0.0], asleep: true);

        World.SetGravity([0.0, -9.81, 0.0]);

        Assert.False(World.Asleep(2));
    }

    [Fact]
    public void ADisposedWorldRefusesMotionControlsAndGravity()
    {
        var gone = new Physics();
        gone.Dispose();

        Assert.Throws<InvalidOperationException>(() => gone.SetMotion(2, PhysicsMotion.Free));
        Assert.Throws<InvalidOperationException>(() => gone.SetControls(2, new PhysicsControls()));
        Assert.Throws<InvalidOperationException>(() => gone.SetGravity([0.0, -9.81, 0.0]));
    }
}
