using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftAttachmentTests : SoftTestBase
{
    private const double Weight = Columns * Columns * Mass * 9.81;

    /// <summary>
    /// A curtain in the x-y plane with its top row at y = 2, on the finer
    /// sub-step a two-corner hang needs.
    /// </summary>
    private static SoftBody Curtain() =>
        SoftBody.Cloth(
            Columns,
            Columns,
            new Vector3(0, 2, 0),
            new Vector3(Spacing, 0, 0),
            new Vector3(0, -Spacing, 0),
            Mass,
            new SoftSpec(substepLength: 1.0 / 960));

    /// <summary>A box in the air, held in place or driven, to hang a cloth from.</summary>
    private static Physics Rig(PhysicsMotion motion = PhysicsMotion.Driven)
    {
        var physics = new Physics();
        physics.Add(1, Shape.Box(0.5, 0.1, 0.5), motion, at: [0.5, 2, 0]);
        physics.Step(Dt);
        return physics;
    }

    /// <summary>
    /// Both top corners of <paramref name="cloth"/> attached to points on body 1,
    /// one <see cref="Columns"/> spacings apart, with the same limit.
    /// </summary>
    private static void Hang(SoftBody cloth, Physics physics, double? limit = null)
    {
        cloth.Attach(0, BodyPoint.Of(physics, 1, new Vector3(-0.5, 0, 0)), limit);
        cloth.Attach(
            Columns - 1,
            BodyPoint.Of(physics, 1, new Vector3(-0.5 + ((Columns - 1) * Spacing), 0, 0)),
            limit);
    }

    private static double CentreX(SoftBody cloth) => Points(cloth).Average(p => p.X);

    [Fact]
    public void TwoCornersCarryTheWeightBetweenThem()
    {
        using var physics = Rig(PhysicsMotion.Fixed);
        var cloth = Curtain();
        Hang(cloth, physics);
        Run(cloth, 5);
        var total = cloth.PullOf(0) + cloth.PullOf(Columns - 1);
        Assert.Equal(-Weight, total.Y, 0.05 * Weight);
        Less(Math.Abs(total.X), 0.05 * Weight);
        Greater(cloth.PullOf(0).X, 0);
        Less(cloth.PullOf(Columns - 1).X, 0);
    }

    [Fact]
    public void ABodyAtRestHoldsItsLoadWhileItSleeps()
    {
        using var physics = Rig(PhysicsMotion.Fixed);
        var cloth = Curtain();
        Hang(cloth, physics);
        Run(cloth, 12);
        Assert.True(cloth.Asleep);
        Less(cloth.PullOf(0).Y + cloth.PullOf(Columns - 1).Y, -30);
    }

    [Fact]
    public void ALimitAboveThePullHolds()
    {
        using var physics = Rig(PhysicsMotion.Fixed);
        var cloth = Curtain();

        // The first swing puts about 41 N on a corner. It settles at 22 N.
        Hang(cloth, physics, limit: 50);
        Run(cloth, 3);
        Assert.True(cloth.IsAttached(0));
        Assert.True(cloth.IsAttached(Columns - 1));
    }

    [Fact]
    public void ALimitBelowThePullLetsGoAndTheRestThenFallToIt()
    {
        using var physics = Rig(PhysicsMotion.Fixed);
        var cloth = Curtain();
        cloth.Attach(0, BodyPoint.Of(physics, 1, new Vector3(-0.5, 0, 0)), limit: 5);
        cloth.Attach(Columns - 1, BodyPoint.Of(physics, 1, new Vector3(0.45, 0, 0)));
        Run(cloth, 3);
        Assert.False(cloth.IsAttached(0));
        Assert.False(cloth.IsPinned(0));
        Assert.True(cloth.IsAttached(Columns - 1));
        Less(cloth.PositionOf(0).Y, 1.5);
    }

    [Fact]
    public void AClothOnAMovingBodyGoesWhereItGoes()
    {
        using var physics = Rig();
        var cloth = Curtain();
        Hang(cloth, physics);
        var start = CentreX(cloth);
        for (var n = 0; n < 120; n++)
        {
            physics.Drive(1, velocity: [1, 0, 0]);
            physics.Step(Dt);
            cloth.Step(Dt);
        }

        var pose = physics.TransformOf(1)!;
        Assert.Equal(2.5, pose[0], 0.05);
        var corner = cloth.PositionOf(0);
        Assert.Equal(pose[0] - 0.5, corner.X, 1e-9);
        Assert.Equal(pose[0] - 0.5, CentreX(cloth) - start, 0.15);
    }

    [Fact]
    public void ABodyThatMovesWakesASleepingCloth()
    {
        using var physics = Rig();
        var cloth = Curtain();
        Hang(cloth, physics);
        Run(cloth, 12);
        Assert.True(cloth.Asleep);
        physics.Place(1, at: [0.5, 2, 0.5]);
        physics.Step(Dt);
        cloth.Step(Dt);
        Assert.False(cloth.Asleep);
        Assert.Equal(0.5, cloth.PositionOf(0).Z, 1e-9);
    }

    [Fact]
    public void APointOnATurnedBodyTurnsWithIt()
    {
        using var physics = new Physics();
        var half = 15 * Math.PI / 180;
        physics.Add(
            1,
            Shape.Box(0.5, 0.5, 0.5),
            PhysicsMotion.Fixed,
            at: [1, 2, 3],
            rotation: [0, Math.Sin(half), 0, Math.Cos(half)]);
        physics.Step(Dt);
        var point = BodyPoint.Of(physics, 1, new Vector3(0.5, 0, 0))();
        var turn = 30 * Math.PI / 180;
        Assert.Equal(1 + (0.5 * Math.Cos(turn)), point.X, 1e-5);
        Assert.Equal(3 - (0.5 * Math.Sin(turn)), point.Z, 1e-5);
        Assert.Equal(2, point.Y, 1e-5);
    }

    [Fact]
    public void APointOnABodyThatIsGoneIsRefused()
    {
        using var physics = Rig();
        var where = BodyPoint.Of(physics, 1, Vector3.Zero());
        physics.Remove(1);
        physics.Step(Dt);
        Assert.Throws<InvalidOperationException>(() => where());
    }

    [Fact]
    public void AnAttachOutsideTheBodyOrWithALimitOfNothing()
    {
        var cloth = Curtain();
        static Vector3 Here() => Vector3.Zero();
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.Attach(-1, Here));
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.Attach(cloth.Count, Here));
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.Attach(0, Here, limit: 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => cloth.Attach(0, Here, limit: double.NaN));
    }

    [Fact]
    public void ThePullOnAParticleThatIsNotAttached()
    {
        Assert.Throws<InvalidOperationException>(() => Curtain().PullOf(0));
    }

    [Fact]
    public void DetachFreesAnAttachedParticleAndLeavesAPinnedOne()
    {
        var cloth = Curtain();
        cloth.Attach(0, () => new Vector3(0, 2, 0));
        cloth.Pin(1);
        cloth.Detach(0);
        cloth.Detach(1);
        Assert.False(cloth.IsAttached(0));
        Assert.False(cloth.IsPinned(0));
        Assert.True(cloth.IsPinned(1));
    }
}
