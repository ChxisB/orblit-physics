using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollBodyAddedTests
{
    private static float[] Velocity(Harness h) => h.Physics.VelocityOf(h.Ragdoll.IdOf("a"))!;

    [Theory]
    [InlineData(1.0, 2.0)]
    [InlineData(4.0, 0.5)]
    public void AHeavierBodyTakesAnImpulseMoreSlowly(double mass, double speed)
    {
        using var h = Limbs.Lying(mass);
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        h.Ragdoll.Hit("a", new Vector3(2, 0, 0));

        h.Frame();

        Near(speed, Velocity(h)[0], 0.02, "speed");
    }

    [Fact]
    public void TheLinearDampingInTheTuningSlowsABody()
    {
        using var free = Limbs.Lying(tuning: new RagdollTuning(LinearDamping: 0, AngularDamping: 0));
        using var damped = Limbs.Lying(tuning: new RagdollTuning(LinearDamping: 3, AngularDamping: 0));
        foreach (var h in new[] { free, damped })
        {
            h.Physics.SetGravity([0.0, 0.0, 0.0]);
            h.Ragdoll.Hit("a", new Vector3(0, 0, 1));
            h.Run(1.0);
        }

        Near(1.0, Velocity(free)[2], 0.02, "undamped speed");
        Less(Velocity(damped)[2], 0.1, "damped speed");
    }

    [Fact]
    public void TheAngularDampingInTheTuningSlowsASpin()
    {
        using var free = Limbs.Lying(tuning: new RagdollTuning(LinearDamping: 0, AngularDamping: 0));
        using var damped = Limbs.Lying(tuning: new RagdollTuning(LinearDamping: 0, AngularDamping: 3));
        foreach (var h in new[] { free, damped })
        {
            h.Physics.SetGravity([0.0, 0.0, 0.0]);
            h.Ragdoll.Hit("a", new Vector3(0, 0, 1), new Vector3(1, 0.15, 0));
            h.Run(1.0);
        }

        Greater(Math.Abs(Velocity(free)[4]), 4.0, "undamped spin");
        Less(Math.Abs(Velocity(damped)[4]), 0.5, "damped spin");
    }

    [Fact]
    public void TheFrictionOfABodyDecidesHowFarItSlidesAcrossTheFloor()
    {
        using var slick = Limbs.Lying(friction: 0);
        using var grippy = Limbs.Lying(friction: 0.5);
        foreach (var h in new[] { slick, grippy })
        {
            h.Ragdoll.Hit("a", new Vector3(1, 0, 0));
            h.Run(2.0);
        }

        Greater(slick.BodyOf("a").X, 2.0, "slick slide");
        Less(grippy.BodyOf("a").X, 1.0, "grippy slide");
    }

    [Theory]
    [InlineData(1, 0, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(0, 0, 1)]
    public void ABodyMovingAlongAnyAxisIsNotAtRest(double x, double y, double z)
    {
        using var h = Limbs.Lying();
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        h.Ragdoll.Hit("a", new Vector3(x, y, z));
        h.Frame();

        Assert.False(h.Ragdoll.IsAtRest());
    }

    [Fact]
    public void ABodyThatOnlySpinsIsNotAtRest()
    {
        using var h = Limbs.Lying();
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        h.Ragdoll.Hit("a", new Vector3(0, 0, 0.18), new Vector3(1, 0.15, 0));
        h.Frame();

        Less(Math.Abs(Velocity(h)[2]), 0.2, "linear speed");
        Assert.False(h.Ragdoll.IsAtRest());
    }

    [Fact]
    public void ABodyThatIsGoneFromTheWorldCannotKeepTheRagdollFromRest()
    {
        using var h = Limbs.Chain(new RagdollHinge(), RagdollMode.Limp);
        h.Physics.Push(h.Ragdoll.IdOf("b"), [0.0, 0.0, 5.0], null);
        h.Frame();
        Assert.False(h.Ragdoll.IsAtRest());

        h.Physics.Remove(h.Ragdoll.IdOf("b"));

        Assert.True(h.Ragdoll.IsAtRest());
    }

    [Fact]
    public void AnAnimatedModelOnTheMoveIsAtRestBecauseNoBodyIsFree()
    {
        using var h = new Harness();
        for (var i = 1; i <= 10; i++)
        {
            h.Model = Matrix4.Compose(new Vector3(0.05 * i, 0, 0), Quaternion.Identity(), new Vector3(1, 1, 1));
            h.Frame();
        }

        Assert.True(h.Ragdoll.IsAtRest());
    }
}
