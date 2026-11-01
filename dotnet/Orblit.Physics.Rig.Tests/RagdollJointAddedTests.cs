using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollJointAddedTests
{
    private static IReadOnlyList<double> Angles(Harness h) => h.Physics.JointStateOf(h.Ragdoll.IdOf("b"))!.Angles;

    private static void Push(Harness h, double[] impulse, double[] at) =>
        h.Physics.Push(h.Ragdoll.IdOf("b"), impulse, at);

    [Theory]
    [InlineData(3.0, 0.9)]
    [InlineData(-3.0, -0.3)]
    public void AHingeStopsAtTheLimitItWasGiven(double push, double stops)
    {
        using var h = Limbs.Chain(new RagdollHinge(low: -0.3, high: 0.9), RagdollMode.Limp);
        Push(h, [0.0, 0.0, push], [1.5, 1.05, 0.0]);

        h.Run(0.6);

        Near(stops, Angles(h)[0], 0.05, "hinge angle");
    }

    [Theory]
    [InlineData(6.0, 2, 0.5)]
    [InlineData(-6.0, 2, -0.5)]
    public void ABallStopsAtItsSwingInTheBendOfTheBone(double push, int axis, double stops)
    {
        using var h = Limbs.Chain(new RagdollBall(swing: 0.5, twistLow: -0.2, twistHigh: 0.7), RagdollMode.Limp);
        Push(h, [0.0, push, 0.0], [2.0, 1.0, 0.0]);

        h.Run(0.6);

        Near(stops, Angles(h)[axis], 0.08, "swing");
    }

    [Theory]
    [InlineData(6.0, -0.5)]
    [InlineData(-6.0, 0.5)]
    public void ABallStopsAtItsSwingToTheSide(double push, double stops)
    {
        using var h = Limbs.Chain(new RagdollBall(swing: 0.5, twistLow: -0.2, twistHigh: 0.7), RagdollMode.Limp);
        Push(h, [0.0, 0.0, push], [2.0, 1.0, 0.0]);

        h.Run(0.6);

        Near(stops, Angles(h)[1], 0.08, "swing");
    }

    [Theory]
    [InlineData(1.0, 0.7)]
    [InlineData(-1.0, -0.2)]
    public void ABallStopsAtTheEndOfItsTwistThatTheTurnWasTowards(double sign, double stops)
    {
        using var h = Limbs.Chain(new RagdollBall(swing: 0.5, twistLow: -0.2, twistHigh: 0.7), RagdollMode.Limp);
        Push(h, [0.0, 0.0, 0.05 * sign], [1.5, 1.05, 0.0]);
        Push(h, [0.0, 0.0, -0.05 * sign], [1.5, 0.95, 0.0]);

        h.Run(1.0);

        Near(stops, Angles(h)[0], 0.05, "twist");
    }

    [Fact]
    public void APoweredBallIsPulledBackOnEveryAxis()
    {
        using var h = Limbs.Chain(new RagdollBall(swing: 0.5, twistLow: -0.2, twistHigh: 0.7), RagdollMode.Powered);
        Push(h, [0.0, 0.3, 0.0], [2.0, 1.0, 0.0]);
        Push(h, [0.0, 0.0, 0.3], [2.0, 1.0, 0.0]);
        Push(h, [0.0, 0.0, 0.05], [1.5, 1.05, 0.0]);
        Push(h, [0.0, 0.0, -0.05], [1.5, 0.95, 0.0]);

        h.Run(2.0);

        foreach (var angle in Angles(h))
        {
            Less(Math.Abs(angle), 0.05, "angle");
        }
    }

    [Fact]
    public void APoweredHingeIsPulledBack()
    {
        using var h = Limbs.Chain(new RagdollHinge(low: -0.3, high: 0.9), RagdollMode.Powered);
        Push(h, [0.0, 0.0, 0.3], [2.0, 1.0, 0.0]);

        h.Run(2.0);

        Less(Math.Abs(Angles(h)[0]), 0.05, "hinge angle");
    }
}
