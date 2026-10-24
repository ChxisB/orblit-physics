using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollRecoveryAddedTests
{
    private static Matrix4 At(double x) =>
        Matrix4.Compose(new Vector3(x, 0, 0), Quaternion.Identity(), new Vector3(1, 1, 1));

    private static Harness Fallen(Matrix4? model = null)
    {
        var h = new Harness(model: model);
        h.Ragdoll.Hit("head", new Vector3(0, 0, 4), spread: 0);
        h.Run(0.5);
        h.Ragdoll.SetAll(RagdollMode.Animated);
        return h;
    }

    [Fact]
    public void AMixThatRunsOutInOneFrameEndsExactlyOnTheAnimatedPose()
    {
        using var h = Fallen();
        h.Modifier.Recover(0.1);

        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(0.25, h.Model));

        Near(1, h.Pose.HeadOf("hips").Y, 1e-6, "hips");
        Near(0, h.Pose.HeadOf("hips").X, 1e-6, "hips");
        Assert.False(h.Modifier.Recovering);
    }

    [Fact]
    public void HalfwayThroughTheMixTheHipsAreHalfwayBetweenTheFallAndTheAnimation()
    {
        using var h = Fallen();
        var fallen = h.Pose.HeadOf("hips").Clone();
        h.Modifier.Recover(1.0);

        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(0.5, h.Model));

        var animated = new Vector3(0, 1, 0);
        Near(0, ((fallen + animated) * 0.5 - h.Pose.HeadOf("hips")).Length, 1e-4, "hips");
    }

    [Fact]
    public void HalfwayThroughTheMixTheScaleIsHalfwayToo()
    {
        using var h = Fallen();
        h.Modifier.Recover(1.0);

        h.Pose.Reset();
        h.Pose["hips"].Scale.SetFrom(new Vector3(2, 2, 2));
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(0.5, h.Model));

        var scale = h.Pose["hips"].Scale;
        Near(1.5, scale.X, 1e-6, "x");
        Near(1.5, scale.Y, 1e-6, "y");
        Near(1.5, scale.Z, 1e-6, "z");
    }

    [Fact]
    public void AModelThatMovedSinceTheFallIsMixedFromWhereTheBonesWereInTheWorld()
    {
        using var h = Fallen(At(5));
        var fallen = h.Model.Transformed3(h.Pose.HeadOf("hips"));
        h.Modifier.Recover(1.0);

        h.Model = At(8);
        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(0.5, h.Model));

        var animated = At(8).Transformed3(new Vector3(0, 1, 0));
        Near(0, ((fallen + animated) * 0.5 - h.HeadOf("hips")).Length, 1e-4, "hips");
    }
}
