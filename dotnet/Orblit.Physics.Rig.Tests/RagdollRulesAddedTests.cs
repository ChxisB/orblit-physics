using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollRulesAddedTests
{
    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    private static Harness Weightless()
    {
        var h = new Harness();
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        return h;
    }

    [Fact]
    public void NothingIsFreeUntilABodyIsLimpOrAPoweredLimb()
    {
        using var h = new Harness();
        Assert.False(h.Ragdoll.HasFree);

        h.Ragdoll.SetMode("head", RagdollMode.Limp);
        Assert.True(h.Ragdoll.HasFree);
    }

    [Fact]
    public void APoweredLimbIsFree()
    {
        using var h = new Harness();

        h.Ragdoll.SetMode("forearm.L", RagdollMode.Powered);

        Assert.True(h.Ragdoll.HasFree);
    }

    [Fact]
    public void ARootThatIsPoweredStaysAnimatedAndNothingIsFree()
    {
        using var h = new Harness();

        h.Ragdoll.SetMode("spine", RagdollMode.Powered);

        Assert.Equal(RagdollMode.Animated, h.Ragdoll.ModeOf("spine"));
        Assert.False(h.Ragdoll.HasFree);
    }

    [Fact]
    public void ABodyHitWithASpreadGoesLimpWhenItsTurnComesOnTheClock()
    {
        using var h = new Harness();
        h.Run(1.0);

        h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 0.5);
        h.Run(0.2);
        Assert.Equal(RagdollMode.Animated, h.Ragdoll.ModeOf("spine"));

        h.Run(0.5);
        Assert.Equal(RagdollMode.Limp, h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void ABodyWhoseTurnIsExactlyThisFrameGoesLimpInIt()
    {
        using var h = new Harness();

        h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: Harness.Dt);
        Assert.Equal(RagdollMode.Animated, h.Ragdoll.ModeOf("spine"));
        h.Frame();

        Assert.Equal(RagdollMode.Limp, h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void ASpreadOfNoughtMakesEveryBodyLimpAtOnce()
    {
        using var h = new Harness();

        h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 0);

        foreach (var bone in h.Ragdoll.Bones)
        {
            Assert.Equal(RagdollMode.Limp, h.Ragdoll.ModeOf(bone));
        }
    }

    [Fact]
    public void ASecondHitDoesNotPushALaterTurnBackOut()
    {
        using var h = new Harness();
        h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 0.1);
        h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 1.0);

        h.Run(0.2);

        Assert.Equal(RagdollMode.Limp, h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void AFrameOfNoTimeLeavesTheBodiesAlone()
    {
        var moved = Matrix4.Compose(new Vector3(0.1, 0, 0), Quaternion.Identity(), new Vector3(1, 1, 1));
        using var h = new Harness();

        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(0, moved));

        foreach (var speed in h.Physics.VelocityOf(h.Ragdoll.IdOf("spine"))!)
        {
            Assert.Equal(0, speed);
        }
    }

    [Fact]
    public void ABoneBelowTheLastOneACapsuleCoversTakesItsParentsBody()
    {
        var armature = new Armature(
        [
            new Bone("a", new Vector3(0, 1, 0), new Vector3(1, 1, 0)),
            new Bone("b", new Vector3(1, 1, 0), new Vector3(2, 1, 0), parent: "a", connected: true),
            new Bone("c", new Vector3(2, 1, 0), new Vector3(3, 1, 0), parent: "b", connected: true),
        ]);
        using var h = new Harness(
            spec: new RagdollSpec([new RagdollBody("b", radius: 0.05, @from: "a")]),
            armature: armature);

        Assert.Equal(h.Ragdoll.IdOf("b"), h.Ragdoll.IdOf("c"));
        Assert.Equal(h.Ragdoll.IdOf("b"), h.Ragdoll.IdOf("a"));
    }

    [Fact]
    public void ALimpBodyOverATwistedRunOfBonesWritesTheBonesBackWhereTheyWere()
    {
        var armature = new Armature(
        [
            new Bone("a", new Vector3(0, 2, 0), new Vector3(1, 2, 0)),
            new Bone("b", new Vector3(1, 2, 0), new Vector3(1, 3, 1), parent: "a", connected: true),
        ]);
        using var h = new Harness(
            spec: new RagdollSpec([new RagdollBody("b", radius: 0.1, @from: "a")]),
            armature: armature);
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        h.Ragdoll.SetMode("b", RagdollMode.Limp);

        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(Harness.Dt, h.Model));

        Near(0, (new Vector3(0, 2, 0) - h.Pose.HeadOf("a")).Length, 1e-4, "head of a");
        Near(0, (new Vector3(1, 2, 0) - h.Pose.TailOf("a")).Length, 1e-4, "tail of a");
        Near(0, (new Vector3(1, 3, 1) - h.Pose.TailOf("b")).Length, 1e-4, "tail of b");
    }

    [Fact]
    public void ALimbHeldByAPowerIsPulledToTheBendThePoseGivesIt()
    {
        using var h = Weightless();
        h.Ragdoll.SetMode("forearm.L", RagdollMode.Powered);
        h.Animate = pose => pose["forearm.L"].Rotation = About(1, 0, 0, 0.8);

        h.Run(1.0);

        Near(0.8, Math.Abs(h.Physics.JointStateOf(h.Ragdoll.IdOf("forearm.L"))!.Angles[0]), 0.1, "elbow");
    }

    [Fact]
    public void LettingAnAnimatedLimbGoLimpLeavesItWhereItWas()
    {
        using var h = Weightless();
        h.Animate = pose => pose["forearm.L"].Rotation = About(1, 0, 0, 0.8);
        h.Run(0.3);
        var before = h.Physics.TransformOf(h.Ragdoll.IdOf("forearm.L"))!;

        h.Ragdoll.SetMode("forearm.L", RagdollMode.Limp);

        var after = h.Physics.TransformOf(h.Ragdoll.IdOf("forearm.L"))!;
        for (var i = 0; i < before.Length; i++)
        {
            Assert.Equal(before[i], after[i], 1e-5);
        }
    }

    [Fact]
    public void ACarriedBodyTurnsAtTheRateTheModelTurns()
    {
        using var h = Weightless();
        for (var i = 1; i <= 10; i++)
        {
            h.Model = Matrix4.Compose(Vector3.Zero(), About(0, 1, 0, 0.05 * i), new Vector3(1, 1, 1));
            h.Frame();
        }

        Near(0.05 / Harness.Dt, h.Physics.VelocityOf(h.Ragdoll.IdOf("spine"))![4], 0.1, "turn rate");
    }
}
