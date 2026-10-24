using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollPoseAddedTests : IDisposable
{
    private readonly Harness _h = new();

    public void Dispose() => _h.Dispose();

    private void FallThenRecover(double seconds)
    {
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 4), spread: 0);
        _h.Run(0.5);
        _h.Ragdoll.SetAll(RagdollMode.Animated);
        _h.Modifier.Recover(seconds);
    }

    [Fact]
    public void RecoveringIsFalseAtTheStart()
    {
        Assert.False(_h.Modifier.Recovering);
        Assert.Same(_h.Ragdoll, _h.Modifier.Ragdoll);
    }

    [Fact]
    public void RecoverWithNoTimeOrNoFallDoesNothing()
    {
        _h.Modifier.Recover(0.4);
        Assert.False(_h.Modifier.Recovering);

        _h.Ragdoll.Hit("head", new Vector3(0, 0, 4), spread: 0);
        _h.Run(0.2);
        _h.Modifier.Recover(0);
        Assert.False(_h.Modifier.Recovering);
        _h.Modifier.Recover(-1);
        Assert.False(_h.Modifier.Recovering);
    }

    [Fact]
    public void RecoverWithNoTimeEndsARecoveryUnderway()
    {
        FallThenRecover(0.4);
        Assert.True(_h.Modifier.Recovering);

        _h.Modifier.Recover(0);

        Assert.False(_h.Modifier.Recovering);
    }

    [Fact]
    public void ARecoveryLastsTheSecondsGivenAndNoMore()
    {
        FallThenRecover(0.4);

        _h.Run(0.3);
        Assert.True(_h.Modifier.Recovering);
        _h.Run(0.15);
        Assert.False(_h.Modifier.Recovering);
    }

    [Fact]
    public void TheMixMovesTheHeadSteadilyFromTheFallToTheAnimation()
    {
        FallThenRecover(0.5);
        _h.Frame();
        var target = Human.Rig();
        var pose = new Pose(target);
        pose.Evaluate();
        var goal = pose.HeadOf("head");

        var distances = new List<double> { (_h.HeadOf("head") - goal).Length };
        for (var i = 0; i < 3; i++)
        {
            _h.Run(0.1);
            distances.Add((_h.HeadOf("head") - goal).Length);
        }

        Assert.True(distances[0] > distances[1], string.Join(", ", distances));
        Assert.True(distances[1] > distances[2], string.Join(", ", distances));
        Assert.True(distances[2] > distances[3], string.Join(", ", distances));
    }

    [Fact]
    public void AResetForgetsTheFallAndTheRecovery()
    {
        FallThenRecover(0.4);

        _h.Modifier.Reset();

        Assert.False(_h.Modifier.Recovering);
        _h.Modifier.Recover(0.4);
        Assert.False(_h.Modifier.Recovering);
    }

    [Fact]
    public void CheckPassesTheRigItWasBuiltFromAndRefusesAnother()
    {
        _h.Modifier.Check(Human.Rig());

        Assert.Throws<ArmatureException>(() => _h.Modifier.Check(new Armature()));
    }

    [Fact]
    public void ARecoveryFromAMovedModelMixesFromWhereTheBonesReallyWere()
    {
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 4), spread: 0);
        _h.Run(0.5);
        var fallen = _h.HeadOf("head");
        _h.Ragdoll.SetAll(RagdollMode.Animated);
        _h.Modifier.Recover(0.4);

        _h.Model = Matrix4.TranslationValues(5, 0, 0);
        _h.Pose.Reset();
        _h.Pose.Evaluate();
        _h.Modifier.Solve(_h.Pose, new ModifierFrame(Harness.Dt, _h.Model));

        // The first frame of the mix is the fallen pose, wherever the model went.
        Less((_h.HeadOf("head") - fallen).Length, 0.05);
    }
}
