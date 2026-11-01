using Orblit.Rig;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollSpecChecksAddedTests
{
    [Theory]
    [InlineData("ghost", "forearm.L")]
    [InlineData("upper_arm.L", "ghost")]
    public void AChainNamesTheBoneTheArmatureLacksWhicheverEndItIs(string from, string bone)
    {
        var body = new RagdollBody(bone, radius: 0.05, @from: from);

        var thrown = Assert.Throws<ArmatureException>(() => RagdollSpec.ChainOf(Human.Rig(), body));

        Assert.Contains("No bone called \"ghost\"", thrown.Message);
    }

    [Fact]
    public void ABodyMayHangFromABoneFurtherUpThanTheOneBesideIt()
    {
        var spec = new RagdollSpec(
        [
            new RagdollBody("hips", radius: 0.1),
            new RagdollBody("head", radius: 0.1, parent: "hips", joint: new RagdollBall()),
        ]);

        spec.Check(Human.Rig());
    }

    [Fact]
    public void ABodyCannotHangFromABodyThatCoversNothingAboveIt()
    {
        var spec = new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.14, @from: "spine"),
            new RagdollBody("thigh.L", radius: 0.07, parent: "spine", joint: new RagdollBall()),
        ]);

        var thrown = Assert.Throws<ArmatureException>(() => spec.Check(Human.Rig()));

        Assert.Contains("which has no bone above", thrown.Message);
    }
}
