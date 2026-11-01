using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig.Tests;

public class RagdollSpecAddedTests
{
    [Fact]
    public void AHingeDefaultsToTheXAxisAndAQuarterTurnOfBend()
    {
        var hinge = new RagdollHinge();

        Assert.Equal(1, hinge.Axis.X);
        Assert.Equal(0, hinge.Axis.Y);
        Assert.Equal(0, hinge.Axis.Z);
        Assert.Equal(0, hinge.Low);
        Assert.Equal(2.4, hinge.High);
    }

    [Fact]
    public void AHingeKeepsALengthOfOneForItsAxis()
    {
        var hinge = new RagdollHinge(axis: new Vector3(0, 3, 0));

        Assert.Equal(1, hinge.Axis.Length, 1e-12);
        Assert.Equal(1, hinge.Axis.Y, 1e-12);
    }

    [Fact]
    public void AHingeKeepsACopyOfTheAxisItWasGiven()
    {
        var axis = new Vector3(0, 1, 0);
        var hinge = new RagdollHinge(axis: axis);

        axis.SetValues(1, 0, 0);

        Assert.Equal(1, hinge.Axis.Y);
    }

    [Fact]
    public void AHingeWithNoAxisIsRefusedForItsAxis()
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new RagdollHinge(axis: Vector3.Zero()));

        Assert.Equal("axis", error.ParamName);
    }

    [Fact]
    public void AHingeGoesFromLowToHighWithEitherEndAllowedToMeet()
    {
        Assert.Equal(0.5, new RagdollHinge(low: 0.5, high: 0.5).Low);
        Assert.Equal("low", Assert.Throws<ArgumentException>(() => new RagdollHinge(low: 1, high: 0.9)).ParamName);
        Assert.Throws<ArgumentException>(() => new RagdollHinge(low: double.NaN));
        Assert.Throws<ArgumentException>(() => new RagdollHinge(low: double.NegativeInfinity));
        Assert.Throws<ArgumentException>(() => new RagdollHinge(high: double.NaN));
        Assert.Throws<ArgumentException>(() => new RagdollHinge(high: double.PositiveInfinity));
    }

    [Fact]
    public void ABallDefaultsToASwingOfOneAndAHalfTurnOfTwist()
    {
        var ball = new RagdollBall();

        Assert.Equal(1.0, ball.Swing);
        Assert.Equal(-0.5, ball.TwistLow);
        Assert.Equal(0.5, ball.TwistHigh);
    }

    [Fact]
    public void ASwingIsFromNoughtToThreeRadiansWithBothEndsAllowed()
    {
        Assert.Equal(0, new RagdollBall(swing: 0).Swing);
        Assert.Equal(3, new RagdollBall(swing: 3).Swing);
        foreach (var swing in new[] { -0.01, 3.01, double.NaN, double.PositiveInfinity })
        {
            var error = Assert.Throws<ArgumentOutOfRangeException>(() => new RagdollBall(swing: swing));
            Assert.Equal("swing", error.ParamName);
        }
    }

    [Fact]
    public void ATwistGoesFromLowToHighWithEitherEndAllowedToMeet()
    {
        Assert.Equal(0.2, new RagdollBall(twistLow: 0.2, twistHigh: 0.2).TwistLow);
        Assert.Equal("twistLow", Assert.Throws<ArgumentException>(() => new RagdollBall(twistLow: 0.3, twistHigh: 0.2)).ParamName);
        Assert.Throws<ArgumentException>(() => new RagdollBall(twistLow: double.NaN));
        Assert.Throws<ArgumentException>(() => new RagdollBall(twistLow: double.NegativeInfinity));
        Assert.Throws<ArgumentException>(() => new RagdollBall(twistHigh: double.NaN));
        Assert.Throws<ArgumentException>(() => new RagdollBall(twistHigh: double.PositiveInfinity));
    }

    [Fact]
    public void ABodyEndsAtItsBoneByDefaultAndHasAMassOfOne()
    {
        var body = new RagdollBody("spine", radius: 0.1);

        Assert.Equal("spine", body.From);
        Assert.Equal(1.0, body.Mass);
        Assert.Equal(0.5, body.Friction);
        Assert.Null(body.Parent);
        Assert.Null(body.Joint);
    }

    [Fact]
    public void ABodyStartsWhereItIsToldTo()
    {
        Assert.Equal("hips", new RagdollBody("spine", radius: 0.1, @from: "hips").From);
    }

    [Fact]
    public void ARadiusAndAMassAreAboveNothingAndFinite()
    {
        foreach (var bad in new[] { 0.0, -1.0, double.NaN, double.PositiveInfinity })
        {
            Assert.Equal("radius", Assert.Throws<ArgumentOutOfRangeException>(() => new RagdollBody("a", radius: bad)).ParamName);
            Assert.Equal("mass", Assert.Throws<ArgumentOutOfRangeException>(() => new RagdollBody("a", radius: 1, mass: bad)).ParamName);
        }
    }

    [Fact]
    public void AParentAndAJointComeTogetherOrNotAtAll()
    {
        Assert.Equal("joint", Assert.Throws<ArgumentException>(() => new RagdollBody("a", radius: 1, parent: "b")).ParamName);
        Assert.Equal("joint", Assert.Throws<ArgumentException>(() => new RagdollBody("a", radius: 1, joint: new RagdollBall())).ParamName);
    }

    [Fact]
    public void ASpecRefusesNoBodiesAtAll()
    {
        Assert.Equal("bodies", Assert.Throws<ArgumentException>(() => new RagdollSpec([])).ParamName);
    }

    [Fact]
    public void ASpecNamesTheBodyWhoseParentComesLater()
    {
        var error = Assert.Throws<ArgumentException>(() => new RagdollSpec(
        [
            new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()),
            new RagdollBody("spine", radius: 0.1),
        ]));

        Assert.Contains("\"head\" hangs from \"spine\"", error.Message);
    }

    [Fact]
    public void ASpecRefusesTwoBodiesAtOneBone()
    {
        var error = Assert.Throws<ArgumentException>(() => new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1),
            new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()),
            new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()),
        ]));

        Assert.Contains("Two bodies end at bone \"head\"", error.Message);
    }

    [Fact]
    public void ASpecRefusesASecondRoot()
    {
        var error = Assert.Throws<ArgumentException>(() => new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1),
            new RagdollBody("head", radius: 0.1),
        ]));

        Assert.Contains("one root", error.Message);
    }

    [Fact]
    public void ASpecKeepsACopyOfTheBodiesItWasGiven()
    {
        var bodies = new List<RagdollBody> { new("spine", radius: 0.1) };
        var spec = new RagdollSpec(bodies);

        bodies.Add(new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()));

        Assert.Single(spec.Bodies);
    }

    [Fact]
    public void CheckNamesABoneTheArmatureLacks()
    {
        var spec = new RagdollSpec([new RagdollBody("tail", radius: 0.1)]);

        var error = Assert.Throws<ArmatureException>(() => spec.Check(Human.Rig()));

        Assert.Contains("No bone called \"tail\"", error.Message);
    }

    [Fact]
    public void CheckNamesAFirstBoneThatIsNotAboveTheLast()
    {
        var spec = new RagdollSpec([new RagdollBody("hips", radius: 0.1, @from: "head")]);

        var error = Assert.Throws<ArmatureException>(() => spec.Check(Human.Rig()));

        Assert.Contains("Bone \"head\" is not above \"hips\"", error.Message);
    }

    [Fact]
    public void CheckNamesTheBodiesThatCoverOneBoneTwice()
    {
        var spec = new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1, @from: "hips"),
            new RagdollBody("head", radius: 0.1, @from: "spine", parent: "spine", joint: new RagdollBall()),
        ]);

        var error = Assert.Throws<ArmatureException>(() => spec.Check(Human.Rig()));

        Assert.Contains("Bone \"spine\" is covered by the bodies ending at \"spine\" and \"head\"", error.Message);
    }

    [Fact]
    public void CheckRefusesAParentWithNoBoneAboveTheChildsFirstBone()
    {
        var spec = new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1, @from: "hips"),
            new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()),
            new RagdollBody("upper_arm.L", radius: 0.1, parent: "head", joint: new RagdollBall()),
        ]);

        var error = Assert.Throws<ArmatureException>(() => spec.Check(Human.Rig()));

        Assert.Contains("Body \"upper_arm.L\" hangs from \"head\"", error.Message);
    }

    [Fact]
    public void CheckPassesTheHumanSpec()
    {
        Human.Spec().Check(Human.Rig());
    }

    [Fact]
    public void ChainOfListsTheBonesFromTheFirstDownToTheLast()
    {
        var armature = Human.Rig();
        var body = new RagdollBody("head", radius: 0.1, @from: "hips");

        Assert.Equal(["hips", "spine", "head"], RagdollSpec.ChainOf(armature, body));
        Assert.Equal(["head"], RagdollSpec.ChainOf(armature, new RagdollBody("head", radius: 0.1)));
    }

    [Fact]
    public void TheTuningDefaultsAreTheDocumentedOnes()
    {
        var tuning = new RagdollTuning();

        Assert.Equal(8, tuning.PowerHertz);
        Assert.Equal(1, tuning.PowerRatio);
        Assert.Equal(0.1, tuning.LinearDamping);
        Assert.Equal(0.5, tuning.AngularDamping);
        Assert.Equal(2, tuning.SnapDistance);
    }
}
