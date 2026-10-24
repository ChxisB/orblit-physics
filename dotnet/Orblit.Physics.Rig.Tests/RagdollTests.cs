using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollTests : IDisposable
{
    private readonly Harness _h = new();

    public void Dispose() => _h.Dispose();

    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    private void AssertEveryBody(RagdollMode mode)
    {
        foreach (var bone in _h.Ragdoll.Bones)
        {
            Assert.True(_h.Ragdoll.ModeOf(bone) == mode, bone);
        }
    }

    [Fact]
    public void StandsABodyForEachSpecEntryWhereTheRigRests()
    {
        Assert.Equal(Human.Spec().Bodies.Count, _h.Ragdoll.Bones.Count);
        var torso = _h.BodyOf("spine");
        // The torso runs from the hips' head to the spine's tail.
        Assert.Equal((1.0 + 1.45) / 2, torso.Y, 1e-4);
        var forearm = _h.BodyOf("forearm.L");
        Assert.Equal(0.6, forearm.X, 1e-4);
        Assert.Equal(1.4, forearm.Y, 1e-4);
    }

    [Fact]
    public void StandsWhereTheModelIs()
    {
        using var moved = new Harness();
        var model = Matrix4.TranslationValues(2, 0, -3);
        moved.Model = model;

        moved.Ragdoll.Snap(moved.Pose, new ModifierFrame(modelToWorld: model));

        Assert.Equal(2, moved.BodyOf("spine").X, 1e-4);
        Assert.Equal(-3, moved.BodyOf("spine").Z, 1e-4);
    }

    [Fact]
    public void HasEveryBodyAnimatedAndOutOfTheCharacterLayer()
    {
        AssertEveryBody(RagdollMode.Animated);
        Assert.Equal(0x100u, _h.Ragdoll.Layers.Is);
        Assert.Equal(0u, _h.Ragdoll.Layers.Cares & Human.CharacterLayer);
        Assert.Equal(0u, _h.Ragdoll.Layers.Cares & 0x100u);
    }

    [Fact]
    public void NamesBodiesFromFirstIdAndResolvesAnUncoveredBone()
    {
        Assert.Equal(_h.Ragdoll.IdOf("spine"), _h.Ragdoll.IdOf("hips"));
        Assert.Equal(1UL, _h.Ragdoll.IdOf("spine"));
        Assert.ThrowsAny<ArgumentException>(() => _h.Ragdoll.IdOf("nowhere"));
    }

    [Fact]
    public void RefusesABoneTheArmatureLacks()
    {
        var spec = new RagdollSpec([new RagdollBody("tail", radius: 0.1)]);

        Assert.Throws<ArmatureException>(() =>
            Ragdoll.Build(_h.Physics, _h.Pose, spec, Matrix4.Identity(), firstId: 500));
    }

    [Fact]
    public void RefusesALayerThatIsNotOneBit()
    {
        Assert.ThrowsAny<ArgumentException>(() =>
            Ragdoll.Build(_h.Physics, _h.Pose, Human.Spec(), Matrix4.Identity(), firstId: 500, layer: 0x300));
    }

    [Fact]
    public void NeedsParentsBeforeChildrenAndOneRoot()
    {
        Assert.ThrowsAny<ArgumentException>(() => new RagdollSpec(
        [
            new RagdollBody("head", radius: 0.1, parent: "spine", joint: new RagdollBall()),
            new RagdollBody("spine", radius: 0.1),
        ]));
        Assert.ThrowsAny<ArgumentException>(() => new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1),
            new RagdollBody("hips", radius: 0.1),
        ]));
    }

    [Fact]
    public void NeedsAParentAndAJointTogether()
    {
        Assert.ThrowsAny<ArgumentException>(() => new RagdollBody("head", radius: 0.1, parent: "spine"));
    }

    [Fact]
    public void RefusesABodyWhoseFirstBoneIsNotAboveItsLast()
    {
        var spec = new RagdollSpec([new RagdollBody("hips", radius: 0.1, @from: "head")]);

        Assert.Throws<ArmatureException>(() => spec.Check(_h.Armature));
    }

    [Fact]
    public void RefusesTwoBodiesOverOneBone()
    {
        var spec = new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.1, @from: "hips"),
            new RagdollBody("head", radius: 0.1, @from: "spine", parent: "spine", joint: new RagdollBall()),
        ]);

        Assert.Throws<ArmatureException>(() => spec.Check(_h.Armature));
    }

    [Fact]
    public void AnAnimatedBodyBelowAFreeOneHoldsItsEndInPlace()
    {
        _h.Ragdoll.SetMode("upper_arm.L", RagdollMode.Limp);

        _h.Run(1);

        Assert.Equal(1.4, _h.BodyOf("upper_arm.L").Y, 0.02);
    }

    [Fact]
    public void CarriesAnimatedBodiesOntoThePoseInOneStep()
    {
        _h.Animate = pose => pose["upper_arm.L"].Rotation = About(0, 0, 1, 0.6);

        _h.Frame();
        _h.Frame();

        var head = _h.HeadOf("upper_arm.L");
        var tail = _h.TailOf("upper_arm.L");
        var centre = (head + tail) * 0.5;
        Less((_h.BodyOf("upper_arm.L") - centre).Length, 1e-3);
        // The arm really did move, or the test would pass by standing still.
        Greater(Math.Abs(tail.Y - 1.4), 0.05);
    }

    [Fact]
    public void PutsABodyThatIsFarOffStraightOntoThePose()
    {
        _h.Model = Matrix4.TranslationValues(10, 0, 0);

        _h.Frame();

        Assert.Equal(10, _h.BodyOf("spine").X, 1e-3);
    }

    [Fact]
    public void LeavesThePoseAloneWhileEveryBodyIsAnimated()
    {
        _h.Frame();

        Assert.Equal(1.15, _h.Pose.HeadOf("spine").Y, 1e-6);
        Assert.False(_h.Ragdoll.HasFree);
    }

    [Fact]
    public void AHitSpreadsOutwardFromTheBoneThatWasStruck()
    {
        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 5), spread: 0.1);
        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("forearm.L"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("upper_arm.L"));

        _h.Run(0.12);
        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("upper_arm.L"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));

        _h.Run(0.12);
        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("spine"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("head"));

        _h.Run(0.4);
        AssertEveryBody(RagdollMode.Limp);
    }

    [Fact]
    public void ASpreadOfNoughtIsLimpEverywhereAtOnce()
    {
        _h.Ragdoll.Hit("head", new Vector3(1, 0, 0), spread: 0);

        AssertEveryBody(RagdollMode.Limp);
    }

    [Fact]
    public void AFreeArmFallsAndTheOtherArmStaysOnThePose()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);

        _h.Run(1);

        Less(_h.Pose.TailOf("upper_arm.L").Y, 1.25);
        Assert.Equal(1.4, _h.Pose.TailOf("upper_arm.R").Y, 1e-3);
        Assert.Equal(1.15, _h.Pose.HeadOf("spine").Y, 1e-3);
    }

    [Fact]
    public void TheArmBelowALimpArmTravelsWithIt()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);

        _h.Run(1);

        var elbow = _h.Pose.TailOf("upper_arm.L");
        var wrist = _h.Pose.TailOf("forearm.L");
        Assert.Equal(0.3, (wrist - _h.Pose.HeadOf("forearm.L")).Length, 1e-3);
        Less((_h.Pose.HeadOf("forearm.L") - elbow).Length, 1e-3);
    }

    [Fact]
    public void BonesKeepTheirLengthsHoweverFarTheBodyGoes()
    {
        _h.Ragdoll.Hit("spine", new Vector3(30, 20, 0), spread: 0);

        _h.Run(1.5);

        foreach (var bone in _h.Armature.Bones)
        {
            var length = (_h.Pose.TailOf(bone.Name) - _h.Pose.HeadOf(bone.Name)).Length;
            Near(bone.Length, length, 1e-3, bone.Name);
        }
    }

    [Fact]
    public void ATorsoOverTwoBonesKeepsTheBoneBetweenItsEnds()
    {
        _h.Ragdoll.SetMode("spine", RagdollMode.Limp);

        _h.Run(1);

        var hips = _h.Pose.TailOf("hips");
        Less((_h.Pose.HeadOf("spine") - hips).Length, 1e-3);
    }

    [Fact]
    public void ALimpRagdollComesToRestOnTheGround()
    {
        _h.Ragdoll.Hit("spine", new Vector3(0, 0, 8), spread: 0.03);

        // The tumble is still settling at 3 s, and where a platform's rounding
        // sends it decides whether it has stopped by then. By 5 s it is asleep.
        _h.Run(6);

        Assert.True(_h.Ragdoll.IsAtRest());
        foreach (var bone in _h.Ragdoll.Bones)
        {
            var y = _h.BodyOf(bone).Y;
            Assert.True(double.IsFinite(y), bone);
            // A capsule's centre cannot be lower than its radius.
            Greater(y, 0.03, bone);
            Less(y, 0.6, bone);
        }
    }

    [Fact]
    public void AHingeHoldsTheAngleThePoseBendsItTo()
    {
        _h.Animate = pose => pose["forearm.L"].Rotation = About(1, 0, 0, 0.9);
        _h.Ragdoll.SetMode("forearm.L", RagdollMode.Powered);

        _h.Run(1.5);

        var state = _h.Physics.JointStateOf(_h.Ragdoll.IdOf("forearm.L"))!;
        Assert.Equal(0.9, state.Angles[0], 0.1);
    }

    [Fact]
    public void ABallHoldsTheSwingThePoseGivesIt()
    {
        _h.Animate = pose => pose["upper_arm.L"].Rotation = About(0, 0, 1, 0.5);
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Powered);

        _h.Run(1.5);

        var state = _h.Physics.JointStateOf(_h.Ragdoll.IdOf("upper_arm.L"))!;
        Assert.Equal(0.5, state.Angles[2], 0.1);
        Assert.Equal(0, state.Angles[0], 0.1);
    }

    [Fact]
    public void AHingeStaysInsideItsLimitsWhateverThePoseAsks()
    {
        _h.Animate = pose => pose["forearm.L"].Rotation = About(1, 0, 0, -1.0);
        _h.Ragdoll.SetMode("forearm.L", RagdollMode.Powered);

        _h.Run(1);

        var state = _h.Physics.JointStateOf(_h.Ragdoll.IdOf("forearm.L"))!;
        Greater(state.Angles[0], -0.05);
    }

    [Fact]
    public void APoweredBodyGivesUnderABlowAndComesBack()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Powered);
        _h.Run(0.5);
        var held = _h.BodyOf("forearm.L").Y;

        _h.Physics.Push(_h.Ragdoll.IdOf("forearm.L"), [0.0, -6.0, 0.0]);
        _h.Run(0.1);
        Less(_h.BodyOf("forearm.L").Y, held - 0.02);

        _h.Run(2);
        Assert.Equal(held, _h.BodyOf("forearm.L").Y, 0.1);
    }

    [Fact]
    public void APoweredRootIsAnimated()
    {
        _h.Ragdoll.SetMode("spine", RagdollMode.Powered);

        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));
    }
}
