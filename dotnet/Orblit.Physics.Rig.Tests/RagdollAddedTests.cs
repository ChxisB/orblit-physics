using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollAddedTests : IDisposable
{
    private readonly Harness _h = new();

    public void Dispose() => _h.Dispose();

    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    [Fact]
    public void NumbersItsBodiesFromTheFirstIdInSpecOrder()
    {
        using var other = new Harness();
        var ragdoll = Ragdoll.Build(
            other.Physics,
            other.Pose,
            Human.Spec(),
            Matrix4.Identity(),
            firstId: 500,
            layer: 0x400);

        var ids = ragdoll.Bones.Select(ragdoll.IdOf).ToList();

        Assert.Equal(Enumerable.Range(500, ids.Count).Select(i => (ulong)i), ids);
        Assert.Equal("spine", ragdoll.Bones[0]);
        ragdoll.Dispose();
    }

    [Fact]
    public void TakesItsLayerAndItsIgnoredLayersOutOfWhatItCaresAbout()
    {
        using var other = new Harness();
        var ragdoll = Ragdoll.Build(
            other.Physics,
            other.Pose,
            Human.Spec(),
            Matrix4.Identity(),
            firstId: 500,
            layer: 0x400,
            ignore: 0x8);

        Assert.Equal(0x400u, ragdoll.Layer);
        Assert.Equal(0x400u, ragdoll.Layers.Is);
        Assert.Equal(0xFFFFFFFFu & ~0x400u & ~0x8u, ragdoll.Layers.Cares);
        ragdoll.Dispose();
    }

    [Fact]
    public void AllowsEachSingleBitLayerAndRefusesNoneAndTwo()
    {
        foreach (var layer in new[] { 1u, 0x80000000u })
        {
            using var other = new Harness();
            Ragdoll.Build(other.Physics, other.Pose, Human.Spec(), Matrix4.Identity(), firstId: 500, layer: layer).Dispose();
        }

        foreach (var layer in new[] { 0u, 3u, 0x101u })
        {
            var error = Assert.Throws<ArgumentOutOfRangeException>(() =>
                Ragdoll.Build(_h.Physics, _h.Pose, Human.Spec(), Matrix4.Identity(), firstId: 500, layer: layer));
            Assert.Equal("layer", error.ParamName);
        }
    }

    [Fact]
    public void ABoneWithNoBodyOfItsOwnResolvesToTheBodyAboveIt()
    {
        Assert.Equal(_h.Ragdoll.IdOf("spine"), _h.Ragdoll.IdOf("hips"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("hips"));
        Assert.NotEqual(_h.Ragdoll.IdOf("thigh.L"), _h.Ragdoll.IdOf("shin.L"));
    }

    [Fact]
    public void AnUnknownBoneIsRefusedByName()
    {
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => _h.Ragdoll.ModeOf("nowhere"));

        Assert.Equal("bone", error.ParamName);
    }

    [Fact]
    public void KeepsTheTuningItWasGivenAndTheDefaultOtherwise()
    {
        var tuning = new RagdollTuning(PowerHertz: 3);
        using var tuned = new Harness(tuning);

        Assert.Same(tuning, tuned.Ragdoll.Tuning);
        Assert.Equal(new RagdollTuning(), _h.Ragdoll.Tuning);
    }

    [Fact]
    public void ASmallerSnapDistancePutsABodyWhereTheDefaultCarriesIt()
    {
        using var snapping = new Harness(new RagdollTuning(SnapDistance: 0.5));
        snapping.Model = Matrix4.TranslationValues(1, 0, 0);
        _h.Model = Matrix4.TranslationValues(1, 0, 0);

        snapping.Frame();
        _h.Frame();

        // Put there with no speed, against carried with the speed that reaches it.
        Assert.Equal(1, snapping.BodyOf("spine").X, 1e-3);
        var carried = _h.Physics.VelocityOf(_h.Ragdoll.IdOf("spine"))!;
        var placed = snapping.Physics.VelocityOf(snapping.Ragdoll.IdOf("spine"))!;
        Less(Math.Abs(placed[0]), 0.1);
        Greater(Math.Abs(carried[0]), 10);
    }

    [Fact]
    public void ASnapDistanceIsMeasuredInTheWorldsUnits()
    {
        _h.Model = Matrix4.Compose(new Vector3(3, 0, 0), Quaternion.Identity(), new Vector3(2, 2, 2));
        _h.Ragdoll.Snap(_h.Pose, new ModifierFrame(modelToWorld: _h.Model));

        // A 3 unit step is under the 2 model units a double scale lets through.
        _h.Model = Matrix4.Compose(new Vector3(3, 0, 0), Quaternion.Identity(), new Vector3(2, 2, 2));
        _h.Frame();

        Less(Math.Abs(_h.BodyOf("spine").X - 3), 0.05);
    }

    [Fact]
    public void AHitAtTheTipTurnsTheForearmFasterThanAHitAtItsCentre()
    {
        var id = _h.Ragdoll.IdOf("forearm.L");
        using var centre = new Harness();
        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 5), new Vector3(0.75, 1.4, 0), spread: 0);
        centre.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 5), spread: 0);
        _h.Physics.Step(Harness.Dt);
        centre.Physics.Step(Harness.Dt);

        var tip = _h.Physics.VelocityOf(id)!;
        var middle = centre.Physics.VelocityOf(id)!;

        // The elbow only bends about x, so a push across the arm turns it about y,
        // twice as hard from twice as far along.
        Greater(Math.Abs(tip[4]), Math.Abs(middle[4]) * 1.5);
        Greater(Math.Abs(middle[4]), 1);
    }

    [Fact]
    public void ANearerHitBringsAPendingLimpTimeForward()
    {
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 1.0);
        _h.Ragdoll.Hit("spine", new Vector3(0, 0, 1), spread: 0.05);

        _h.Run(0.2);

        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("upper_arm.L"));
    }

    [Fact]
    public void AFartherHitDoesNotPutAPendingLimpTimeBack()
    {
        _h.Ragdoll.Hit("spine", new Vector3(0, 0, 1), spread: 0.1);
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 1.0);

        _h.Run(0.25);

        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("upper_arm.L"));
    }

    [Fact]
    public void SettingAModeCancelsAPendingLimpTime()
    {
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 0.5);
        _h.Ragdoll.SetMode("spine", RagdollMode.Animated);

        _h.Run(1);

        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void ABodyLimpAlreadyIsNotMadeLimpAgainByAHitBelowIt()
    {
        _h.Ragdoll.SetMode("upper_arm.L", RagdollMode.Limp);
        _h.Run(0.2);

        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 1), spread: 1.0);
        _h.Run(0.1);

        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("upper_arm.L"));
    }

    [Fact]
    public void SetAllMakesEveryBodyTheMode()
    {
        _h.Ragdoll.SetAll(RagdollMode.Limp);
        foreach (var bone in _h.Ragdoll.Bones)
        {
            Assert.True(_h.Ragdoll.ModeOf(bone) == RagdollMode.Limp, bone);
        }

        _h.Ragdoll.SetAll(RagdollMode.Powered);
        foreach (var bone in _h.Ragdoll.Bones.Skip(1))
        {
            Assert.True(_h.Ragdoll.ModeOf(bone) == RagdollMode.Powered, bone);
        }

        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void SetFromReachesEveryBodyBelowAndNoneBesideOrAbove()
    {
        _h.Ragdoll.SetFrom("thigh.L", RagdollMode.Limp);

        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("thigh.L"));
        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("shin.L"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("thigh.R"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void APoweredLimbLetsGoWhenMadeLimpAndFalls()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Powered);
        _h.Run(0.5);
        var held = _h.BodyOf("forearm.L").Y;
        Assert.Equal(1.4, held, 0.1);

        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);
        _h.Run(1);

        Less(_h.BodyOf("forearm.L").Y, held - 0.3);
    }

    [Fact]
    public void ALimpLimbMadePoweredAgainIsHeldOnThePoseAgain()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);
        _h.Run(0.3);

        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Powered);
        _h.Run(2);

        Assert.Equal(1.4, _h.BodyOf("forearm.L").Y, 0.1);
    }

    [Fact]
    public void AFreeBodyKeepsItsVelocityThroughTheMakingOfItsJoint()
    {
        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 5), spread: 0);
        _h.Run(0.1);
        var id = _h.Ragdoll.IdOf("forearm.L");
        var before = _h.Physics.VelocityOf(id)!;

        _h.Ragdoll.SetMode("forearm.L", RagdollMode.Powered);
        var after = _h.Physics.VelocityOf(id)!;

        for (var i = 0; i < 6; i++)
        {
            Assert.Equal(before[i], after[i], 1e-5);
        }
    }

    [Fact]
    public void IsAtRestIsTrueWhileNothingIsFree()
    {
        Assert.True(_h.Ragdoll.IsAtRest());
    }

    [Fact]
    public void IsAtRestIsFalseWhileABodyMovesFasterThanTheSpeedGiven()
    {
        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 5), spread: 0);
        _h.Physics.Step(Harness.Dt);

        Assert.False(_h.Ragdoll.IsAtRest());
        Assert.True(_h.Ragdoll.IsAtRest(speed: 100));
    }

    [Fact]
    public void IsAtRestAlsoWatchesTheSpin()
    {
        // A light blow at the tip is slow in a line but spins the forearm fast.
        _h.Ragdoll.Hit("forearm.L", new Vector3(0, 0, 0.45), new Vector3(0.75, 1.4, 0), spread: 0);
        _h.Physics.Step(Harness.Dt);
        var velocity = _h.Physics.VelocityOf(_h.Ragdoll.IdOf("forearm.L"))!;
        var line = Math.Sqrt((velocity[0] * velocity[0]) + (velocity[1] * velocity[1]) + (velocity[2] * velocity[2]));
        var spin = Math.Sqrt((velocity[3] * velocity[3]) + (velocity[4] * velocity[4]) + (velocity[5] * velocity[5]));

        Assert.False(_h.Ragdoll.IsAtRest(speed: Math.Max(line, spin / 4) * 0.5));
        Assert.True(_h.Ragdoll.IsAtRest(speed: Math.Max(line, spin / 4) * 2));
    }

    [Fact]
    public void RootPositionIsWhereTheRootBodyIs()
    {
        var spine = _h.BodyOf("spine");

        Assert.Equal(spine.X, _h.Ragdoll.RootPosition.X, 1e-6);
        Assert.Equal(spine.Y, _h.Ragdoll.RootPosition.Y, 1e-6);
        Assert.Equal(spine.Z, _h.Ragdoll.RootPosition.Z, 1e-6);
    }

    [Fact]
    public void DisposeTakesEveryBodyOutOfTheWorld()
    {
        var ids = _h.Ragdoll.Bones.Select(_h.Ragdoll.IdOf).ToList();

        _h.Ragdoll.Dispose();

        Assert.All(ids, id => Assert.Null(_h.Physics.TransformOf(id)));
        Assert.Throws<InvalidOperationException>(() => _h.Ragdoll.RootPosition);
    }

    [Fact]
    public void BonesAreListedRootFirstInSpecOrder()
    {
        Assert.Equal(
            Human.Spec().Bodies.Select(body => body.Bone),
            _h.Ragdoll.Bones);
    }

    [Fact]
    public void AFrameOfNoTimeOnlyReadsThePose()
    {
        _h.Ragdoll.Hit("head", new Vector3(0, 0, 1), spread: 0.01);

        _h.Pose.Evaluate();
        _h.Ragdoll.FollowPose(_h.Pose, new ModifierFrame(0, _h.Model));
        _h.Ragdoll.FollowPose(_h.Pose, new ModifierFrame(0, _h.Model));

        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("spine"));
    }

    [Fact]
    public void WritePoseLeavesBonesWithNoFreeBodyAlone()
    {
        _h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);
        _h.Run(1);

        Assert.Equal(1.15, _h.Pose.HeadOf("spine").Y, 1e-3);
        Assert.Equal(0.15, _h.Pose.HeadOf("upper_arm.R").X * -1, 1e-3);
    }

    [Fact]
    public void ALimpRootIsDrawnWithItsBodyHalfWayBetweenTheEndsOfItsBones()
    {
        _h.Ragdoll.Hit("spine", new Vector3(0, 0, 8), spread: 0);
        _h.Run(0.5);

        _h.Pose.Reset();
        _h.Pose.Evaluate();
        _h.Modifier.Solve(_h.Pose, new ModifierFrame(Harness.Dt, _h.Model));

        var middle = (_h.Pose.HeadOf("hips") + _h.Pose.TailOf("spine")) * 0.5;
        Less((middle - _h.BodyOf("spine")).Length, 1e-3);
        Greater((_h.Pose.HeadOf("hips") - _h.Pose.TailOf("spine")).Length, 0.4);
    }

    [Fact]
    public void ThePoseAnimationOfABoneWithNoBodyIsKeptBelowAFreeBody()
    {
        _h.Animate = pose => pose["thigh.L"].Rotation = About(1, 0, 0, 0.5);
        _h.Ragdoll.SetMode("spine", RagdollMode.Limp);

        _h.Run(0.1);

        Assert.Equal(0.45, (_h.Pose.TailOf("thigh.L") - _h.Pose.HeadOf("thigh.L")).Length, 1e-3);
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("thigh.L"));
    }
}
