using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollSpaceAddedTests
{
    private static readonly Matrix4 Moved = Matrix4.Compose(
        new Vector3(1, 0.5, -2),
        Quaternion.AxisAngle(new Vector3(1, 2, 0.5).Normalized(), 0.8),
        new Vector3(1.5, 1.5, 1.5));

    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    private static bool Touches(Harness h, string bone, Vector3 point) =>
        h.Physics.Overlap([point.X, point.Y, point.Z], Shape.Sphere(0.01)).Contains(h.Ragdoll.IdOf(bone));

    private static Vector3 At(Harness h, ulong id)
    {
        var t = h.Physics.TransformOf(id)!;
        return new Vector3(t[0], t[1], t[2]);
    }

    private static void Same(Vector3 expected, Vector3 actual, double tolerance)
    {
        Near(expected.X, actual.X, tolerance, "x");
        Near(expected.Y, actual.Y, tolerance, "y");
        Near(expected.Z, actual.Z, tolerance, "z");
    }

    [Fact]
    public void ACapsuleRunsFromTheHeadOfItsFirstBoneToTheTailOfItsLast()
    {
        using var h = new Harness();

        Assert.True(Touches(h, "spine", new Vector3(0, 1.44, 0)));
        Assert.True(Touches(h, "spine", new Vector3(0, 1.01, 0)));
        Assert.False(Touches(h, "spine", new Vector3(0, 1.5, 0)));
        Assert.False(Touches(h, "spine", new Vector3(0, 0.95, 0)));
    }

    [Fact]
    public void ACapsuleIsAsThickAsItsRadius()
    {
        using var h = new Harness();

        Assert.True(Touches(h, "spine", new Vector3(0.13, 1.225, 0)));
        Assert.False(Touches(h, "spine", new Vector3(0.16, 1.225, 0)));
    }

    [Fact]
    public void AScaledModelScalesTheLengthAndTheThicknessOfEveryCapsule()
    {
        var model = Matrix4.Compose(new Vector3(1, 0, 0), Quaternion.Identity(), new Vector3(2, 2, 2));
        using var h = new Harness(model: model);

        Assert.True(Touches(h, "spine", new Vector3(1.25, 2.45, 0)));
        Assert.False(Touches(h, "spine", new Vector3(1.31, 2.45, 0)));
        Assert.True(Touches(h, "spine", new Vector3(1, 2.85, 0)));
        Assert.False(Touches(h, "spine", new Vector3(1, 2.95, 0)));
        Assert.True(Touches(h, "spine", new Vector3(1, 2.05, 0)));
        Assert.False(Touches(h, "spine", new Vector3(1, 1.95, 0)));
    }

    [Fact]
    public void ABodyStandsWhereTheModelPutsItsBones()
    {
        using var h = new Harness(model: Moved);

        foreach (var (bone, centre) in new[]
        {
            ("spine", new Vector3(0, 1.225, 0)),
            ("forearm.L", new Vector3(0.6, 1.4, 0)),
            ("shin.R", new Vector3(-0.1, 0.3, 0)),
        })
        {
            Same(Moved.Transformed3(centre), At(h, h.Ragdoll.IdOf(bone)), 1e-4);
        }
    }

    [Fact]
    public void EveryJointStartsAtRestInAMovedRotatedAndScaledModel()
    {
        using var h = new Harness(model: Moved);

        foreach (var bone in h.Ragdoll.Bones.Skip(1))
        {
            var state = h.Physics.JointStateOf(h.Ragdoll.IdOf(bone))!;
            Less(Math.Abs(state.Offset[0]) + Math.Abs(state.Offset[1]) + Math.Abs(state.Offset[2]), 1e-3, bone);
            Less(Math.Abs(state.Angles[0]) + Math.Abs(state.Angles[1]) + Math.Abs(state.Angles[2]), 1e-3, bone);
        }
    }

    [Fact]
    public void TheJointsStayAtRestWhileTheModelWalksAboutAndTheBodiesAreCarried()
    {
        using var h = new Harness(model: Moved);

        for (var i = 1; i <= 30; i++)
        {
            h.Model = Matrix4.Compose(
                new Vector3(1 + (0.01 * i), 0.5, -2),
                About(1, 2, 0.5, 0.8 + (0.01 * i)),
                new Vector3(1.5, 1.5, 1.5));
            h.Frame();
        }

        Same(h.Model.Transformed3(new Vector3(0.6, 1.4, 0)), At(h, h.Ragdoll.IdOf("forearm.L")), 0.02);
        foreach (var bone in h.Ragdoll.Bones.Skip(1))
        {
            var state = h.Physics.JointStateOf(h.Ragdoll.IdOf(bone))!;
            Less(Math.Abs(state.Offset[0]) + Math.Abs(state.Offset[1]) + Math.Abs(state.Offset[2]), 0.05, bone);
        }
    }

    [Fact]
    public void ASnapDistanceIsMeasuredInTheWorldsUnitsSoAScaledModelCarriesWhatAnUnscaledOneSnaps()
    {
        var scaled = Matrix4.Compose(Vector3.Zero(), Quaternion.Identity(), new Vector3(2, 2, 2));
        using var h = new Harness(model: scaled);
        h.Model = Matrix4.Compose(new Vector3(3, 0, 0), Quaternion.Identity(), new Vector3(2, 2, 2));

        h.Frame();

        // 3 world units is under the 4 a snap distance of 2 model units allows, so
        // the body is given the speed that reaches it in a step.
        Greater(h.Physics.VelocityOf(h.Ragdoll.IdOf("spine"))![0], 100);
    }

    [Fact]
    public void TheModelsTurnDoesNotTurnTheBonesWrittenBackFromALimpBody()
    {
        using var h = new Harness(model: Matrix4.Compose(new Vector3(5, 0, 0), About(0, 1, 0, 0.7), new Vector3(1, 1, 1)));
        h.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);

        h.Run(0.4);

        // The arm falls in the model's own x and y plane whichever way the model faces.
        Less(Math.Abs(h.Pose.TailOf("upper_arm.L").Z), 0.05);
        Near(0.3, (h.Pose.TailOf("upper_arm.L") - h.Pose.HeadOf("upper_arm.L")).Length, 1e-3);
        Same(new Vector3(0.15, 1.4, 0), h.Pose.HeadOf("upper_arm.L"), 1e-4);
    }

    [Fact]
    public void AnAnimatedBoneIsWrittenBackExactlyAndNotWhereItsBodyLaggedTo()
    {
        using var h = new Harness();
        var reference = new Pose(h.Armature);

        h.Ragdoll.SetFrom("head", RagdollMode.Limp);
        for (var i = 1; i <= 10; i++)
        {
            var angle = 0.08 * i;
            h.Animate = pose => pose["upper_arm.L"].Rotation = About(0, 0, 1, angle);
            h.Frame();
            reference.Reset();
            reference["upper_arm.L"].Rotation = About(0, 0, 1, angle);
            reference.Evaluate();

            Same(reference.TailOf("upper_arm.L"), h.Pose.TailOf("upper_arm.L"), 1e-5);
        }
    }

    private static Armature Bent() => new(
    [
        new Bone("a", new Vector3(0, 2, 0), new Vector3(0, 3, 0)),
        new Bone("b", new Vector3(0, 3, 0), new Vector3(1, 3, 0), parent: "a", connected: true),
    ]);

    private static RagdollSpec BentSpec() => new([new RagdollBody("b", radius: 0.1, @from: "a")]);

    [Fact]
    public void ABodyOverABentRunOfBonesLiesAlongTheLineFromHeadToFarTail()
    {
        using var h = new Harness(spec: BentSpec(), armature: Bent());

        Assert.True(Touches(h, "b", new Vector3(0.5, 2.5, 0)));
        Assert.True(Touches(h, "b", new Vector3(0.1, 2.1, 0)));
        Assert.True(Touches(h, "b", new Vector3(0.9, 2.9, 0)));
        Assert.False(Touches(h, "b", new Vector3(0.9, 2.1, 0)));
        Assert.False(Touches(h, "b", new Vector3(0.1, 2.9, 0)));
    }

    [Fact]
    public void ALimpBodyOverABentRunOfBonesWritesTheBonesBackWhereTheyWere()
    {
        using var h = new Harness(spec: BentSpec(), armature: Bent());
        h.Ragdoll.SetMode("b", RagdollMode.Limp);

        h.Pose.Reset();
        h.Pose.Evaluate();
        h.Modifier.Solve(h.Pose, new ModifierFrame(Harness.Dt, h.Model));

        Same(new Vector3(0, 2, 0), h.Pose.HeadOf("a"), 1e-4);
        Same(new Vector3(0, 3, 0), h.Pose.TailOf("a"), 1e-4);
        Same(new Vector3(1, 3, 0), h.Pose.TailOf("b"), 1e-4);
    }

    [Fact]
    public void BuildingOnAnAnimatedPoseStandsTheBodiesWhereThePoseHasThem()
    {
        using var physics = new Physics();
        var armature = Human.Rig();
        var pose = new Pose(armature);
        pose["upper_arm.L"].Rotation = About(0, 0, 1, 0.8);
        pose.Evaluate();

        using var ragdoll = Ragdoll.Build(physics, pose, Human.Spec(), Matrix4.Identity());

        var t = physics.TransformOf(ragdoll.IdOf("upper_arm.L"))!;
        var middle = (pose.HeadOf("upper_arm.L") + pose.TailOf("upper_arm.L")) * 0.5;
        Same(middle, new Vector3(t[0], t[1], t[2]), 1e-4);
    }

    [Fact]
    public void ABuildRefusesASpecThatOverlapsItself()
    {
        using var physics = new Physics();
        var pose = new Pose(Human.Rig());
        pose.Evaluate();
        var spec = new RagdollSpec(
        [
            new RagdollBody("spine", radius: 0.14, @from: "hips"),
            new RagdollBody("head", radius: 0.1, @from: "hips", parent: "spine", joint: new RagdollBall()),
        ]);

        Assert.Throws<ArmatureException>(() => Ragdoll.Build(physics, pose, spec, Matrix4.Identity()));
    }
}
