using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class RagdollAimAddedTests
{
    private static readonly Matrix4 Turned = Matrix4.Compose(
        new Vector3(5, 0, -3),
        Quaternion.AxisAngle(new Vector3(0, 1, 0), 0.7),
        new Vector3(1, 1, 1));

    private static Quaternion About(Vector3 axis, double angle) => Quaternion.AxisAngle(axis, angle);

    private static IReadOnlyList<double> Angles(Harness h, string bone = "b") =>
        h.Physics.JointStateOf(h.Ragdoll.IdOf(bone))!.Angles;

    private static Vector3 Unit(int axis) => new(axis == 0 ? 1 : 0, axis == 1 ? 1 : 0, axis == 2 ? 1 : 0);

    [Theory]
    [InlineData(3.0, 0.9)]
    [InlineData(-3.0, -0.3)]
    public void AHingeAboutAnotherAxisStopsAtItsLimits(double push, double stops)
    {
        using var h = Limbs.Chain(new RagdollHinge(new Vector3(0, 0, 1), low: -0.3, high: 0.9), RagdollMode.Limp);
        h.Physics.Push(h.Ragdoll.IdOf("b"), [0.0, push, 0.0], [2.0, 1.0, 0.0]);

        h.Run(0.6);

        Near(stops, Angles(h)[0], 0.05, "hinge angle");
    }

    [Fact]
    public void APoweredHingeAboutAnotherAxisIsAimedAtTheAngleOfThePose()
    {
        using var h = Limbs.Chain(new RagdollHinge(new Vector3(0, 0, 1), low: -0.3, high: 0.9), RagdollMode.Powered);
        h.Animate = pose => pose["b"].Rotation = About(Unit(2), 0.5);

        h.Run(2.0);

        Near(0.5, Angles(h)[0], 0.05, "hinge angle");
    }

    [Theory]
    [InlineData(1, 0.3, 0, 0.3)]
    [InlineData(2, 0.4, 2, 0.4)]
    [InlineData(0, 0.3, 1, -0.3)]
    public void APoweredBallIsAimedAtTheTwistAndSwingOfThePose(int about, double angle, int way, double expected)
    {
        using var h = Limbs.Chain(new RagdollBall(swing: 0.5, twistLow: -0.2, twistHigh: 0.7), RagdollMode.Powered);
        h.Animate = pose => pose["b"].Rotation = About(Unit(about), angle);

        h.Run(2.0);

        Near(expected, Angles(h)[way], 0.05, "joint angle");
    }

    [Fact]
    public void ChangingModeLeavesABodyWhereItIsAndMovingAsItWas()
    {
        using var h = Limbs.Chain(new RagdollHinge(), RagdollMode.Limp);
        var id = h.Ragdoll.IdOf("b");
        h.Physics.Push(id, [0.0, 0.0, 3.0], [2.0, 1.0, 0.0]);
        h.Run(0.2);
        Greater(Math.Abs(Angles(h)[0]), 0.05, "the body has swung");
        var place = h.Physics.TransformOf(id)!;
        var speed = h.Physics.VelocityOf(id)!;

        h.Ragdoll.SetMode("b", RagdollMode.Powered);

        var after = h.Physics.TransformOf(id)!;
        var then = h.Physics.VelocityOf(id)!;
        for (var i = 0; i < place.Length; i++)
        {
            Near(place[i], after[i], 1e-4, $"place {i}");
        }

        for (var i = 0; i < speed.Length; i++)
        {
            Near(speed[i], then[i], 1e-4, $"speed {i}");
        }
    }

    [Fact]
    public void ALimpArmFallsTheSameWayWhereverTheModelStands()
    {
        using var plain = new Harness();
        using var turned = new Harness(model: Turned);
        plain.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);
        turned.Ragdoll.SetFrom("upper_arm.L", RagdollMode.Limp);

        plain.Run(0.15);
        turned.Run(0.15);

        Less(plain.Pose.TailOf("upper_arm.L").Y, 1.35, "the arm has fallen");
        foreach (var bone in new[] { "upper_arm.L", "forearm.L" })
        {
            var expected = plain.Pose.TailOf(bone);
            var actual = turned.Pose.TailOf(bone);
            Near(expected.X, actual.X, 5e-3, $"{bone} x");
            Near(expected.Y, actual.Y, 5e-3, $"{bone} y");
            Near(expected.Z, actual.Z, 5e-3, $"{bone} z");
        }
    }

    [Fact]
    public void TheElbowOfAScaledModelPivotsWhereTheBonesMeet()
    {
        var scaled = Matrix4.Compose(Vector3.Zero(), Quaternion.Identity(), new Vector3(2, 2, 2));
        using var h = new Harness(model: scaled);
        h.Ragdoll.SetMode("forearm.L", RagdollMode.Limp);
        h.Physics.Push(h.Ragdoll.IdOf("forearm.L"), [0.0, 0.0, 3.0], [1.5, 2.8, 0.0]);

        h.Run(0.3);

        Greater(Math.Abs(Angles(h, "forearm.L")[0]), 0.2, "the elbow has bent");
        var gap = EndOf(h, "upper_arm.L", 1) - EndOf(h, "forearm.L", -1);
        Less(gap.Length, 0.05, "gap at the elbow");
    }

    [Fact]
    public void ABodyHungFromAnAnimatedOneFallsFreelyUntilItsHingeStopsIt()
    {
        using var h = Limbs.Chain(new RagdollHinge(new Vector3(0, 0, 1), low: -3, high: 3), RagdollMode.Animated);
        h.Physics.SetGravity([0.0, -9.81, 0.0]);
        h.Ragdoll.SetMode("a", RagdollMode.Limp);

        h.Run(0.8);

        Greater(Math.Abs(Angles(h)[0]), 1.0, "hinge angle");
    }

    [Fact]
    public void ABodyOverBonesThatComeBackToWhereTheyStartedStandsInTheTurnOfItsFirstBone()
    {
        var armature = new Armature(
        [
            new Bone("a", new Vector3(0, 1, 0), new Vector3(1, 1, 0)),
            new Bone("b", new Vector3(1, 1, 0), new Vector3(0, 1, 0), parent: "a", connected: true),
        ]);
        using var h = new Harness(
            spec: new RagdollSpec([new RagdollBody("b", radius: 0.1, @from: "a")]),
            armature: armature);

        var turn = h.Physics.TransformOf(h.Ragdoll.IdOf("b"))!;

        // The first bone lies along x, which is a quarter turn about z from the y a capsule stands along.
        Near(Math.Sqrt(0.5), Math.Abs(turn[5]), 1e-4, "z");
        Near(Math.Sqrt(0.5), Math.Abs(turn[6]), 1e-4, "w");
    }

    /// <summary>The end of a capsule's axis, <paramref name="way"/> 1 toward the tail and -1 toward the head.</summary>
    private static Vector3 EndOf(Harness h, string bone, int way)
    {
        var t = h.Physics.TransformOf(h.Ragdoll.IdOf(bone))!;
        var frame = new Frame(new Vector3(t[0], t[1], t[2]), new Quaternion(t[3], t[4], t[5], t[6]));
        // Both arm bones are 0.3 long in the model and the model is twice as big.
        return frame.Position + frame.Turn(new Vector3(0, 0.3 * way, 0));
    }
}
