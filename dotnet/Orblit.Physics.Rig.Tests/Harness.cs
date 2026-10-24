using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig.Tests;

/// <summary>A person made of bones, and the ragdoll spec that stands a body on each limb.</summary>
internal static class Human
{
    public const uint CharacterLayer = 2;

    public static Armature Rig()
    {
        var bones = new List<Bone>
        {
            new("hips", new Vector3(0, 1, 0), new Vector3(0, 1.15, 0)),
            new("spine", new Vector3(0, 1.15, 0), new Vector3(0, 1.45, 0), parent: "hips", connected: true),
            new("head", new Vector3(0, 1.45, 0), new Vector3(0, 1.75, 0), parent: "spine", connected: true),
        };
        foreach (var (side, s) in new[] { ("L", 1.0), ("R", -1.0) })
        {
            bones.Add(new Bone(
                $"upper_arm.{side}",
                new Vector3(0.15 * s, 1.4, 0),
                new Vector3(0.45 * s, 1.4, 0),
                parent: "spine"));
            bones.Add(new Bone(
                $"forearm.{side}",
                new Vector3(0.45 * s, 1.4, 0),
                new Vector3(0.75 * s, 1.4, 0),
                parent: $"upper_arm.{side}",
                connected: true));
            bones.Add(new Bone(
                $"thigh.{side}",
                new Vector3(0.1 * s, 1.0, 0),
                new Vector3(0.1 * s, 0.55, 0),
                parent: "hips"));
            bones.Add(new Bone(
                $"shin.{side}",
                new Vector3(0.1 * s, 0.55, 0),
                new Vector3(0.1 * s, 0.05, 0),
                parent: $"thigh.{side}",
                connected: true));
        }

        return new Armature(bones);
    }

    public static RagdollSpec Spec()
    {
        var bodies = new List<RagdollBody>
        {
            new("spine", radius: 0.14, @from: "hips", mass: 20),
            new("head", radius: 0.1, parent: "spine", joint: new RagdollBall(swing: 0.6), mass: 4),
        };
        foreach (var side in new[] { "L", "R" })
        {
            bodies.Add(new RagdollBody(
                $"upper_arm.{side}",
                radius: 0.05,
                parent: "spine",
                joint: new RagdollBall(swing: 1.5),
                mass: 2));
            bodies.Add(new RagdollBody(
                $"forearm.{side}",
                radius: 0.04,
                parent: $"upper_arm.{side}",
                joint: new RagdollHinge(),
                mass: 1.5));
            bodies.Add(new RagdollBody(
                $"thigh.{side}",
                radius: 0.07,
                parent: "spine",
                joint: new RagdollBall(swing: 1.2),
                mass: 7));
            bodies.Add(new RagdollBody(
                $"shin.{side}",
                radius: 0.05,
                parent: $"thigh.{side}",
                joint: new RagdollHinge(),
                mass: 4));
        }

        return new RagdollSpec(bodies);
    }
}

/// <summary>
/// A ragdoll on a floor, run as a game runs it: solve the pose, then step the world.
/// </summary>
internal sealed class Harness : IDisposable
{
    public const double Dt = 1.0 / 60;

    public Harness(
        RagdollTuning? tuning = null,
        Matrix4? model = null,
        RagdollSpec? spec = null,
        Armature? armature = null)
    {
        Model = model ?? Matrix4.Identity();
        Armature = armature ?? Human.Rig();
        Pose = new Pose(Armature);
        Physics.Add(900, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
        Pose.Evaluate();
        Ragdoll = Ragdoll.Build(
            Physics,
            Pose,
            spec ?? Human.Spec(),
            Model,
            ignore: Human.CharacterLayer,
            tuning: tuning);
        Modifier = new RagdollPose(Ragdoll);
    }

    public Physics Physics { get; } = new();

    public Armature Armature { get; }

    public Pose Pose { get; }

    public Ragdoll Ragdoll { get; }

    public RagdollPose Modifier { get; }

    public Matrix4 Model { get; set; }

    /// <summary>Poses the bones each frame before the modifier runs.</summary>
    public Action<Pose>? Animate { get; set; }

    public void Frame()
    {
        Pose.Reset();
        Animate?.Invoke(Pose);
        Pose.Evaluate();
        Modifier.Solve(Pose, new ModifierFrame(Dt, Model));
        Physics.Step(Dt);
    }

    public void Run(double seconds)
    {
        var frames = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
        for (var i = 0; i < frames; i++)
        {
            Frame();
        }
    }

    public Vector3 HeadOf(string bone) => Model.Transformed3(Pose.HeadOf(bone));

    public Vector3 TailOf(string bone) => Model.Transformed3(Pose.TailOf(bone));

    public Vector3 BodyOf(string bone)
    {
        var t = Physics.TransformOf(Ragdoll.IdOf(bone))!;
        return new Vector3(t[0], t[1], t[2]);
    }

    public void Dispose()
    {
        Ragdoll.Dispose();
        Physics.Dispose();
    }
}

/// <summary>Two bones end to end along x, a body on each, with no gravity so only a push moves them.</summary>
internal static class Limbs
{
    public static Armature Arm() => new(
    [
        new Bone("a", new Vector3(0, 1, 0), new Vector3(1, 1, 0)),
        new Bone("b", new Vector3(1, 1, 0), new Vector3(2, 1, 0), parent: "a", connected: true),
    ]);

    public static Harness Chain(RagdollJoint joint, RagdollMode mode)
    {
        var h = new Harness(null, null, new RagdollSpec(
        [
            new RagdollBody("a", radius: 0.05),
            new RagdollBody("b", radius: 0.05, parent: "a", joint: joint),
        ]), Arm());
        h.Physics.SetGravity([0.0, 0.0, 0.0]);
        h.Ragdoll.SetMode("b", mode);
        return h;
    }

    /// <summary>One bone lying along x a little above a floor.</summary>
    public static Harness Lying(double mass = 1, double friction = 0.5, RagdollTuning? tuning = null) =>
        new(
            tuning,
            null,
            new RagdollSpec([new RagdollBody("a", radius: 0.1, mass: mass, friction: friction)]),
            new Armature([new Bone("a", new Vector3(0, 0.15, 0), new Vector3(1, 0.15, 0))]));
}
