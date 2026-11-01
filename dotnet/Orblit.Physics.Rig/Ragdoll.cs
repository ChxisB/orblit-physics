using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// A body for each bone a <see cref="RagdollSpec"/> names, joined as the rig is.
/// </summary>
/// <remarks>
/// Each body is <see cref="RagdollMode.Animated"/>, <see cref="RagdollMode.Powered"/>
/// or <see cref="RagdollMode.Limp"/>, and a bone changes between them whenever it
/// is told to. A character is hit with <see cref="Hit"/>. A
/// <see cref="RagdollPose"/> runs <see cref="FollowPose"/> and
/// <see cref="WritePose"/> inside a modifier stack, and they can be called by
/// hand when the ragdoll is run without one.
/// Bodies are named by a run of ids from <c>firstId</c>, one for each body in the
/// spec's order. A joint has the same number as its body, which is allowed
/// because the two are numbered apart.
/// The bodies are one layer that does not collide with itself, so a forearm
/// passes through the thigh it hangs beside and only the world and other things
/// push on it.
/// </remarks>
public sealed class Ragdoll : IDisposable
{
    private readonly List<BodyPart> _parts;
    private readonly Dictionary<string, int> _covering;
    private List<Target> _targets = [];
    private double _clock;

    // F2: eight fields, built in one place by Build.
    private Ragdoll(
        Physics physics,
        Armature armature,
        RagdollSpec spec,
        uint layer,
        Layers layers,
        RagdollTuning tuning,
        List<BodyPart> parts,
        Dictionary<string, int> covering)
    {
        Physics = physics;
        Armature = armature;
        Spec = spec;
        Layer = layer;
        Layers = layers;
        Tuning = tuning;
        _parts = parts;
        _covering = covering;
    }

    public Physics Physics { get; }

    public Armature Armature { get; }

    public RagdollSpec Spec { get; }

    public RagdollTuning Tuning { get; }

    /// <summary>The one layer bit every body is in.</summary>
    public uint Layer { get; }

    /// <summary>
    /// What the bodies are in and care about. Also the layers to cast with when
    /// the ragdoll should not see itself.
    /// </summary>
    public Layers Layers { get; }

    /// <summary>The bone each body ends at, root first.</summary>
    public IReadOnlyList<string> Bones => _parts.Select(part => part.Bone).ToList();

    /// <summary>Whether any body is free, so the ragdoll is writing bones.</summary>
    public bool HasFree => _parts.Any(IsFree);

    /// <summary>Where the root body is, in the world.</summary>
    public Vector3 RootPosition => WorldFrameOf(_parts[0]).Position;

    /// <summary>
    /// Makes bodies and joints for <paramref name="spec"/> in <paramref name="physics"/>,
    /// standing where <paramref name="pose"/> puts them under <paramref name="modelToWorld"/>.
    /// </summary>
    /// <remarks>
    /// <paramref name="pose"/> has to have been evaluated. Every body starts
    /// animated. <paramref name="layer"/> is the one bit the group is in, and
    /// <paramref name="ignore"/> the layers it takes no notice of.
    /// Throws an <see cref="ArmatureException"/> for a bone the pose's armature lacks.
    /// </remarks>
    // F2: eight parameters, the first four required and the rest called by name.
    public static Ragdoll Build(
        Physics physics,
        Pose pose,
        RagdollSpec spec,
        Matrix4 modelToWorld,
        ulong firstId = 1,
        uint layer = 0x100,
        uint ignore = 0,
        RagdollTuning? tuning = null)
    {
        if (layer == 0 || (layer & (layer - 1)) != 0)
        {
            throw new ArgumentOutOfRangeException(nameof(layer), layer, "A layer is a single bit.");
        }

        var armature = pose.Armature;
        spec.Check(armature);
        var frame = new ModifierFrame(modelToWorld: modelToWorld);
        var parts = new List<BodyPart>();
        var covering = new Dictionary<string, int>();
        foreach (var body in spec.Bodies)
        {
            var parent = body.Parent is null ? null : parts[covering[body.Parent]];
            var part = BodyPart.Of(armature, body, parts.Count, firstId + (ulong)parts.Count, parent, frame.Scale);
            parent?.Children.Add(part.Index);
            foreach (var name in RagdollSpec.ChainOf(armature, body))
            {
                covering[name] = part.Index;
            }

            parts.Add(part);
        }

        var ragdoll = new Ragdoll(
            physics,
            armature,
            spec,
            layer,
            new Layers(layer, 0xFFFFFFFFu & ~layer & ~ignore),
            tuning ?? new RagdollTuning(),
            parts,
            covering);
        ragdoll.Lay(new ModelSpace(frame));
        ragdoll.Snap(pose, frame);
        return ragdoll;
    }

    /// <summary>The physics id of the body that covers <paramref name="bone"/>.</summary>
    public ulong IdOf(string bone) => PartFor(bone).Id;

    public RagdollMode ModeOf(string bone) => PartFor(bone).Mode;

    /// <summary>
    /// Holds the body that covers <paramref name="bone"/> as <paramref name="mode"/> says.
    /// </summary>
    /// <remarks>
    /// A bone that is powered or limp has its body made free, and its joint made
    /// again with springs for powered and without for limp. The body stays where
    /// it is.
    /// A body that stays animated below a free one holds that one's far end where
    /// the pose has it, as a hand holds a rail. For a limb that goes loose from a
    /// bone down, use <see cref="SetFrom"/>.
    /// </remarks>
    public void SetMode(string bone, RagdollMode mode) => Enter(PartFor(bone), mode);

    /// <summary>
    /// Holds the body that covers <paramref name="bone"/> and every body that
    /// hangs below it as <paramref name="mode"/> says.
    /// </summary>
    public void SetFrom(string bone, RagdollMode mode)
    {
        var queue = new List<int> { PartFor(bone).Index };
        for (var i = 0; i < queue.Count; i++)
        {
            var part = _parts[queue[i]];
            Enter(part, mode);
            queue.AddRange(part.Children);
        }
    }

    public void SetAll(RagdollMode mode)
    {
        foreach (var part in _parts)
        {
            Enter(part, mode);
        }
    }

    /// <summary>
    /// Hits the body that covers <paramref name="bone"/> with <paramref name="impulse"/>,
    /// newton-seconds in the world, at the world point <paramref name="at"/> or its centre.
    /// </summary>
    /// <remarks>
    /// That body goes limp at once. Each body one joint away follows after
    /// <paramref name="spread"/> seconds, and the next after the same again, so
    /// the blow runs out from where it landed. A spread of nought makes every
    /// body limp now.
    /// </remarks>
    public void Hit(string bone, Vector3 impulse, Vector3? at = null, double spread = 0.04)
    {
        var origin = PartFor(bone);
        Enter(origin, RagdollMode.Limp);
        Physics.Push(origin.Id, Storage.Of(impulse), at is null ? null : Storage.Of(at));

        var hops = HopsFrom(origin);
        foreach (var part in _parts)
        {
            if (part.Mode == RagdollMode.Limp)
            {
                continue;
            }

            var wait = hops[part.Index] * spread;
            if (wait <= 0)
            {
                Enter(part, RagdollMode.Limp);
                continue;
            }

            if (part.LimpAt is null || _clock + wait < part.LimpAt)
            {
                part.LimpAt = _clock + wait;
            }
        }
    }

    /// <summary>
    /// Whether every free body has all but stopped, which is when a fallen
    /// character is ready to be got up.
    /// </summary>
    public bool IsAtRest(double speed = 0.2)
    {
        foreach (var part in _parts.Where(IsFree))
        {
            if (Physics.VelocityOf(part.Id) is not { } v)
            {
                continue;
            }

            if (Magnitude(v, 0) > speed || Magnitude(v, 3) > speed * 4)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Puts every body where <paramref name="pose"/> puts it, with no velocity:
    /// the teleport. Free bodies are put there too.
    /// </summary>
    public void Snap(Pose pose, ModifierFrame frame)
    {
        var space = new ModelSpace(frame);
        _targets = ReadTargets(pose);
        foreach (var part in _parts)
        {
            Place(part, space.ToWorld(_targets[part.Index].Body));
        }
    }

    /// <summary>
    /// Sends what <paramref name="pose"/> asks for to the bodies for the next
    /// step: animated bodies are given the velocity that brings them to it,
    /// powered joints are aimed at its angles, and bodies due to go limp from a
    /// hit do.
    /// </summary>
    /// <remarks>
    /// <paramref name="pose"/> has to be the animated pose, evaluated, and not
    /// yet written to by <see cref="WritePose"/>. <see cref="ModifierFrame.Dt"/>
    /// is the length of the step that follows.
    /// </remarks>
    public void FollowPose(Pose pose, ModifierFrame frame)
    {
        _targets = ReadTargets(pose);
        if (frame.Dt <= 0)
        {
            return;
        }

        _clock += frame.Dt;
        foreach (var part in _parts.Where(part => part.LimpAt <= _clock))
        {
            Enter(part, RagdollMode.Limp);
        }

        var space = new ModelSpace(frame);
        foreach (var part in _parts)
        {
            switch (part.Mode)
            {
                case RagdollMode.Animated:
                    Carry(part, space.ToWorld(_targets[part.Index].Body), space, frame.Dt);
                    break;
                case RagdollMode.Powered:
                    Hold(part);
                    break;
            }
        }
    }

    /// <summary>
    /// Writes the bones whose bodies are free from where the bodies are.
    /// </summary>
    /// <remarks>
    /// Only the root is moved. Below it a bone is turned and stays on its parent,
    /// so it keeps the length the rig gave it however far a joint has been
    /// pulled. Bones that have no body of their own follow the body above them
    /// with their animated pose.
    /// </remarks>
    public void WritePose(Pose pose, ModifierFrame frame)
    {
        var space = new ModelSpace(frame);
        foreach (var part in _parts.Where(IsFree))
        {
            WriteBone(pose, part, space.ToModel(WorldFrameOf(part)));
        }
    }

    /// <summary>Takes every body and joint out of the world.</summary>
    public void Dispose()
    {
        foreach (var part in _parts)
        {
            Physics.Remove(part.Id);
        }
    }

    private static double Magnitude(float[] values, int start)
    {
        double x = values[start];
        double y = values[start + 1];
        double z = values[start + 2];
        return Math.Sqrt((x * x) + (y * y) + (z * z));
    }

    private static bool IsFree(BodyPart part) =>
        part.Mode == RagdollMode.Limp
        || (part.Mode == RagdollMode.Powered && part.Parent is not null);

    private void Lay(ModelSpace space)
    {
        foreach (var part in _parts)
        {
            var at = space.ToWorld(part.RestBody);
            Physics.Add(
                part.Id,
                part.Shape,
                PhysicsMotion.Driven,
                at: Storage.Of(at.Position),
                rotation: Storage.Of(at.Rotation),
                mass: part.Body.Mass,
                friction: part.Body.Friction,
                linearDamping: Tuning.LinearDamping,
                angularDamping: Tuning.AngularDamping,
                layers: Layers);
        }

        foreach (var part in _parts.Where(part => part.Parent is not null))
        {
            Join(part, sprung: false);
        }
    }

    private BodyPart PartFor(string bone)
    {
        var name = bone;
        while (name is not null)
        {
            if (_covering.TryGetValue(name, out var found))
            {
                return _parts[found];
            }

            name = Armature[name]?.Parent;
        }

        throw new ArgumentOutOfRangeException(nameof(bone), bone, "No body covers this bone.");
    }

    private void Enter(BodyPart part, RagdollMode mode)
    {
        // A root has no joint to spring, so powered is as good as animated.
        var wanted = part.Parent is null && mode == RagdollMode.Powered
            ? RagdollMode.Animated
            : mode;
        part.LimpAt = null;
        if (part.Mode == wanted)
        {
            return;
        }

        part.Mode = wanted;
        Physics.SetMotion(
            part.Id,
            wanted == RagdollMode.Animated ? PhysicsMotion.Driven : PhysicsMotion.Free);
        var sprung = wanted == RagdollMode.Powered;
        var held = part.Parent is not null && wanted != RagdollMode.Animated;
        if (held && sprung != part.Sprung)
        {
            Join(part, sprung);
        }
    }

    /// <summary>
    /// How many joints each body is from <paramref name="origin"/>, by way of
    /// parents and children alike.
    /// </summary>
    private int[] HopsFrom(BodyPart origin)
    {
        var hops = Enumerable.Repeat(-1, _parts.Count).ToArray();
        var queue = new List<int> { origin.Index };
        hops[origin.Index] = 0;
        for (var i = 0; i < queue.Count; i++)
        {
            var part = _parts[queue[i]];
            var next = part.Parent is { } up ? part.Children.Prepend(up) : part.Children;
            foreach (var other in next.Where(other => hops[other] < 0))
            {
                hops[other] = hops[part.Index] + 1;
                queue.Add(other);
            }
        }

        return hops;
    }

    private Joint JointFor(BodyPart part, bool sprung)
    {
        JointSpring? spring = sprung ? new JointSpring(Tuning.PowerHertz, Tuning.PowerRatio) : null;
        var locked = JointLimit.Locked();
        return part.Body.Joint switch
        {
            RagdollHinge hinge => Joint.SixAxis(
                alongX: locked,
                alongY: locked,
                alongZ: locked,
                aboutX: new JointLimit(hinge.Low, hinge.High),
                aboutY: locked,
                aboutZ: locked,
                springs: new JointSprings(AboutX: spring)),
            RagdollBall ball => Joint.SixAxis(
                alongX: locked,
                alongY: locked,
                alongZ: locked,
                aboutX: new JointLimit(ball.TwistLow, ball.TwistHigh),
                aboutY: new JointLimit(-ball.Swing, ball.Swing),
                aboutZ: new JointLimit(-ball.Swing, ball.Swing),
                springs: new JointSprings(AboutX: spring, AboutY: spring, AboutZ: spring)),
            _ => throw new InvalidOperationException($"Body \"{part.Bone}\" has no joint to make."),
        };
    }

    /// <summary>
    /// Makes the joint of <paramref name="part"/> again, with or without springs.
    /// </summary>
    /// <remarks>
    /// A joint holds the turn its bodies stood in when it was made as its nought,
    /// and its limits and springs are measured from there. So the body is stood
    /// at its rest against its parent for a moment, the joint is made, and the
    /// body goes back with the velocity it had. No step runs between, so nothing
    /// sees it move.
    /// </remarks>
    private void Join(BodyPart part, bool sprung)
    {
        // Only a part with a parent has a joint, and every caller checks that.
        var parent = _parts[part.Parent!.Value];
        var outer = WorldFrameOf(parent);
        var here = WorldFrameOf(part);
        var moving = Physics.VelocityOf(part.Id);

        Place(part, outer.Then(part.RestInParent));
        var joint = outer.Then(part.JointInParent);
        Physics.Unjoin(part.Id);
        Physics.Join(
            part.Id,
            JointFor(part, sprung),
            parent.Id,
            Storage.Of(joint.Position),
            part.Id,
            Storage.Of(joint.Rotation));
        Place(part, here);
        if (moving is not null)
        {
            Physics.Drive(
                part.Id,
                velocity: [moving[0], moving[1], moving[2]],
                spin: [moving[3], moving[4], moving[5]]);
        }

        part.Sprung = sprung;
    }

    private void Place(BodyPart part, Frame at) =>
        Physics.Place(part.Id, Storage.Of(at.Position), Storage.Of(at.Rotation));

    private Frame WorldFrameOf(BodyPart part)
    {
        var t = Physics.TransformOf(part.Id)
            ?? throw new InvalidOperationException($"Body {part.Id} of the ragdoll has been removed.");
        return new Frame(new Vector3(t[0], t[1], t[2]), new Quaternion(t[3], t[4], t[5], t[6]));
    }

    private List<Target> ReadTargets(Pose pose) =>
        _parts.Select(part => TargetOf(part, pose)).ToList();

    private static Target TargetOf(BodyPart part, Pose pose)
    {
        var position = Vector3.Zero();
        var rotation = Quaternion.Identity();
        var scale = Vector3.Zero();
        pose.WorldOf(part.Body.From).Decompose(position, rotation, scale);
        rotation.Normalize();
        var bone = new Frame(position, rotation);
        return new Target(bone, bone.Then(part.Offset), scale);
    }

    /// <summary>
    /// Gives an animated body the velocity that brings it to
    /// <paramref name="goal"/> in one step, or puts it there if it is further off
    /// than a step can mean.
    /// </summary>
    private void Carry(BodyPart part, Frame goal, ModelSpace space, double dt)
    {
        var here = WorldFrameOf(part);
        var gap = goal.Position - here.Position;
        if (gap.Length > Tuning.SnapDistance * space.Scale)
        {
            Place(part, goal);
            return;
        }

        var turn = Turns.Between(here.Rotation, goal.Rotation);
        Physics.Drive(part.Id, Storage.Of(gap / dt), Storage.Of(turn / dt));
    }

    /// <summary>Aims a powered joint at the angle the pose has between its two bones.</summary>
    private void Hold(BodyPart part)
    {
        if (part.Parent is not { } up)
        {
            return;
        }

        var above = Departure(_parts[up]);
        var below = Departure(part);
        var joint = part.RestFrame;
        var turn = joint.Conjugated() * above.Conjugated() * below * joint;
        Physics.Aim(part.Id, part.RestsFor(turn));
    }

    /// <summary>
    /// How far the pose has turned the part's first bone from rest, in the
    /// model's space.
    /// </summary>
    private Quaternion Departure(BodyPart part) =>
        _targets[part.Index].Bone.Rotation * part.RestBone.Rotation.Conjugated();

    private void WriteBone(Pose pose, BodyPart part, Frame body)
    {
        var rotation = (body.Rotation * part.Offset.Rotation.Conjugated()).Normalized();
        var turned = new Frame(Vector3.Zero(), rotation);
        var head = part.Parent is null
            ? body.Position - turned.Turn(part.Offset.Position)
            : pose.HeadOf(part.Body.From);
        BoneEdit.PlaceBone(
            pose,
            part.Body.From,
            Matrix4.Compose(head, rotation, _targets[part.Index].Scale));
    }

    /// <summary>Where the pose puts one body this frame, in the model's space.</summary>
    /// <param name="Bone">The frame of the body's first bone.</param>
    /// <param name="Body">The frame of the body, which is the bone's offset to the capsule.</param>
    /// <param name="Scale">The bone's scale.</param>
    private sealed record Target(Frame Bone, Frame Body, Vector3 Scale);
}
