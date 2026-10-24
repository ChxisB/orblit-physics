using System.Diagnostics.CodeAnalysis;
using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

public sealed partial class ScenePhysics
{
    /// <summary>
    /// Makes the world's joints agree with the document for every joint entity in
    /// <paramref name="ids"/>, and for every joint the world dropped when the bodies in
    /// <paramref name="gone"/> were taken out to be rebuilt.
    /// </summary>
    private void Rejoin(IReadOnlyList<string> ids, HashSet<ulong> gone)
    {
        var joints = JointsAmong(ids, gone);
        if (joints.Count == 0)
        {
            return;
        }

        // Taken apart first, all of them, so none is made against a body another is about to be moved
        // off.
        foreach (var id in joints)
        {
            if (_held.Remove(id))
            {
                Physics.Unjoin(_jointOf[id]);
            }
        }

        // Nearest the root first. The world solves joints in the order they were made, and a chain
        // solved from its fixed end outwards settles in fewer passes than one solved from its tip.
        foreach (var id in joints.OrderBy(id => AncestorsOf(id).Count))
        {
            Join(id);
        }
    }

    private List<string> JointsAmong(IReadOnlyList<string> ids, HashSet<ulong> gone)
    {
        var named = ids.Where(id => _jointOf.ContainsKey(id) || _document[id]?[SceneComponents.Joint] is JointComponent);
        var dropped = _held.Where(held => gone.Contains(held.Value.A) || gone.Contains(held.Value.B)).Select(held => held.Key);
        return Unique(named.Concat(dropped));
    }

    /// <summary>
    /// Makes the world's rules agree with the pairs told to be ignored: a pair whose entity has lost its
    /// body is forgotten, and one whose body was rebuilt in <paramref name="gone"/> is made again,
    /// because the world drops a rule with the body it names.
    /// </summary>
    private void Reignore(HashSet<ulong> gone)
    {
        _ignored.RemoveWhere(pair => !_bodyOf.ContainsKey(pair.A) || !_bodyOf.ContainsKey(pair.B));
        foreach (var (first, second) in _ignored)
        {
            var a = _bodyOf[first];
            var b = _bodyOf[second];
            if (gone.Contains(a) || gone.Contains(b))
            {
                Physics.SetRule(a, b, new PhysicsRule(ignore: true));
            }
        }
    }

    /// <summary>
    /// Makes <paramref name="id"/>'s joint in the world, if it has one, it is not broken and there is a
    /// body for it to hold.
    /// </summary>
    private void Join(string id)
    {
        if (_document[id]?[SceneComponents.Joint] is not JointComponent joint)
        {
            if (_jointOf.Remove(id, out var gone))
            {
                _entityOfJoint.Remove(gone);
            }

            _broken.Remove(id);
            return;
        }

        if (!_jointOf.TryGetValue(id, out var number))
        {
            number = _nextJoint++;
            _jointOf[id] = number;
        }

        _entityOfJoint[number] = id;
        if (!_broken.Contains(id))
        {
            Hold(id, number, joint);
        }
    }

    private void Hold(string id, ulong number, JointComponent joint)
    {
        var (body, holder) = JointComponent.EndsOf(id, ParentOf, _bodyOf.ContainsKey);

        // A pulley with other than two points is not one. The world would make it from nothing, which is
        // worse than not making it.
        if (body is null || (joint.Kind == JointKind.Pulley && joint.Over.Count != 2))
        {
            return;
        }

        var b = _bodyOf[body];
        var a = holder is null ? 0 : _bodyOf[holder];

        // The joint entity's own place and turn, not its size: a hinge scaled to two is the same hinge.
        var at = Vector3.Zero();
        var rotation = Quaternion.Identity();
        ScenePose.WorldOf(_document, id).Decompose(at, rotation, Vector3.Zero());

        // The body was added or rebuilt before its joints, so the world has it.
        var middle = Physics.TransformOf(b)!;
        var frame = Matrix4.Compose(at, rotation, Vector3.All(1));
        var across = joint.AxisB.Length2 > 0 ? frame.Rotated3(joint.AxisB) : null;
        var (over, path) = PointsOf(joint, frame);

        var made = Physics.Join(
            number,
            JointFor(joint),
            a,
            Listed(at),
            b,
            [rotation.X, rotation.Y, rotation.Z, rotation.W],
            to: [middle[0], middle[1], middle[2]],
            axisB: across is null ? null : Listed(across),
            over: over,
            path: path,
            breakingForce: joint.BreakingForce,
            breakingTorque: joint.BreakingTorque,
            collide: joint.Collide);
        if (made)
        {
            _held[id] = (a, b);
        }
    }

    private string? ParentOf(string id)
    {
        var parent = _document[id]?.Parent;
        return parent == id ? null : parent;
    }

    /// <summary>
    /// The points a coupling or a path names are the joint's own, and move with the entity into the
    /// world.
    /// </summary>
    private static (IReadOnlyList<IReadOnlyList<double>>? Over, IReadOnlyList<IReadOnlyList<double>>? Path) PointsOf(
        JointComponent joint,
        Matrix4 frame)
    {
        IReadOnlyList<double>[] InWorld(IReadOnlyList<Vector3> local) => [.. local.Select(point => Listed(frame.Transformed3(point)))];

        return (
            joint.Kind == JointKind.Pulley ? InWorld(joint.Over) : null,
            joint.Kind == JointKind.Path ? InWorld(joint.Path) : null);
    }

    private static double[] Listed(Vector3 point) => [point.X, point.Y, point.Z];

    /// <summary>
    /// What <paramref name="joint"/> holds, in the world's units: the document gives angles in degrees,
    /// as a transform's rotation is, and the world wants radians.
    /// </summary>
    [SuppressMessage("Maintainability", "CA1502", Justification = "F2: a list of cases is data.")]
    private static Joint JointFor(JointComponent joint)
    {
        var springs = new JointSprings(
            AlongX: SpringOf(joint, JointAxis.AlongX),
            AlongY: SpringOf(joint, JointAxis.AlongY),
            AlongZ: SpringOf(joint, JointAxis.AlongZ),
            AboutX: SpringOf(joint, JointAxis.AboutX),
            AboutY: SpringOf(joint, JointAxis.AboutY),
            AboutZ: SpringOf(joint, JointAxis.AboutZ));
        var soft = joint.SoftHertz > 0 ? new JointSoftness(joint.SoftHertz, joint.SoftRatio) : null;

        return joint.Kind switch
        {
            JointKind.Fixed => Joint.Fixed(springs),
            JointKind.Point => Joint.Point(springs),
            JointKind.Hinge => Joint.Hinge(
                LimitOf(joint, JointAxis.AboutX),
                Doubles.Radians(joint.Speed),
                joint.Strength,
                GoalOf(joint, turning: true),
                springs,
                soft),
            JointKind.Slider => Joint.Slider(
                LimitOf(joint, JointAxis.AlongX),
                joint.Speed,
                joint.Strength,
                GoalOf(joint, turning: false),
                springs,
                soft),
            JointKind.Distance => Joint.Distance(LimitOf(joint, JointAxis.AlongX), springs, soft),
            JointKind.Cone => Joint.Cone(Doubles.Radians(joint.Swing), LimitOf(joint, JointAxis.AboutX), springs, soft),
            JointKind.SixAxis => Joint.SixAxis(
                LimitOf(joint, JointAxis.AlongX),
                LimitOf(joint, JointAxis.AlongY),
                LimitOf(joint, JointAxis.AlongZ),
                LimitOf(joint, JointAxis.AboutX),
                LimitOf(joint, JointAxis.AboutY),
                LimitOf(joint, JointAxis.AboutZ),
                springs,
                soft),
            JointKind.Gear => Joint.Gear(joint.Ratio, springs),
            JointKind.Rack => Joint.Rack(joint.Ratio, springs),
            JointKind.Pulley => Joint.Pulley(joint.Ratio, springs),
            JointKind.Path => Joint.Path(
                LimitOf(joint, JointAxis.AlongX),
                joint.Speed,
                joint.Strength,
                GoalOf(joint, turning: false),
                springs,
                soft,
                joint.Follow),
            _ => throw new InvalidOperationException($"No joint is made for {joint.Kind}."),
        };
    }

    private static JointLimit? LimitOf(JointComponent joint, JointAxis axis)
    {
        if (!joint.Limits.TryGetValue(axis, out var range))
        {
            return null;
        }

        return JointAxes.Turns(axis)
            ? new JointLimit(Doubles.Radians(range.Low), Doubles.Radians(range.High))
            : new JointLimit(range.Low, range.High);
    }

    private static JointSpring? SpringOf(JointComponent joint, JointAxis axis)
    {
        if (!joint.Springs.TryGetValue(axis, out var give) || !(give.Hertz > 0) || !double.IsFinite(give.Hertz))
        {
            return null;
        }

        var rest = give.Rest is { } at && JointAxes.Turns(axis) ? Doubles.Radians(at) : give.Rest;
        return new JointSpring(give.Hertz, give.Ratio, rest);
    }

    /// <summary>
    /// A goal is in degrees for a hinge and metres for a slider or a path, and a motor without one is a
    /// motor of speed.
    /// </summary>
    private static JointGoal? GoalOf(JointComponent joint, bool turning) => joint.GoalHertz > 0
        ? new JointGoal(turning ? Doubles.Radians(joint.Goal) : joint.Goal, joint.GoalHertz, joint.GoalRatio)
        : null;
}
