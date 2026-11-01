using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

public sealed partial class ScenePhysics
{
    /// <summary>Floats in a pose: translation xyz then rotation xyzw.</summary>
    private const int PoseSize = 7;

    private SceneDiff WriteBack()
    {
        ulong[] bodies = [.. _known.Keys];
        var now = Smooth ? Blended(bodies) : Read(bodies, _known);
        var changed = Differing(bodies, now);
        if (changed.Count == 0)
        {
            return SceneDiff.None;
        }

        var moved = new HashSet<string>();
        foreach (var i in changed)
        {
            now.AsSpan(i * PoseSize, PoseSize).CopyTo(_known[bodies[i]]);
            moved.Add(_entityOf[bodies[i]]);
        }

        return Place(Writing(moved));
    }

    /// <summary>
    /// Writes the transform of each of <paramref name="ids"/>, in order, so each is measured against
    /// where the ones before it went. An id whose parent has been squashed flat is skipped.
    /// </summary>
    private SceneDiff Place(List<string> ids)
    {
        var placed = new Dictionary<string, TransformComponent>();
        var operations = new List<SceneOp>();
        foreach (var id in ids)
        {
            if (_document[id] is not { } entity || entity[SceneComponents.Body] is not BodyComponent body)
            {
                continue;
            }

            var was = entity[SceneComponents.Transform] as TransformComponent;
            var next = TransformFor(entity, body, _known[_bodyOf[id]], placed);
            if (next is null)
            {
                continue;
            }

            placed[id] = next;
            operations.Add(new SetComponent(id, SceneComponents.Transform, was?.ToJson(), next.ToJson()));
        }

        Settle(placed);
        return new SceneDiff(operations);
    }

    /// <summary>
    /// The places in <paramref name="bodies"/> of the ones the world has somewhere other than
    /// <c>_known</c> says. Compared as the floats are, so a pose that is not a number is always different.
    /// </summary>
    private List<int> Differing(ulong[] bodies, float[] now)
    {
        var changed = new List<int>();
        for (var i = 0; i < bodies.Length; i++)
        {
            if (!Same(_known[bodies[i]], now, i * PoseSize))
            {
                changed.Add(i);
            }
        }

        return changed;
    }

    private static bool Same(float[] known, float[] now, int from)
    {
        for (var i = 0; i < PoseSize; i++)
        {
            if (known[i] != now[from + i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Every entity with a body that moved, or that has a moved body above it, parents first. A body
    /// under one that moved is written too, even though the world left it where it was: its transform
    /// is relative to its parent, and what it is relative to has gone. Each child is then measured
    /// against where its parent went.
    /// </summary>
    private List<string> Writing(HashSet<string> moved)
    {
        var depths = new Dictionary<string, int>();
        var writing = new List<string>();
        foreach (var id in _bodyOf.Keys)
        {
            var above = AncestorsOf(id);
            if (moved.Contains(id) || above.Any(moved.Contains))
            {
                depths[id] = above.Count;
                writing.Add(id);
            }
        }

        return [.. writing.OrderBy(id => depths[id])];
    }

    /// <summary>
    /// Moves each car's wheel entities to where the last step left the wheels, and says what moved. With
    /// <see cref="Smooth"/> the chassis is shown a step behind and the wheels are not, which is a few
    /// millimetres of spring.
    /// </summary>
    private SceneDiff Seat()
    {
        var placed = new Dictionary<string, TransformComponent>();
        var operations = new List<SceneOp>();
        foreach (var car in _cars.Values)
        {
            foreach (var (id, next) in car.Poses(_document))
            {
                var was = _document[id]?[SceneComponents.Transform] as TransformComponent;
                placed[id] = next;
                operations.Add(new SetComponent(id, SceneComponents.Transform, was?.ToJson(), next.ToJson()));
            }
        }

        if (operations.Count == 0)
        {
            return SceneDiff.None;
        }

        Settle(placed);
        return new SceneDiff(operations);
    }

    /// <summary>
    /// Puts <paramref name="placed"/> in the document. Built once rather than an operation at a time:
    /// replacing one entity copies the document, and a pile of crates settling is hundreds a step.
    /// </summary>
    private void Settle(Dictionary<string, TransformComponent> placed) => _document = _document.CopyWith(
        entities:
        [
            .. _document.Entities.Select(entity => placed.TryGetValue(entity.Id, out var transform)
                ? entity.WithComponent(SceneComponents.Transform, transform)
                : entity),
        ]);

    /// <summary>
    /// Where the world has <paramref name="bodies"/>, seven floats each. A body it has lost, taken out
    /// through <see cref="Physics"/> directly, is read as <paramref name="kept"/> has it, so it reads as
    /// not having moved.
    /// </summary>
    private float[] Read(ulong[] bodies, OrderedDictionary<ulong, float[]> kept)
    {
        var poses = new float[bodies.Length * PoseSize];
        for (var i = 0; i < bodies.Length; i++)
        {
            kept[bodies[i]].CopyTo(poses, i * PoseSize);
        }

        Physics.ReadInto(bodies, poses);
        return poses;
    }

    /// <summary>
    /// Where <paramref name="bodies"/> are shown: each one the time owed to the next step of the way
    /// from where it was after the last but one step to where it is after the last. Seven floats each.
    /// </summary>
    private float[] Blended(ulong[] bodies)
    {
        var alpha = _owed / Step;
        var poses = new float[bodies.Length * PoseSize];
        for (var i = 0; i < bodies.Length; i++)
        {
            ScenePose.BlendPose(_previous[bodies[i]], _latest[bodies[i]], alpha, poses.AsSpan(i * PoseSize, PoseSize));
        }

        return poses;
    }

    /// <summary>
    /// Takes where the world has every body as the end of the blend, and the end it had as the start. A
    /// body in <paramref name="sharp"/> has no start: it is shown as the world has it.
    /// </summary>
    private void Carry(HashSet<ulong> sharp)
    {
        ulong[] bodies = [.. _latest.Keys];
        var now = Read(bodies, _latest);
        for (var i = 0; i < bodies.Length; i++)
        {
            var latest = _latest[bodies[i]];
            var pose = now.AsSpan(i * PoseSize, PoseSize);
            Span<float> start = sharp.Contains(bodies[i]) ? pose : latest;
            start.CopyTo(_previous[bodies[i]]);
            pose.CopyTo(latest);
        }
    }

    /// <summary>Makes <paramref name="bodies"/> stand still in the blend, where the world has them now.</summary>
    private void Collapse(ulong[] bodies)
    {
        if (!Smooth)
        {
            return;
        }

        var now = Read(bodies, _latest);
        for (var i = 0; i < bodies.Length; i++)
        {
            var pose = now.AsSpan(i * PoseSize, PoseSize);
            pose.CopyTo(_previous[bodies[i]]);
            pose.CopyTo(_latest[bodies[i]]);
        }
    }

    /// <summary>The bodies of every entity <see cref="StopSmoothing"/> has switched off, and of every entity under one.</summary>
    private HashSet<ulong> SharpBodies() => [.. _sharp.SelectMany(BodiesUnder)];

    /// <summary>The bodies of <paramref name="entity"/> and of everything under it.</summary>
    private ulong[] BodiesUnder(string entity) => [.. _document.SubtreeOf(entity)
        .Where(under => _bodyOf.ContainsKey(under.Id))
        .Select(under => _bodyOf[under.Id])];

    /// <summary>
    /// The transform that puts <paramref name="entity"/>'s body at <paramref name="pose"/>, its local
    /// position and rotation under its parent, keeping the scale it already had. Null when its parent
    /// has been squashed flat and no transform can.
    /// </summary>
    private TransformComponent? TransformFor(
        SceneEntity entity,
        BodyComponent body,
        float[] pose,
        Dictionary<string, TransformComponent> placed)
    {
        // Nothing here changes a size, so the entity is as big in the world as the document already
        // makes it.
        var scale = Vector3.Zero();
        ScenePose.WorldOf(_document, entity.Id).Decompose(Vector3.Zero(), Quaternion.Identity(), scale);

        // The world has the shape's middle. The entity is wherever the body's centre is measured from.
        // Through the matrix, as placing the body did: turning a vector by the quaternion directly
        // turns by the inverse, and the entity would jump sideways.
        var rotation = new Quaternion(pose[3], pose[4], pose[5], pose[6]);
        rotation.Normalize();
        var offset = rotation.AsRotationMatrix().Transformed(ScenePose.Times(CentreOf(body, _bodyOf[entity.Id]), scale));
        var at = new Vector3(pose[0], pose[1], pose[2]) - offset;
        var world = Matrix4.Compose(at, rotation, scale);

        var parent = entity.Parent == entity.Id ? null : entity.Parent;
        var local = Matrix4.Identity();
        if (local.CopyInverse(ScenePose.WorldOf(_document, parent, placed)) == 0)
        {
            return null;
        }

        local.Multiply(world);
        var position = Vector3.Zero();
        var turned = Quaternion.Identity();
        local.Decompose(position, turned, Vector3.Zero());
        var was = entity[SceneComponents.Transform] as TransformComponent;
        return new TransformComponent(
            position,
            ScenePose.EulerOf(turned.AsRotationMatrix(), was?.Rotation),
            was?.Scale.Clone());
    }

    /// <summary><paramref name="id"/>'s parent, its parent's parent, and so on up.</summary>
    private List<string> AncestorsOf(string id)
    {
        var above = new List<string>();
        var seen = new HashSet<string> { id };
        var at = _document[id]?.Parent;
        while (at is not null && seen.Add(at) && _document.Contains(at))
        {
            above.Add(at);
            at = _document[at]?.Parent;
        }

        return above;
    }
}
