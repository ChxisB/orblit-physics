using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

public sealed partial class ScenePhysics
{
    /// <summary>
    /// What the world is given to make a body: its shape and where it stands, in the world's own lists.
    /// </summary>
    private readonly record struct Placement(Shape Shape, double[] At, double[] Rotation);

    /// <summary>
    /// Makes the world agree with the document about <paramref name="ids"/>: each one that has a body gets
    /// it, where the document now puts it, and each one that has lost its body, or has gone, loses it from
    /// the world.
    /// </summary>
    private void Rebuild(IReadOnlyList<string> ids)
    {
        var added = new List<ulong>();
        var gone = new HashSet<ulong>();
        foreach (var id in ids)
        {
            var number = BodyOf(id);
            if (number is { } had)
            {
                TakeOut(had, gone);
            }

            if (_document[id]?[SceneComponents.Body] is not BodyComponent body)
            {
                Forget(id, number);
                continue;
            }

            var kept = number ?? _next++;
            _bodyOf[id] = kept;
            _entityOf[kept] = id;
            Add(kept, body, ScenePose.WorldOf(_document, id), _document[id]?[SceneComponents.Zone] as ZoneComponent);
            added.Add(kept);
        }

        Rejoin(ids, gone);
        Reignore(gone);
        Refit(ids);
        DropIdleHulls();
        ReadAdded(added);
    }

    private void TakeOut(ulong number, HashSet<ulong> gone)
    {
        Physics.Remove(number);
        _crouching.Remove(number);
        if (_compoundBodies.Remove(number))
        {
            Physics.DropCompound(number);
        }

        if (_meshBodies.Remove(number))
        {
            Physics.DropMesh(number);
        }

        _partHullKeys.Remove(number);
        gone.Add(number);
        _hullKeyOf.Remove(number);
    }

    /// <summary>An entity without a body has no walk, and none of the numbers that followed its body.</summary>
    private void Forget(string id, ulong? number)
    {
        _walks.Remove(id);
        if (number is not { } had)
        {
            return;
        }

        _bodyOf.Remove(id);
        _entityOf.Remove(had);
        _known.Remove(had);
        _previous.Remove(had);
        _latest.Remove(had);
    }

    /// <summary>
    /// Read back rather than remembered from what was sent: the world keeps floats and tidies the
    /// rotation it is handed, and a body whose first reading differed from what was sent would be written
    /// back on the first step without having moved.
    /// </summary>
    private void ReadAdded(List<ulong> added)
    {
        if (added.Count == 0)
        {
            return;
        }

        var read = new float[added.Count * 7];
        Physics.ReadInto(added.ToArray(), read);
        for (var i = 0; i < added.Count; i++)
        {
            _known[added[i]] = read.AsSpan(i * 7, 7).ToArray();

            // Both ends of the blend are the new place, so a body that was put somewhere does not slide
            // there from where it was.
            if (Smooth)
            {
                _previous[added[i]] = read.AsSpan(i * 7, 7).ToArray();
                _latest[added[i]] = read.AsSpan(i * 7, 7).ToArray();
            }
        }
    }

    private void Add(ulong number, BodyComponent body, Matrix4 world, ZoneComponent? zone)
    {
        var rotation = Quaternion.Identity();
        var scale = Vector3.Zero();
        world.Decompose(Vector3.Zero(), rotation, scale);
        scale = ScenePose.Absolute(scale);
        var at = world.Transformed3(body.Centre);
        var compound = body.Shape == BodyShape.Compound || !body.ShapeScale.Equals(Vector3.All(1));
        if (body.Shape == BodyShape.Hull)
        {
            LayHull(number, body, compound ? Vector3.All(1) : scale);
        }

        var placement = new Placement(
            ShapeFor(number, body, scale, compound),
            [at.X, at.Y, at.Z],
            [rotation.X, rotation.Y, rotation.Z, rotation.W]);
        if (IsCharacter(body) && !compound)
        {
            AddCharacter(number, body, placement);
            return;
        }

        AddBody(number, body, placement, zone);

        if (body.Surface.Length2 > 0)
        {
            Physics.SetSurface(number, [body.Surface.X, body.Surface.Y, body.Surface.Z]);
        }

        if (zone is not null)
        {
            Physics.SetZone(number, ZoneFor(zone));
        }

        // Quiet, so a body added asleep stays asleep with its controls.
        if (ControlsFor(body, scale) is { } controls)
        {
            Physics.SetControls(number, controls, quiet: true);
        }
    }

    private void AddCharacter(ulong number, BodyComponent body, Placement placement) => Physics.AddCharacter(
        number,
        shape: placement.Shape,
        at: placement.At,
        rotation: placement.Rotation,
        stepHeight: body.StepHeight,
        steepest: body.Steepest,
        skin: body.Skin,
        strength: body.Strength,
        friction: body.Friction,
        layers: LayersOf(body));

    private void AddBody(ulong number, BodyComponent body, Placement placement, ZoneComponent? zone) => Physics.Add(
        number,
        shape: placement.Shape,
        motion: body.Shape is BodyShape.Plane or BodyShape.Mesh ? PhysicsMotion.Fixed : MotionOf(body.Motion),
        at: placement.At,
        rotation: placement.Rotation,
        mass: body.Mass,
        friction: body.Friction,
        restitution: body.Restitution,
        linearDamping: body.LinearDamping,
        angularDamping: body.AngularDamping,
        layers: LayersOf(body),
        asleep: body.StartsAsleep,

        // A zone is a region, and only a trigger is one.
        trigger: body.Trigger || zone is not null,
        stay: body.Stay);

    private static Layers LayersOf(BodyComponent body) => new(unchecked((uint)body.Layers), unchecked((uint)body.Cares));

    private Shape ShapeFor(ulong number, BodyComponent body, Vector3 scale, bool compound)
    {
        if (body.Shape == BodyShape.Mesh)
        {
            return MeshOf(number, body, scale);
        }

        return compound && body.Shape != BodyShape.Plane
            ? CompoundOf(number, body, scale)
            : ShapeOf(number, body, scale);
    }

    /// <summary>
    /// How <paramref name="body"/> says it moves, at the size <paramref name="scale"/> makes it, or null
    /// when it says nothing and the world's own ways apply. The centre of mass grows with the entity, like
    /// the shape it is measured from. The inertia does not: it is a mass property the document gives
    /// outright.
    /// </summary>
    private static PhysicsControls? ControlsFor(BodyComponent body, Vector3 scale)
    {
        var centre = ScenePose.Times(body.CentreOfMass, scale);
        var inertia = body.Inertia;
        if (IsPlain(body, centre))
        {
            return null;
        }

        // A number the world would refuse takes every other control with it, so a hand edited file's
        // negative cap or inertia is read as none.
        return new PhysicsControls(
            Locks: body.Locks.Aggregate(PhysicsLock.None, (all, one) => all | Enum.Parse<PhysicsLock>(one.ToString())),
            GravityScale: body.GravityScale,
            MaxSpeed: Math.Max(body.MaxSpeed, 0),
            MaxSpin: Math.Max(body.MaxSpin, 0),
            Centre: [centre.X, centre.Y, centre.Z],
            Inertia: inertia.Length2 == 0
                ? null
                : [Math.Max(inertia.X, 0), Math.Max(inertia.Y, 0), Math.Max(inertia.Z, 0)],
            Continuous: body.Continuous);
    }

    private static bool IsPlain(BodyComponent body, Vector3 centre) =>
        body.Locks.Count == 0
        && body.GravityScale == 1
        && body.MaxSpeed == 0
        && body.MaxSpin == 0
        && !body.Continuous
        && centre.Length2 == 0
        && body.Inertia.Length2 == 0;

    private static PhysicsZone ZoneFor(ZoneComponent zone) => new(
        Gravity: zone.Gravity is { } gravity ? [gravity.X, gravity.Y, gravity.Z] : null,
        LinearDamping: zone.LinearDamping,
        AngularDamping: zone.AngularDamping,
        Priority: (int)Math.Clamp(zone.Priority, int.MinValue, int.MaxValue));

    /// <summary>
    /// Lays the hull <paramref name="body"/> is made of, at the size <paramref name="scale"/> makes it,
    /// unless the world already has it, and notes that body <paramref name="number"/> is made of it.
    /// </summary>
    /// <remarks>
    /// Corners that are not a whole number of points are not a hull, and neither are ones the world cannot
    /// cook: no hull is noted, and the body is not made.
    /// </remarks>
    private void LayHull(ulong number, BodyComponent body, Vector3 scale)
    {
        var corners = body.Hull;
        if (corners.Count % 3 != 0)
        {
            return;
        }

        var scaled = new double[corners.Count];
        for (var i = 0; i < scaled.Length; i++)
        {
            scaled[i] = corners[i] * scale[i % 3];
        }

        var key = KeyOf(scaled);
        _hullKeyOf[number] = key;
        if (_hullIdOf.ContainsKey(key))
        {
            return;
        }

        var id = _nextHull++;
        if (Physics.LayHull(id, scaled))
        {
            _hullIdOf[key] = id;
        }
    }

    /// <summary>
    /// Takes out of the world the hulls no body is made of any more. A hull is cooked once and kept while
    /// any body names it, and it goes with the last.
    /// </summary>
    private void DropIdleHulls()
    {
        var used = new HashSet<string>(_hullKeyOf.Values);
        foreach (var keys in _partHullKeys.Values)
        {
            used.UnionWith(keys);
        }

        // A copy, because a hull that goes is taken out of the map being read.
        foreach (var (key, id) in _hullIdOf.ToList())
        {
            if (!used.Contains(key) && Physics.DropHull(id))
            {
                _hullIdOf.Remove(key);
            }
        }
    }

    private static string KeyOf(IReadOnlyList<double> points) =>
        string.Join(',', points.Select(point => point.ToString("R", CultureInfo.InvariantCulture)));

    /// <summary>
    /// Body <paramref name="number"/>'s shape, made of <paramref name="body"/> at the size
    /// <paramref name="scale"/> makes it. A ball, a capsule and a cylinder cannot be stretched, only grown,
    /// so each takes the scale that makes it biggest in the directions it has: every one for a ball, the
    /// sideways ones for a radius and the upright one for a height. A hull is stretched along each axis.
    /// </summary>
    [SuppressMessage("Maintainability", "CA1502", Justification = "F2: a list of cases is data.")]
    private Shape ShapeOf(ulong number, BodyComponent body, Vector3 scale)
    {
        var size = body.Size;
        return body.Shape switch
        {
            BodyShape.Mesh => Shape.Mesh(_meshBodies.Contains(number) ? number : 0),
            BodyShape.Compound => Shape.Compound(_compoundBodies.Contains(number) ? number : 0),
            BodyShape.Box => Shape.Box(
                Math.Abs(size.X) * scale.X / 2,
                Math.Abs(size.Y) * scale.Y / 2,
                Math.Abs(size.Z) * scale.Z / 2),
            BodyShape.Sphere => Shape.Sphere(Math.Abs(body.Radius) * Math.Max(scale.X, Math.Max(scale.Y, scale.Z))),
            BodyShape.Capsule => Capsule(body.Radius, body.Height, scale),
            BodyShape.Cylinder => Shape.Cylinder(
                Math.Abs(body.Radius) * Math.Max(scale.X, scale.Z),
                Math.Abs(body.Height) * scale.Y / 2),

            // Zero names no hull, so corners that enclose nothing make no body.
            BodyShape.Hull => Shape.Hull(HullOf(number)),

            // The entity's own up, through the body's centre. The world turns and places the normal by the
            // body's pose, so it is given unturned here.
            BodyShape.Plane => Shape.Plane(0, 1, 0),
            _ => throw new InvalidOperationException($"Shape {body.Shape} is not one the world has."),
        };
    }

    private ulong HullOf(ulong number) =>
        _hullKeyOf.TryGetValue(number, out var key) && _hullIdOf.TryGetValue(key, out var id) ? id : 0;

    /// <summary>
    /// A capsule <paramref name="height"/> tall from tip to tip. The solver wants the straight part, from
    /// the middle, and none left means all ends and no middle: a ball.
    /// </summary>
    private static Shape Capsule(double radius, double height, Vector3 scale)
    {
        var round = Math.Abs(radius) * Math.Max(scale.X, scale.Z);
        var straight = (Math.Abs(height) * scale.Y / 2) - round;
        return straight > 0 ? Shape.Capsule(round, straight) : Shape.Sphere(round);
    }

    /// <summary>
    /// Whether the world can sweep <paramref name="body"/> as a character: it says it is one and is a shape
    /// that has a stance. Anything else it asks for is a driven body.
    /// </summary>
    private static bool IsCharacter(BodyComponent body) =>
        body.Motion == BodyMotion.Character
        && body.Shape is BodyShape.Capsule or BodyShape.Sphere or BodyShape.Box or BodyShape.Cylinder;

    /// <summary>
    /// Where the shape's middle is from the entity's origin, in the entity's own units. A crouching
    /// character's has come down with its head, because its feet stayed.
    /// </summary>
    private Vector3 CentreOf(BodyComponent body, ulong number)
    {
        var centre = body.Centre.Clone();
        if (_crouching.Contains(number))
        {
            centre.Y -= (Math.Abs(body.Height) - Math.Abs(body.CrouchHeight)) / 2;
        }

        return centre;
    }

    /// <summary>The scale <paramref name="entity"/> has in the world.</summary>
    private Vector3 ScaleOf(string entity)
    {
        var scale = Vector3.Zero();
        ScenePose.WorldOf(_document, entity).Decompose(Vector3.Zero(), Quaternion.Identity(), scale);
        return ScenePose.Absolute(scale);
    }

    /// <summary>Asks every character with a walk what it wants for the step about to be taken.</summary>
    private void Drive()
    {
        foreach (var (entity, walk) in _walks)
        {
            if (!_bodyOf.TryGetValue(entity, out var number))
            {
                continue;
            }

            Physics.Drive(number, velocity: walk.Ask(Step, Physics.FootingOf(number)));
        }

        foreach (var car in _cars.Values)
        {
            car.Vehicle.Update(Step);
        }
    }

    /// <summary>
    /// Makes the cars agree with the document for every chassis entity in <paramref name="ids"/>, and for
    /// the car of every wheel entity in it or that was one. A car made again has lost its tyres' bend, like
    /// a body made again its velocity.
    /// </summary>
    private void Refit(IReadOnlyList<string> ids)
    {
        foreach (var id in CarsAmong(ids))
        {
            _cars.Remove(id);
            var car = _bodyOf.TryGetValue(id, out var number)
                ? Car.Make(_document, id, Physics, number, _gravity)
                : null;
            if (car is null)
            {
                _pilots.Remove(id);
                continue;
            }

            _cars[id] = car;
            Steer(id, car);
        }
    }

    private List<string> CarsAmong(IReadOnlyList<string> ids)
    {
        var touched = new HashSet<string>(ids);
        var cars = ids
            .Where(id => _cars.ContainsKey(id) || _document[id]?[SceneComponents.Vehicle] is VehicleComponent)
            .ToList();
        foreach (var id in ids)
        {
            if (_document[id] is { Parent: { } parent } entity && entity[SceneComponents.Wheel] is WheelComponent)
            {
                cars.Add(parent);
            }
        }

        cars.AddRange(_cars.Where(entry => entry.Value.Wheels.Any(touched.Contains)).Select(entry => entry.Key));
        return Unique(cars);
    }

    private void Steer(string id, Car car)
    {
        if (!_pilots.TryGetValue(id, out var input))
        {
            return;
        }

        try
        {
            car.Vehicle.SetInput(input);
        }
        catch (ArgumentException)
        {
            // An edit took away a gear the driver was holding.
            _pilots.Remove(id);
        }
    }

    private Shape MeshOf(ulong number, BodyComponent body, Vector3 scale)
    {
        var outer = ScenePose.Times(scale, body.ShapeScale);
        if (!IsFiniteAndPositive(outer))
        {
            return Shape.Mesh(0);
        }

        var vertices = body.MeshVertices;
        var points = new double[vertices.Count];
        for (var i = 0; i < points.Length; i++)
        {
            points[i] = vertices[i] * outer[i % 3];
        }

        // An index past what the world holds is a mesh it cannot make.
        if (!body.MeshIndices.All(index => index is >= int.MinValue and <= int.MaxValue)
            || !Physics.LayMesh(number, points, [.. body.MeshIndices.Select(index => (int)index)]))
        {
            return Shape.Mesh(0);
        }

        _meshBodies.Add(number);
        return Shape.Mesh(number);
    }

    private static bool IsFiniteAndPositive(Vector3 size) =>
        new[] { size.X, size.Y, size.Z }.All(part => double.IsFinite(part) && part > 0);

    private Shape CompoundOf(ulong number, BodyComponent body, Vector3 scale)
    {
        var outer = ScenePose.Times(scale, body.ShapeScale);
        try
        {
            var parts = PartsOf(number, body);
            if (parts is not null && Physics.LayCompound(number, parts, [outer.X, outer.Y, outer.Z]))
            {
                _compoundBodies.Add(number);
                return Shape.Compound(number);
            }
        }
        catch (ArgumentException)
        {
            // Hand edited invalid geometry stays in the document as an inert body.
        }

        _partHullKeys.Remove(number);
        return Shape.Compound(0);
    }

    /// <summary>The parts of a compound body, one for a body that is only scaled, or null when there are none or too many.</summary>
    private List<ShapePart>? PartsOf(ulong number, BodyComponent body)
    {
        if (body.Shape != BodyShape.Compound)
        {
            return [new ShapePart(ShapeOf(number, body, Vector3.All(1)))];
        }

        if (body.Parts.Count == 0 || body.Parts.Count > 64)
        {
            return null;
        }

        return [.. body.Parts.Select(part => CompoundParts.PartOf(part, points => PartHull(number, points)))];
    }

    private ulong PartHull(ulong number, IReadOnlyList<double> points)
    {
        if (points.Count % 3 != 0)
        {
            return 0;
        }

        var key = KeyOf(points);
        if (!_partHullKeys.TryGetValue(number, out var keys))
        {
            keys = [];
            _partHullKeys[number] = keys;
        }

        keys.Add(key);
        if (_hullIdOf.TryGetValue(key, out var known))
        {
            return known;
        }

        var id = _nextHull++;
        if (!Physics.LayHull(id, points))
        {
            return 0;
        }

        _hullIdOf[key] = id;
        return id;
    }

    private static PhysicsMotion MotionOf(BodyMotion motion) => motion switch
    {
        BodyMotion.Fixed => PhysicsMotion.Fixed,
        BodyMotion.Driven or BodyMotion.Character => PhysicsMotion.Driven,
        BodyMotion.Free => PhysicsMotion.Free,
        _ => throw new InvalidOperationException($"Motion {motion} is not one the world has."),
    };
}
