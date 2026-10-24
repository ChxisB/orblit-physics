using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>
/// Particles joined by links that hold a distance.
/// </summary>
/// <remarks>
/// <para>
/// A cloth is a grid of particles with links along its rows and columns, along
/// both diagonals so it resists shear, and two apart so it resists folding. A
/// link two apart shortens only with the square of the bend, so a cloth drapes
/// like silk: it resists a sharp fold and gives to a gentle curve. A rope is a
/// chain. Each step moves the particles under gravity, then pulls every link
/// back to its length. The links are solved as position constraints, so they
/// stay stable at a stiffness a spring would blow up at.
/// </para>
/// <para>
/// A balloon is a closed mesh of particles that also holds the volume it began
/// with, so a body of it sags under gravity and squashes under a load without
/// going flat. A pillow is two sheets that close up round their rims.
/// </para>
/// <para>
/// A particle can be pinned, which holds it where the caller puts it.
/// </para>
/// <para>
/// Given an <see cref="ISoftSurface"/>, the particles stay off what it
/// describes and grip it by <see cref="SoftSpec.Friction"/>. Each step looks
/// once, ahead of every particle, for the surface it will meet, and holds it
/// to that plane in the sub-steps. A particle is a point with a skin, so a
/// surface thinner than the gap between two particles lets one through, and
/// the particles do not meet one another. A body asleep does not look, so wake
/// it when the surface moves.
/// </para>
/// </remarks>
public sealed class SoftBody
{
    /// <summary>
    /// A body slower than this, everywhere, for <see cref="SleepTime"/> seconds
    /// stops being stepped until something wakes it.
    /// </summary>
    public const double SleepSpeed = 0.005;

    public const double SleepTime = 1.0;

    private readonly MeshVolume? _volume;
    private readonly double[] _x;
    private readonly double[] _prev;
    private readonly double[] _v;
    private readonly double[] _w;
    private readonly double _inverseMass;
    private readonly LinkSet _links;

    /// <summary>
    /// For each particle that will meet a surface this step, the plane it must
    /// stay in front of: a normal and the offset along it.
    /// </summary>
    private readonly double[] _plane;
    private readonly byte[] _touch;
    private readonly OrderedDictionary<int, Anchor> _anchors = [];

    /// <summary>The wind at the middle of the body, as of the last step.</summary>
    private Vector3 _wind = Vector3.Zero();

    /// <summary>Seconds the largest move has been under <see cref="SleepSpeed"/>.</summary>
    private double _still;

    // F2: seven parameters, all required and called by name.
    private SoftBody(
        SoftSpec spec,
        double[] x,
        double mass,
        LinkSet links,
        ISoftSurface? surface,
        ISoftAir? air,
        MeshVolume? volume)
    {
        Spec = spec;
        Surface = surface;
        Air = air;
        _x = x;
        _links = links;
        _volume = volume;
        _prev = [.. x];
        _v = new double[x.Length];
        _inverseMass = 1.0 / mass;
        _w = new double[x.Length / 3];
        Array.Fill(_w, _inverseMass);
        _plane = new double[x.Length / 3 * 4];
        _touch = new byte[x.Length / 3];
    }

    public SoftSpec Spec { get; }

    /// <summary>What the particles stay off, or null for a body that falls through everything.</summary>
    public ISoftSurface? Surface { get; }

    /// <summary>The wind the particles are dragged along by, or null for still air.</summary>
    public ISoftAir? Air { get; }

    /// <summary>How many particles there are. A cloth numbers them along its rows.</summary>
    public int Count => _x.Length / 3;

    /// <summary>Whether the solver has stopped for lack of anything to do.</summary>
    public bool Asleep { get; private set; }

    /// <summary>
    /// The volume a balloon encloses now, in cubic metres, or null for a body
    /// that is not closed.
    /// </summary>
    public double? Volume => _volume?.VolumeOf(_x);

    /// <summary>The fastest particle, in metres a second.</summary>
    public double MaxSpeed
    {
        get
        {
            var most = 0.0;
            for (var i = 0; i < _v.Length; i += 3)
            {
                most = Math.Max(most, (_v[i] * _v[i]) + (_v[i + 1] * _v[i + 1]) + (_v[i + 2] * _v[i + 2]));
            }

            return Math.Sqrt(most);
        }
    }

    /// <summary>
    /// The most any edge or diagonal is stretched or squeezed, as a fraction of
    /// its length. The links that resist folding are not counted.
    /// </summary>
    public double MaxStretch
    {
        get
        {
            var most = 0.0;
            for (var l = 0; l < _links.Length; l++)
            {
                if (_links.Bends[l] != 0)
                {
                    continue;
                }

                most = Math.Max(most, Math.Abs((LengthOf(l) / _links.Rest[l]) - 1));
            }

            return most;
        }
    }

    /// <summary>How fast the air takes a particle up to its own speed, per second.</summary>
    private double Drag => Air is null ? 0 : Spec.WindDrag;

    /// <summary>
    /// A sheet of <paramref name="columns"/> by <paramref name="rows"/>
    /// particles. <paramref name="across"/> is the offset from one particle to
    /// the next along a row and <paramref name="down"/> to the next row, so
    /// their lengths are the spacing and their angle is the shear.
    /// <paramref name="particleMass"/> is the mass of one particle in kilograms.
    /// </summary>
    // F2: nine parameters, the first five required and the rest called by name.
    public static SoftBody Cloth(
        int columns,
        int rows,
        Vector3 origin,
        Vector3 across,
        Vector3 down,
        double particleMass = 0.01,
        SoftSpec? spec = null,
        ISoftSurface? surface = null,
        ISoftAir? air = null)
    {
        if (columns < 2 || rows < 2)
        {
            throw new ArgumentException("A cloth needs 2 columns and 2 rows at least.");
        }

        CheckMass(particleMass);
        var soft = spec ?? new SoftSpec();
        var x = new double[columns * rows * 3];
        for (var j = 0; j < rows; j++)
        {
            for (var i = 0; i < columns; i++)
            {
                var p = origin + (across * (double)i) + (down * (double)j);
                Packed.Write(p, x, ((j * columns) + i) * 3);
            }
        }

        return new SoftBody(soft, x, particleMass, LinkBuilder.Cloth(columns, rows, x, soft), surface, air, null);
    }

    /// <summary>
    /// A closed surface of <paramref name="triangles"/> over
    /// <paramref name="points"/>, three indices into the points for each
    /// triangle, wound so the normals point out. Every edge is a link, and the
    /// volume the points enclose is held as a constraint. A
    /// <paramref name="volumeCompliance"/> above zero lets it give under load.
    /// </summary>
    /// <remarks>
    /// Throws for a surface with a hole in it or one wound inside out.
    /// shortcut: no link resists folding across an edge, so the shape is held
    /// by the volume alone and a balloon wrinkles when squeezed. Add bend links
    /// across each shared edge if a game needs it taut.
    /// </remarks>
    // F2: seven parameters, the first two required and the rest called by name.
    public static SoftBody Balloon(
        IReadOnlyList<Vector3> points,
        IReadOnlyList<int> triangles,
        double particleMass = 0.01,
        double volumeCompliance = 0,
        SoftSpec? spec = null,
        ISoftSurface? surface = null,
        ISoftAir? air = null)
    {
        CheckMass(particleMass);
        if (!(volumeCompliance >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(volumeCompliance), volumeCompliance, "Must not be negative.");
        }

        var soft = spec ?? new SoftSpec();
        var x = Flatten(points);
        var mesh = new ClosedMesh(points.Count, triangles);
        var links = new LinkBuilder(x);
        foreach (var (a, b) in mesh.Edges)
        {
            links.Stretch(a, b, soft.StretchCompliance);
        }

        var volume = new MeshVolume(mesh, x, volumeCompliance);
        return new SoftBody(soft, x, particleMass, links.Build(), surface, air, volume);
    }

    /// <summary>
    /// A chain through <paramref name="points"/>, each joined to the next. It
    /// resists folding across the particle between two links.
    /// </summary>
    // F2: five parameters, the first required and the rest called by name.
    public static SoftBody Rope(
        IReadOnlyList<Vector3> points,
        double particleMass = 0.01,
        SoftSpec? spec = null,
        ISoftSurface? surface = null,
        ISoftAir? air = null)
    {
        if (points.Count < 2)
        {
            throw new ArgumentException("A rope needs 2 particles at least.", nameof(points));
        }

        CheckMass(particleMass);
        var soft = spec ?? new SoftSpec();
        var x = Flatten(points);
        var links = new LinkBuilder(x);
        for (var i = 0; i + 1 < points.Count; i++)
        {
            links.Stretch(i, i + 1, soft.StretchCompliance);
        }

        for (var i = 0; i + 2 < points.Count; i++)
        {
            links.Bend(i, i + 2, soft.BendCompliance);
        }

        return new SoftBody(soft, x, particleMass, links.Build(), surface, air, null);
    }

    public Vector3 PositionOf(int i)
    {
        CheckParticle(i);
        return Packed.Read(_x, i * 3);
    }

    public Vector3 VelocityOf(int i)
    {
        CheckParticle(i);
        return Packed.Read(_v, i * 3);
    }

    public bool IsPinned(int i)
    {
        CheckParticle(i);
        return _w[i] == 0;
    }

    /// <summary>Holds particle <paramref name="i"/> where it is. It stops falling and moves only by <see cref="Place"/>.</summary>
    public void Pin(int i)
    {
        CheckParticle(i);
        _w[i] = 0;
        Array.Clear(_v, i * 3, 3);
        Wake();
    }

    /// <summary>Lets a pinned or attached particle go.</summary>
    public void Release(int i)
    {
        CheckParticle(i);
        _anchors.Remove(i);
        _w[i] = _inverseMass;
        Wake();
    }

    /// <summary>
    /// Holds particle <paramref name="i"/> to the point <paramref name="where"/>
    /// answers, which is asked at the start of every step. The particle jumps
    /// to it, so a point that moves faster than the body can follow stretches
    /// the links.
    /// </summary>
    /// <remarks>
    /// With a <paramref name="limit"/>, in newtons, the particle lets go when
    /// the body pulls on it harder than that, averaged over one step.
    /// <see cref="BodyPoint.Of"/> gives the point for a place on a physics
    /// body. A bone is any closure of the caller's own. Placing an attached
    /// particle by hand is undone by the next step.
    /// </remarks>
    public void Attach(int i, Func<Vector3> where, double? limit = null)
    {
        CheckParticle(i);
        if (limit is not null && !(limit > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(limit), limit, "Must be above zero.");
        }

        var ending = new List<int>();
        for (var l = 0; l < _links.Length; l++)
        {
            if (_links.A[l] == i || _links.B[l] == i)
            {
                ending.Add(l);
            }
        }

        Pin(i);
        _anchors[i] = new Anchor(i, where, limit, ending);
    }

    /// <summary>Lets an attached particle hang free. A particle pinned by hand stays.</summary>
    public void Detach(int i)
    {
        if (_anchors.Remove(i))
        {
            Release(i);
        }
    }

    public bool IsAttached(int i) => _anchors.ContainsKey(i);

    /// <summary>
    /// The force, in newtons, the body puts on attached particle
    /// <paramref name="i"/>, averaged over the last step. It points the way the
    /// body pulls. It is the last value while the body sleeps, which is the
    /// load it is holding up.
    /// </summary>
    public Vector3 PullOf(int i) =>
        _anchors.TryGetValue(i, out var anchor)
            ? anchor.Pull.Clone()
            : throw new InvalidOperationException($"Particle {i} is not attached.");

    /// <summary>Moves a pinned particle. The links pull the rest after it.</summary>
    public void Place(int i, Vector3 at)
    {
        CheckParticle(i);
        if (_w[i] != 0)
        {
            throw new InvalidOperationException($"Particle {i} is not pinned.");
        }

        Packed.Write(at, _x, i * 3);
        Packed.Write(at, _prev, i * 3);
        Wake();
    }

    public void Wake()
    {
        Asleep = false;
        _still = 0;
    }

    /// <summary>
    /// Moves the body on by <paramref name="dt"/> seconds, in sub-steps. A span
    /// above <see cref="SoftSpec.MaxStep"/> is cut to it.
    /// </summary>
    public void Step(double dt)
    {
        if (!(dt > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "Must be above zero.");
        }

        Follow();
        Blow();
        if (Asleep)
        {
            return;
        }

        var span = Math.Min(dt, SoftSpec.MaxStep);
        var pieces = (int)Math.Ceiling(span / Spec.SubstepLength);
        var h = span / pieces;
        foreach (var anchor in _anchors.Values)
        {
            anchor.Pull.SetZero();
        }

        Look(span);
        for (var s = 0; s < pieces; s++)
        {
            Integrate(h);
            Solve(h);
            Gather(h);
            Derive(h);
        }

        Tear(pieces);
        Settle(span);
    }

    private static void CheckMass(double mass)
    {
        if (!(mass > 0 && double.IsFinite(mass)))
        {
            throw new ArgumentOutOfRangeException(nameof(mass), mass, "Must be above zero.");
        }
    }

    private static double[] Flatten(IReadOnlyList<Vector3> points)
    {
        var x = new double[points.Count * 3];
        for (var i = 0; i < points.Count; i++)
        {
            Packed.Write(points[i], x, i * 3);
        }

        return x;
    }

    private void CheckParticle(int i)
    {
        if (i < 0 || i >= Count)
        {
            throw new ArgumentOutOfRangeException(nameof(i), i, $"Must be 0 to {Count - 1}.");
        }
    }

    /// <summary>
    /// Asks each anchor where its particle goes. A body whose anchors have not
    /// moved stays asleep.
    /// </summary>
    private void Follow()
    {
        foreach (var anchor in _anchors.Values)
        {
            var target = anchor.Where();
            if (PositionOf(anchor.Particle).DistanceToSquared(target) > 1e-18)
            {
                Place(anchor.Particle, target);
            }
        }
    }

    /// <summary>Samples the wind, and wakes the body if it has changed.</summary>
    private void Blow()
    {
        if (Air is null)
        {
            return;
        }

        var wind = Air.WindAt(Centre());
        if (wind.DistanceToSquared(_wind) > 1e-12)
        {
            Wake();
        }

        _wind = wind;
    }

    private Vector3 Centre()
    {
        var sum = Vector3.Zero();
        for (var i = 0; i < Count; i++)
        {
            sum.X += _x[i * 3];
            sum.Y += _x[(i * 3) + 1];
            sum.Z += _x[(i * 3) + 2];
        }

        return sum / (double)Count;
    }

    /// <summary>
    /// Adds the force each link put on its anchor in this sub-step. A link
    /// pushes its ends apart by <c>lambda</c> along its length, over the
    /// sub-step squared, so a stretched link, whose <c>lambda</c> is negative,
    /// pulls them together.
    /// </summary>
    private void Gather(double h)
    {
        foreach (var anchor in _anchors.Values)
        {
            foreach (var l in anchor.Links)
            {
                var a = _links.A[l] * 3;
                var b = _links.B[l] * 3;
                var length = LengthOf(l);
                if (length < 1e-12)
                {
                    continue;
                }

                var sign = _links.A[l] == anchor.Particle ? 1.0 : -1.0;
                var scale = sign * _links.Lambda[l] / (length * h * h);
                anchor.Pull.X += (_x[a] - _x[b]) * scale;
                anchor.Pull.Y += (_x[a + 1] - _x[b + 1]) * scale;
                anchor.Pull.Z += (_x[a + 2] - _x[b + 2]) * scale;
            }
        }
    }

    /// <summary>Turns the sums into means, and lets go of what was pulled too hard.</summary>
    private void Tear(int pieces)
    {
        foreach (var anchor in _anchors.Values.ToList())
        {
            anchor.Pull.Scale(1.0 / pieces);
            if (anchor.Limit is { } limit && anchor.Pull.Length > limit)
            {
                Release(anchor.Particle);
            }
        }
    }

    private void Integrate(double h)
    {
        var drag = Drag;
        var g = Spec.Gravity + (_wind * drag);
        var keep = Math.Max(0.0, 1 - (Spec.Damping * h)) / (1 + (drag * h));
        for (var i = 0; i < Count; i++)
        {
            var k = i * 3;
            for (var c = 0; c < 3; c++)
            {
                _prev[k + c] = _x[k + c];
                if (_w[i] == 0)
                {
                    continue;
                }

                _v[k + c] = (_v[k + c] + (g[c] * h)) * keep;
                _x[k + c] += _v[k + c] * h;
            }
        }
    }

    private void Solve(double h)
    {
        Array.Clear(_links.Lambda);
        _volume?.Restart();
        var inverseSquare = 1 / (h * h);
        for (var pass = 0; pass < Spec.Iterations; pass++)
        {
            for (var l = 0; l < _links.Length; l++)
            {
                Project(l, inverseSquare);
            }

            _volume?.Project(_x, _w, inverseSquare);
            Press();
        }
    }

    /// <summary>
    /// Finds the surface each free particle would meet this step if nothing
    /// turned it. The ball is a tenth wider than the skin, so a particle held
    /// exactly one skin off a surface still meets it, rather than grazing it
    /// and being let go every other step.
    /// </summary>
    /// <remarks>
    /// A ball that overlaps a curved surface answers a point inside it, so a
    /// particle is held a tenth short of a full skin from one.
    /// </remarks>
    private void Look(double span)
    {
        if (Surface is null)
        {
            return;
        }

        var skin = Spec.Thickness;
        var fall = (Spec.Gravity + (_wind * Drag)) * (0.5 * span * span);
        for (var i = 0; i < Count; i++)
        {
            _touch[i] = 0;
            if (_w[i] == 0)
            {
                continue;
            }

            var move = (Packed.Read(_v, i * 3) * span) + fall;
            var length = move.Length;

            // A particle with nowhere to go still looks down, for a surface it is in.
            var dir = length > 1e-9 ? move / length : new Vector3(0, -1, 0);
            var contact = Surface.Sweep(Packed.Read(_x, i * 3), dir, length, skin * 1.1);
            if (contact is not null)
            {
                Hold(i, contact);
            }
        }
    }

    private void Hold(int i, SoftContact contact)
    {
        Packed.Write(contact.Normal, _plane, i * 4);
        _plane[(i * 4) + 3] = contact.Normal.Dot(contact.At) + Spec.Thickness;
        _touch[i] = 1;
    }

    /// <summary>
    /// Moves every particle that has gone behind its plane back in front of it,
    /// and takes the slide it made on the way in. What a particle sank in this
    /// sub-step is undone whole, so a fast one is caught. A depth it began the
    /// sub-step with is undone by no more than one skin, and not kept as speed,
    /// so a surface that closes on the body moves it out over several sub-steps
    /// rather than throwing it.
    /// </summary>
    private void Press()
    {
        if (Surface is null)
        {
            return;
        }

        for (var i = 0; i < Count; i++)
        {
            if (_touch[i] != 0)
            {
                Push(i);
            }
        }
    }

    private void Push(int i)
    {
        var k = i * 3;
        var q = i * 4;
        var (nx, ny, nz) = (_plane[q], _plane[q + 1], _plane[q + 2]);
        var depth = _plane[q + 3] - (nx * _x[k]) - (ny * _x[k + 1]) - (nz * _x[k + 2]);
        if (depth <= 0)
        {
            return;
        }

        var sank = Math.Max(
            0.0,
            -(((_x[k] - _prev[k]) * nx) + ((_x[k + 1] - _prev[k + 1]) * ny) + ((_x[k + 2] - _prev[k + 2]) * nz)));
        var carried = Math.Min(Math.Max(0.0, depth - sank), Spec.Thickness);
        var push = Math.Min(depth, sank) + carried;
        _x[k] += nx * push;
        _x[k + 1] += ny * push;
        _x[k + 2] += nz * push;
        _prev[k] += nx * carried;
        _prev[k + 1] += ny * carried;
        _prev[k + 2] += nz * carried;

        // The slide over the surface in this sub-step, less what the grip takes.
        var dx = _x[k] - _prev[k];
        var dy = _x[k + 1] - _prev[k + 1];
        var dz = _x[k + 2] - _prev[k + 2];
        var along = (dx * nx) + (dy * ny) + (dz * nz);
        var (sx, sy, sz) = (dx - (nx * along), dy - (ny * along), dz - (nz * along));
        var slide = Math.Sqrt((sx * sx) + (sy * sy) + (sz * sz));
        if (slide == 0)
        {
            return;
        }

        var grip = Math.Min(1.0, Spec.Friction * push / slide);
        _x[k] -= sx * grip;
        _x[k + 1] -= sy * grip;
        _x[k + 2] -= sz * grip;
    }

    /// <summary>
    /// Pulls the two ends of link <paramref name="l"/> towards its length by an
    /// amount that follows its compliance. <c>lambda</c> is the push the link
    /// has made so far in this sub-step, so a second pass corrects the first
    /// rather than adding to it.
    /// </summary>
    private void Project(int l, double inverseSquare)
    {
        var a = _links.A[l] * 3;
        var b = _links.B[l] * 3;
        var dx = _x[a] - _x[b];
        var dy = _x[a + 1] - _x[b + 1];
        var dz = _x[a + 2] - _x[b + 2];
        var length = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
        var wa = _w[a / 3];
        var wb = _w[b / 3];
        var alpha = _links.Alpha[l] * inverseSquare;
        var weight = wa + wb + alpha;

        // Coincident ends have no direction to push along, and two pinned ends
        // with no give cannot move.
        if (length < 1e-12 || weight == 0)
        {
            return;
        }

        var change = (-(length - _links.Rest[l]) - (alpha * _links.Lambda[l])) / weight;
        _links.Lambda[l] += change;
        var s = change / length;
        _x[a] += wa * s * dx;
        _x[a + 1] += wa * s * dy;
        _x[a + 2] += wa * s * dz;
        _x[b] -= wb * s * dx;
        _x[b + 1] -= wb * s * dy;
        _x[b + 2] -= wb * s * dz;
    }

    private void Derive(double h)
    {
        for (var i = 0; i < Count; i++)
        {
            if (_w[i] == 0)
            {
                continue;
            }

            for (var c = i * 3; c < (i * 3) + 3; c++)
            {
                _v[c] = (_x[c] - _prev[c]) / h;
            }
        }
    }

    private void Settle(double span)
    {
        _still = MaxSpeed < SleepSpeed ? _still + span : 0;
        Asleep = _still >= SleepTime;
    }

    private double LengthOf(int l)
    {
        var a = _links.A[l] * 3;
        var b = _links.B[l] * 3;
        var dx = _x[a] - _x[b];
        var dy = _x[a + 1] - _x[b + 1];
        var dz = _x[a + 2] - _x[b + 2];
        return Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
    }
}
