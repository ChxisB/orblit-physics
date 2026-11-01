using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

/// <summary>Where an entity sits, and how a pose is blended, in the terms the physics world uses.</summary>
internal static class ScenePose
{
    /// <summary>
    /// How close to a quarter turn about Y a rotation must be, as the cosine of the Y angle, for X to be
    /// let go. Float rounding is the whole of what is there to read when nothing is known of the angles
    /// it had.
    /// </summary>
    private const double RoundingBand = 1e-6;

    /// <summary>
    /// The same, when the angles it had are known. A body that lands flat keeps a tilt of a few
    /// hundred-thousandths of a radian that nothing levels, and X read from that is a large angle that
    /// means nothing, so the crate would be shown turned about X and back about Z.
    /// </summary>
    private const double SettledBand = 1e-3;

    /// <summary>
    /// Degrees about X, Y and Z as a rotation, composed Z, then Y, then X.
    /// </summary>
    /// <remarks>
    /// The order the stage draws a scene in, and so the only order that puts a body where its entity is
    /// drawn. Euler angles do not commute: composed any other way, a crate turned about two axes is
    /// simulated somewhere near where it is drawn, which is worse than somewhere obviously wrong.
    /// </remarks>
    public static Matrix4 RotationOf(Vector3 degrees)
    {
        var turn = Matrix4.RotationZ(Doubles.Radians(degrees.Z));
        turn.Multiply(Matrix4.RotationY(Doubles.Radians(degrees.Y)));
        turn.Multiply(Matrix4.RotationX(Doubles.Radians(degrees.X)));
        return turn;
    }

    /// <summary>
    /// Where an entity sits relative to its parent. No transform is no offset: a folder in the outliner
    /// does not move what is in it.
    /// </summary>
    public static Matrix4 LocalOf(TransformComponent? transform)
    {
        if (transform is null)
        {
            return Matrix4.Identity();
        }

        var local = Matrix4.Identity();
        local.SetTranslation(transform.Position);
        local.Multiply(RotationOf(transform.Rotation));
        local.Multiply(Matrix4.Diagonal3Values(transform.Scale.X, transform.Scale.Y, transform.Scale.Z));
        return local;
    }

    /// <summary>
    /// Where an entity sits in the world: its own transform under every one of its parents'. The
    /// identity for null or an id the document does not have.
    /// </summary>
    /// <remarks>
    /// <paramref name="overriding"/> stands in for the transforms of entities that have moved but are
    /// not in <paramref name="document"/> yet, so a pass that moves a parent and then its child measures
    /// the child against where the parent went.
    /// </remarks>
    public static Matrix4 WorldOf(
        SceneDocument document,
        string? id,
        IReadOnlyDictionary<string, TransformComponent>? overriding = null)
    {
        var chain = new List<SceneEntity>();
        var seen = new HashSet<string>();
        var at = id;

        // A parent that is missing, or that is the entity itself, is the root. That is the rule the
        // stage follows, so a document it would draw is one this can read. The set is only a guard:
        // decode breaks every loop long before here.
        while (at is not null && seen.Add(at))
        {
            var entity = document[at];
            if (entity is null)
            {
                break;
            }

            chain.Add(entity);
            at = entity.Parent;
        }

        var world = Matrix4.Identity();
        for (var i = chain.Count - 1; i >= 0; i--)
        {
            var entity = chain[i];
            var transform = overriding is not null && overriding.TryGetValue(entity.Id, out var placed)
                ? placed
                : entity[SceneComponents.Transform] as TransformComponent;
            world.Multiply(LocalOf(transform));
        }

        return world;
    }

    /// <summary>
    /// A rotation as degrees about X, Y and Z: the inverse of <see cref="RotationOf"/>.
    /// </summary>
    /// <remarks>
    /// <para>
    /// Every rotation can be written two ways in these three angles, and any of them can have a whole
    /// turn added. Given <paramref name="near"/>, the angles the entity had before, the answer is the
    /// writing closest to those, so a crate that tips over turns smoothly in the inspector instead of
    /// jumping between two sets of numbers that happen to mean the same thing.
    /// </para>
    /// <para>
    /// A quarter turn about Y, a crate turned to face sideways, is where X and Z turn about the same
    /// axis and only how far they turn between them is known. There X keeps the angle
    /// <paramref name="near"/> gives it, or none, and Z takes the rest. With <paramref name="near"/> that
    /// holds within a thousandth of the quarter turn, and the angles then differ from the rotation by
    /// about that.
    /// </para>
    /// </remarks>
    public static Vector3 EulerOf(Matrix3 rotation, Vector3? near = null)
    {
        var cosY = Math.Sqrt((rotation.Entry(0, 0) * rotation.Entry(0, 0)) + (rotation.Entry(1, 0) * rotation.Entry(1, 0)));
        var y = Math.Atan2(-rotation.Entry(2, 0), cosY);
        var band = near is null ? RoundingBand : SettledBand;

        // Near the quarter turn the angle X is read from is two numbers each about as small as the
        // noise in them, and means nothing. It is then the angle near has, in whichever of the two
        // writings is the one nearby, which is a half turn out in the other.
        var held = Doubles.Radians(near?.X ?? 0);
        Vector3[] writings = cosY > band
            ? [Written(rotation, y, Math.Atan2(rotation.Entry(2, 1), rotation.Entry(2, 2)))]
            : [Written(rotation, y, held), Written(rotation, y, held - Math.PI)];
        if (near is null)
        {
            return writings[0];
        }

        // The other writing of the same rotation: half a turn about X and Z, and Y reflected about a
        // quarter turn.
        var candidates = new List<Vector3>();
        foreach (var found in writings)
        {
            candidates.Add(Unwrapped(found, near));
            candidates.Add(Unwrapped(new Vector3(found.X + 180, 180 - found.Y, found.Z + 180), near));
        }

        var best = candidates[0];
        foreach (var candidate in candidates.Skip(1))
        {
            if (Apart(best, near) > Apart(candidate, near))
            {
                best = candidate;
            }
        }

        return best;
    }

    /// <summary>
    /// Writes into <paramref name="output"/> the pose <paramref name="alpha"/> of the way from
    /// <paramref name="from"/> to <paramref name="to"/>, where <paramref name="alpha"/> is 0 to 1. A pose
    /// is seven floats: translation xyz then rotation xyzw, as the world reports it.
    /// </summary>
    /// <remarks>
    /// The translation goes in a straight line. The rotation goes the short way round along the arc, so a
    /// body turns at an even rate across the blend. A rotation that has not changed is copied as it is. A
    /// body at rest is the same pose in both, and rounding in the blend must not be read as movement.
    /// </remarks>
    public static void BlendPose(ReadOnlySpan<float> from, ReadOnlySpan<float> to, double alpha, Span<float> output)
    {
        for (var i = 0; i < 3; i++)
        {
            double start = from[i];
            output[i] = (float)(start + (((double)to[i] - start) * alpha));
        }

        if (SameTurn(from, to))
        {
            to.Slice(3, 4).CopyTo(output.Slice(3, 4));
            return;
        }

        // A turn and its negative are the same turn. Take the one that is near.
        var dot = ((double)from[3] * to[3]) + ((double)from[4] * to[4]) + ((double)from[5] * to[5]) + ((double)from[6] * to[6]);
        var cosine = Math.Abs(dot);
        var weightFrom = 1 - alpha;
        var weightTo = alpha;

        // Close together the arc is a line, and its sine is too small to divide by.
        if (cosine < 0.9995)
        {
            var angle = Math.Acos(cosine);
            var sine = Math.Sin(angle);
            weightFrom = Math.Sin((1 - alpha) * angle) / sine;
            weightTo = Math.Sin(alpha * angle) / sine;
        }

        if (dot < 0)
        {
            weightTo = -weightTo;
        }

        var turn = new Quaternion(
            (from[3] * weightFrom) + (to[3] * weightTo),
            (from[4] * weightFrom) + (to[4] * weightTo),
            (from[5] * weightFrom) + (to[5] * weightTo),
            (from[6] * weightFrom) + (to[6] * weightTo));
        turn.Normalize();
        output[3] = (float)turn.X;
        output[4] = (float)turn.Y;
        output[5] = (float)turn.Z;
        output[6] = (float)turn.W;
    }

    /// <summary>
    /// The angles for <paramref name="rotation"/> given Y and X, in degrees. Z is read from what is left
    /// once X is taken back out, rather than from the matrix as it came: whatever X turned out to be, Z
    /// makes up the difference, so the angles always describe the rotation they were read from.
    /// </summary>
    private static Vector3 Written(Matrix3 rotation, double y, double x)
    {
        var sinX = Math.Sin(x);
        var cosX = Math.Cos(x);
        var z = Math.Atan2(
            (sinX * rotation.Entry(0, 2)) - (cosX * rotation.Entry(0, 1)),
            (cosX * rotation.Entry(1, 1)) - (sinX * rotation.Entry(1, 2)));
        return new Vector3(Doubles.Degrees(x), Doubles.Degrees(y), Doubles.Degrees(z));
    }

    /// <summary>Each part of <paramref name="a"/> times the same part of <paramref name="b"/>.</summary>
    public static Vector3 Times(Vector3 a, Vector3 b) => new(a.X * b.X, a.Y * b.Y, a.Z * b.Z);

    public static Vector3 Absolute(Vector3 a) => new(Math.Abs(a.X), Math.Abs(a.Y), Math.Abs(a.Z));

    private static bool SameTurn(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        for (var i = 3; i < 7; i++)
        {
            if (a[i] != b[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// <paramref name="angles"/> with whole turns added to each until it is within half a turn of the same
    /// angle in <paramref name="near"/>.
    /// </summary>
    private static Vector3 Unwrapped(Vector3 angles, Vector3 near) => new(
        Toward(angles.X, near.X),
        Toward(angles.Y, near.Y),
        Toward(angles.Z, near.Z));

    private static double Toward(double angle, double near) =>
        angle + (360 * Math.Round((near - angle) / 360, MidpointRounding.AwayFromZero));

    private static double Apart(Vector3 a, Vector3 b) =>
        Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y) + Math.Abs(a.Z - b.Z);
}
