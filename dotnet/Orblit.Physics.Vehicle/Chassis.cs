using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// Where the chassis stands, and how it moves, at the start of a step.
/// </summary>
/// <param name="Origin">Where the body is placed.</param>
/// <param name="Turn">How the body is turned.</param>
/// <param name="Rotation"><paramref name="Turn"/> as a matrix.</param>
/// <param name="Centre">Where its weight is.</param>
/// <param name="Velocity">How the point of its weight moves.</param>
/// <param name="Spin">How it turns.</param>
// F2: six fields, all read at once from the world.
internal sealed record Chassis(
    Vector3 Origin,
    Quaternion Turn,
    Matrix3 Rotation,
    Vector3 Centre,
    Vector3 Velocity,
    Vector3 Spin)
{
    /// <summary>Its own down, in the world.</summary>
    public Vector3 Down => Direct(new Vector3(0, -1, 0));

    /// <summary>Null when the body is gone.</summary>
    public static Chassis? Read(Physics physics, ulong id, IReadOnlyList<double> centre)
    {
        var t = physics.TransformOf(id);
        var v = physics.VelocityOf(id);
        if (t == null || v == null)
        {
            return null;
        }

        // AsRotationMatrix, not a rotation of the vector, which turns the other way.
        var origin = new Vector3(t[0], t[1], t[2]);
        var turn = new Quaternion(t[3], t[4], t[5], t[6]);
        var rotation = turn.AsRotationMatrix();
        return new Chassis(
            origin,
            turn,
            rotation,
            origin + rotation.Transformed(new Vector3(centre[0], centre[1], centre[2])),
            new Vector3(v[0], v[1], v[2]),
            new Vector3(v[3], v[4], v[5]));
    }

    public Vector3 Direct(Vector3 local) => Rotation.Transformed(local);

    public Vector3 ToWorld(Vector3 local) => Origin + Direct(local);

    /// <summary>
    /// How fast the chassis moves at <paramref name="point"/> in the world, taken
    /// from where that point of the chassis was at the <paramref name="before"/>
    /// update, <paramref name="dt"/> seconds ago.
    /// </summary>
    /// <remarks>
    /// Not from the solver's velocity, which is read at the end of its
    /// sub-steps. For a body held up by one push a step that reading is a
    /// sawtooth about the motion, and a tyre that holds the sawtooth still
    /// lets the car creep. The first update has no past and takes the reading.
    /// </remarks>
    public Vector3 MotionAt(Vector3 point, Chassis? before, double dt)
    {
        if (before == null)
        {
            return Velocity + Spin.Cross(point - Centre);
        }

        // Fresh matrices, since the product and the transpose work in place.
        var turned = before.Turn.AsRotationMatrix();
        var now = Turn.AsRotationMatrix();
        now.Transpose();
        turned.Multiply(now);
        var earlier = before.Origin + turned.Transformed(point - Origin);
        return (point - earlier) * (1 / dt);
    }
}
