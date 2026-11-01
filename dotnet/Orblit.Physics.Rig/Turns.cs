using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>Turns measured and mixed the way the solver measures them.</summary>
internal static class Turns
{
    /// <summary>
    /// How a joint measures the turn of its second body, as the solver does.
    /// </summary>
    /// <remarks>
    /// <paramref name="turn"/> is the second body's frame seen from the first's.
    /// It is split into a twist about x and then a swing that tips x over, the
    /// shorter way round, and answered as the three numbers
    /// <see cref="Physics.JointStateOf"/> reads back: the twist, then the swing's
    /// turn about y and about z, in radians.
    /// A pose's angles have to be measured exactly as the joint measures them,
    /// because they become the places the joint's springs pull to. A different
    /// split is right for small turns and wrong for large ones.
    /// </remarks>
    public static double[] Angles(Quaternion turn)
    {
        var x = turn.X;
        var y = turn.Y;
        var z = turn.Z;
        var w = turn.W;
        if (w < 0)
        {
            x = -x;
            y = -y;
            z = -z;
            w = -w;
        }

        var twist = 2 * Math.Atan2(x, w);

        // With x tipped all the way over there is no twist to take out, and what
        // is left is all swing.
        var size = Math.Sqrt((x * x) + (w * w));
        var about = size > 1e-9
            ? new Quaternion(-x / size, 0, 0, w / size)
            : Quaternion.Identity();
        var swing = new Quaternion(x, y, z, w) * about;

        var sine = Math.Sqrt((swing.Y * swing.Y) + (swing.Z * swing.Z));
        var swung = 2 * Math.Atan2(sine, swing.W);
        if (sine <= 1e-9)
        {
            return [twist, 2 * swing.Y, 2 * swing.Z];
        }

        return [twist, swing.Y * swung / sine, swing.Z * swung / sine];
    }

    /// <summary>
    /// The turn from <paramref name="before"/> to <paramref name="after"/> as a
    /// vector along its axis, whose length is the angle in radians, the shorter
    /// way round.
    /// </summary>
    public static Vector3 Between(Quaternion before, Quaternion after)
    {
        var turn = after * before.Conjugated();
        if (turn.W < 0)
        {
            turn.SetValues(-turn.X, -turn.Y, -turn.Z, -turn.W);
        }

        var part = new Vector3(turn.X, turn.Y, turn.Z);
        var sine = part.Length;
        if (sine < 1e-9)
        {
            return part * 2;
        }

        return part * (2 * Math.Atan2(sine, turn.W) / sine);
    }

    /// <summary><paramref name="from"/> taken <paramref name="amount"/> of the way to <paramref name="to"/>, by the shorter arc.</summary>
    public static Quaternion Slerp(Quaternion from, Quaternion to, double amount)
    {
        var dot = (from.X * to.X) + (from.Y * to.Y) + (from.Z * to.Z) + (from.W * to.W);
        var sign = dot < 0 ? -1.0 : 1.0;
        dot *= sign;
        if (dot > 0.9995)
        {
            return new Quaternion(
                from.X + (((sign * to.X) - from.X) * amount),
                from.Y + (((sign * to.Y) - from.Y) * amount),
                from.Z + (((sign * to.Z) - from.Z) * amount),
                from.W + (((sign * to.W) - from.W) * amount)).Normalized();
        }

        var angle = Math.Acos(Doubles.Clamp(dot, -1.0, 1.0));
        var sine = Math.Sin(angle);
        var a = Math.Sin((1 - amount) * angle) / sine;
        var b = Math.Sin(amount * angle) / sine * sign;
        return new Quaternion(
            (from.X * a) + (to.X * b),
            (from.Y * a) + (to.Y * b),
            (from.Z * a) + (to.Z * b),
            (from.W * a) + (to.W * b)).Normalized();
    }
}
