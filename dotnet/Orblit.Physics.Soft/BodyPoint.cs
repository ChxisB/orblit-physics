using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>A place on a physics body, as the point a soft body is attached to.</summary>
public static class BodyPoint
{
    /// <summary>
    /// The point <paramref name="local"/> on physics body <paramref name="id"/>,
    /// in the world, for the <c>where</c> of <see cref="SoftBody.Attach"/>. It
    /// turns with the body.
    /// </summary>
    /// <remarks>
    /// The point throws once the body is gone, so detach the cloth before
    /// removing it.
    /// </remarks>
    public static Func<Vector3> Of(Physics physics, ulong id, Vector3 local)
    {
        var offset = local.Clone();
        return () =>
        {
            var pose = physics.TransformOf(id) ?? throw new InvalidOperationException($"Body {id} is not in the world.");
            // AsRotationMatrix, not a rotated copy of the vector, which turns the other way.
            var turn = new Quaternion(pose[3], pose[4], pose[5], pose[6]).AsRotationMatrix();
            var point = turn.Transformed(offset);
            point.Add(new Vector3(pose[0], pose[1], pose[2]));
            return point;
        };
    }
}
