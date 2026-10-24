using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// Where something is and which way it faces, with no size.
/// </summary>
/// <remarks>
/// A body's place in the world and a bone's place in the model are both this,
/// and the ragdoll's work is carrying one between the two.
/// </remarks>
internal sealed class Frame(Vector3 position, Quaternion rotation)
{
    public Vector3 Position { get; } = position.Clone();

    public Quaternion Rotation { get; } = rotation.Clone();

    /// <summary>The frame a matrix puts things in, with its scale taken out.</summary>
    public static Frame FromMatrix(Matrix4 matrix)
    {
        var position = Vector3.Zero();
        var rotation = Quaternion.Identity();
        matrix.Decompose(position, rotation, Vector3.Zero());
        rotation.Normalize();
        return new Frame(position, rotation);
    }

    /// <summary>
    /// <paramref name="vector"/> turned by this frame's rotation. By hand,
    /// because <c>Quaternion.Rotated</c> turns by the inverse of the rotation
    /// it is given.
    /// </summary>
    public Vector3 Turn(Vector3 vector)
    {
        var axis = new Vector3(Rotation.X, Rotation.Y, Rotation.Z);
        var across = axis.Cross(vector) * 2;
        return vector + (across * Rotation.W) + axis.Cross(across);
    }

    /// <summary><paramref name="local"/>, which is given in this frame, as a frame of the outer space.</summary>
    public Frame Then(Frame local) => new(
        Position + Turn(local.Position),
        (Rotation * local.Rotation).Normalized());

    /// <summary>The same frame with its position <paramref name="factor"/> times as far from the origin.</summary>
    public Frame Scaled(double factor) => new(Position * factor, Rotation);

    /// <summary>This frame's place as seen from <paramref name="outer"/>.</summary>
    public Frame Within(Frame outer)
    {
        var back = new Frame(Vector3.Zero(), outer.Rotation.Conjugated());
        return new Frame(
            back.Turn(Position - outer.Position),
            (back.Rotation * Rotation).Normalized());
    }
}

/// <summary>
/// The model's place in the world, which is what a rig's bones are measured
/// from.
/// </summary>
internal sealed class ModelSpace(ModifierFrame frame)
{
    /// <summary>The turn of the model's axes in the world's.</summary>
    public Quaternion Rotation { get; } = Frame.FromMatrix(frame.ModelToWorld).Rotation;

    /// <summary>How much longer a length is in the world than in the model.</summary>
    public double Scale => frame.Scale;

    public Frame ToWorld(Frame model) => new(
        frame.ToWorld(model.Position),
        (Rotation * model.Rotation).Normalized());

    public Frame ToModel(Frame world) => new(
        frame.ToModel(world.Position),
        (Rotation.Conjugated() * world.Rotation).Normalized());
}

/// <summary>The lists the physics world takes for a vector or a rotation.</summary>
internal static class Storage
{
    public static double[] Of(Vector3 vector) => [vector.X, vector.Y, vector.Z];

    public static double[] Of(Quaternion rotation) => [rotation.X, rotation.Y, rotation.Z, rotation.W];
}
