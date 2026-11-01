using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>
/// The bodies of a <see cref="Physics"/> world, as a soft body meets them.
/// </summary>
/// <remarks>
/// A world cannot cast a plane, so a floor a cloth lands on must be a box. A
/// trigger is never met. Step the world before the soft body, so the cast sees
/// where the bodies are now.
/// </remarks>
public sealed class PhysicsSurface(Physics physics, Layers? layers = null, ulong ignore = 0) : ISoftSurface
{
    public Physics Physics { get; } = physics;

    /// <summary>What the cloth collides with.</summary>
    public Layers Layers { get; } = layers ?? Layers.Everything;

    /// <summary>A body the cloth passes through, such as the one it hangs from.</summary>
    public ulong Ignore { get; } = ignore;

    public SoftContact? Sweep(Vector3 from, Vector3 direction, double distance, double radius)
    {
        var hit = Physics.Cast(
            [from.X, from.Y, from.Z],
            [direction.X, direction.Y, direction.Z],
            distance,
            shape: Shape.Sphere(radius),
            layers: Layers,
            ignore: Ignore);
        return hit is null ? null : new SoftContact(Packed.Read(hit.At, 0), Packed.Read(hit.Normal, 0));
    }
}
