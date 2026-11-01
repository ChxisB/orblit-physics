using Orblit.Scene;

namespace Orblit.Physics.Scene;

internal static class CompoundParts
{
    /// <summary>Reads only a part's geometry. Body dynamics belong to its owning entity.</summary>
    public static ShapePart PartOf(BodyPart part, Func<IReadOnlyList<double>, ulong> hullOf)
    {
        var radius = part.Radius;
        var straight = Math.Max(0.0, (part.Height / 2) - radius);
        var shape = part.Shape switch
        {
            BodyShape.Box => Shape.Box(part.Size.X / 2, part.Size.Y / 2, part.Size.Z / 2),
            BodyShape.Sphere => Shape.Sphere(radius),
            BodyShape.Capsule => Shape.Capsule(radius, straight),
            BodyShape.Cylinder => Shape.Cylinder(radius, part.Height / 2),
            BodyShape.Hull => Shape.Hull(hullOf(part.Hull)),
            _ => throw new ArgumentException("A part must be convex.", nameof(part)),
        };
        return new ShapePart(
            shape,
            [part.Centre.X, part.Centre.Y, part.Centre.Z],
            [part.Rotation.X, part.Rotation.Y, part.Rotation.Z, part.Rotation.W],
            [part.Scale.X, part.Scale.Y, part.Scale.Z]);
    }
}
