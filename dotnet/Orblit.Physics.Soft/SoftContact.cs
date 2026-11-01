using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>Where a swept ball met the world: a point on the surface and the way out of it.</summary>
public sealed record SoftContact(Vector3 At, Vector3 Normal);
