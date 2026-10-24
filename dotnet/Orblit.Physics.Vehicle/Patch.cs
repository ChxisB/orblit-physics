using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// The ground under one wheel for one step.
/// </summary>
/// <param name="Velocity">How fast the chassis moves at the point the wheel touches.</param>
/// <param name="Normal">Unit length, away from the ground.</param>
/// <param name="Heading">Unit length, the way the wheel rolls. It need not lie in the ground.</param>
/// <param name="Load">Newtons the wheel presses on the ground with.</param>
/// <param name="Surface">The grip of what it stands on. One is dry ground.</param>
/// <param name="Mass">Kilograms of the car that the wheel carries.</param>
// F2: six fields, all required.
internal sealed record Patch(
    Vector3 Velocity,
    Vector3 Normal,
    Vector3 Heading,
    double Load,
    double Surface,
    double Mass);
