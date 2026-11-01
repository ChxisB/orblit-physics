using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// The ground found under one wheel.
/// </summary>
/// <param name="Body">The body the cast met.</param>
/// <param name="Compression">Metres squeezed.</param>
/// <param name="Rate">How fast it is being squeezed.</param>
/// <param name="At">Where the wheel touches, in the world.</param>
/// <param name="Normal">Away from the ground.</param>
// F2: five fields, all required.
internal sealed record Probe(ulong Body, double Compression, double Rate, IReadOnlyList<double> At, Vector3 Normal);
