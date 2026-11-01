namespace Orblit.Physics;

/// <summary>How a joint stands, and how hard it held on the last step.</summary>
/// <param name="Offset">
/// Where the second body's point is from the first's, along the first's axes,
/// in metres. For a distance joint, the first is the distance.
/// </param>
/// <param name="Angles">
/// How far the second body has turned from the first, in radians: the twist
/// about x, then the swing's turn about y and about z.
/// </param>
/// <param name="Force">
/// The force it held with on the last step, in newtons. The number to read
/// before choosing where it breaks.
/// </param>
/// <param name="Torque">The torque it held with, in newton-metres.</param>
public sealed record JointState(
    IReadOnlyList<double> Offset,
    IReadOnlyList<double> Angles,
    double Force,
    double Torque);
