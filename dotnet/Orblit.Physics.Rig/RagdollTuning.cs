namespace Orblit.Physics.Rig;

/// <summary>
/// How hard a ragdoll holds and how it settles.
/// </summary>
/// <param name="PowerHertz">How quickly a powered joint pulls to the pose, in cycles a second.</param>
/// <param name="PowerRatio">Damping of that pull. One settles as fast as it can without overshoot.</param>
/// <param name="LinearDamping">How fast a body loses speed.</param>
/// <param name="AngularDamping">How fast a body loses spin.</param>
/// <param name="SnapDistance">
/// A driven body further than this from where the pose puts it, in the
/// model's units, is put there rather than carried.
/// </param>
// F2: five numbers, all optional and set by name.
public sealed record RagdollTuning(
    double PowerHertz = 8,
    double PowerRatio = 1,
    double LinearDamping = 0.1,
    double AngularDamping = 0.5,
    double SnapDistance = 2);
