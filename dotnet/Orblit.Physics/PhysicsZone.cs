namespace Orblit.Physics;

/// <summary>
/// What a zone changes about the bodies inside it. A field left null is left
/// as the body has it.
/// </summary>
/// <remarks>
/// Water is a fluid, or for a pool that only slows things, a weak gravity and a
/// lot of damping. A lift shaft is a gravity that points up. The numbers are
/// checked by the world, which refuses a zone with one that is not a number.
/// </remarks>
/// <param name="Gravity">Metres per second squared, in place of the world's.</param>
/// <param name="LinearDamping">A fraction shed per second, in place of the body's own.</param>
/// <param name="AngularDamping">A fraction shed per second, in place of the body's own.</param>
/// <param name="FluidDensity">
/// Makes the zone a fluid of this density in kilograms per cubic metre, a
/// thousand for water. Its surface is the top of the trigger's bounds, level.
/// A sphere, box or capsule in it is lifted by the weight of the fluid it
/// displaces, so it floats when its own density is lower.
/// </param>
/// <param name="FluidDrag">
/// The fraction of a body's speed the fluid sheds per second when it is wholly
/// under, and less as less of it is. Ignored without a density.
/// </param>
/// <param name="Priority">
/// Where zones overlap, each field takes the answer of the zone that claims it
/// with the highest priority, and at a tie the lower body id.
/// </param>
// F2: six fields, all optional and called by name.
public sealed record PhysicsZone(
    IReadOnlyList<double>? Gravity = null,
    double? LinearDamping = null,
    double? AngularDamping = null,
    double? FluidDensity = null,
    double FluidDrag = 0.0,
    int Priority = 0);
