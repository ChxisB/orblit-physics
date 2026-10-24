namespace Orblit.Physics;

/// <summary>
/// How much a world holds, and how much its last step looked at. Bodies is
/// every body, triggers and ground included, and the static, kinematic and
/// dynamic counts add up to it. Kinematic is driven bodies and characters,
/// which the solver does not push. Asleep, Triggers and Characters are counted
/// within Bodies, and are each other's neighbours rather than parts of one
/// another. Pairs is the pairs of bodies the last step's narrowphase looked at,
/// Touching how many of those touched, and Points the contact points they
/// made. StepMicroseconds is how long the last <see cref="Physics.Step"/> took
/// in the engine, zero before the first. It is about the machine, so a
/// snapshot does not carry it.
/// </summary>
// F2: fourteen counts, all required and built in one place from the native stats.
public sealed record PhysicsStats(
    int Bodies,
    int StaticBodies,
    int KinematicBodies,
    int DynamicBodies,
    int Asleep,
    int Triggers,
    int Characters,
    int Joints,
    int Zones,
    int Rules,
    int Pairs,
    int Touching,
    int Points,
    double StepMicroseconds);
