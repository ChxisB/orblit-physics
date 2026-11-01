namespace Orblit.Physics;

/// <summary>
/// How a body moves, apart from what shape it is. The default leaves a body as
/// it was made.
/// </summary>
/// <remarks>
/// All of it is set together by <see cref="Physics.SetControls"/>. A fixed or
/// driven body keeps its controls for when it is made free.
/// </remarks>
/// <param name="Locks">
/// The ways it may not move. A locked move drops that part of a velocity, of a
/// push and of every contact. A locked turn does the same to spin. The solver
/// holds them, so a body locked to a plane still rests on a floor that is
/// tilted across it.
/// </param>
/// <param name="GravityScale">
/// How much of the world's gravity, and of a zone's, it feels. One is
/// ordinary, zero floats and a negative number rises.
/// </param>
/// <param name="MaxSpeed">
/// The fastest it may go in metres per second. Zero is no cap. A cap is
/// applied once a step, after the contacts, so a body can pass it inside one
/// step but never leaves one above it.
/// </param>
/// <param name="MaxSpin">The fastest it may spin in radians per second. Zero is no cap.</param>
/// <param name="Centre">
/// Where its weight is, in the body's own frame. It turns about this point and
/// a push through it does not spin it. Null is the middle of the shape.
/// </param>
/// <param name="Inertia">
/// The inertia about each of its own axes through the centre of mass, in
/// kilogram square metres. Null uses the shape's, moved to the centre of mass.
/// So does a set with a zero in it, since all three are used or none. A
/// negative one is refused.
/// </param>
/// <param name="Continuous">
/// Stops it at the first fixed body or ground its move would take it through,
/// instead of finding it on the far side. Speculative contacts already hold
/// anything fast, so this is for what they miss: a small body at a walking pace
/// against a thin wall, and a body that is knocked into one in the same step.
/// It costs one cast a step while the body moves. It keeps the body's velocity
/// and leaves the stop to the contacts of the next step. Only a free body is
/// cast, and only against fixed bodies, so a driven body, a character and
/// another free body are not stopped by it. A trigger, a layer it does not
/// collide with and a pair told to ignore each other are passed as they are in
/// a step. A body already inside another is left to the contacts.
/// </param>
// F2: seven fields, all optional and called by name.
public sealed record PhysicsControls(
    PhysicsLock Locks = PhysicsLock.None,
    double GravityScale = 1.0,
    double MaxSpeed = 0.0,
    double MaxSpin = 0.0,
    IReadOnlyList<double>? Centre = null,
    IReadOnlyList<double>? Inertia = null,
    bool Continuous = false);
