namespace Orblit.Physics;

/// <summary>
/// What a joint holds, apart from where it is.
/// </summary>
/// <remarks>
/// Every joint holds some of the six ways its second body can move relative
/// to its first, along the three axes of its frame and about them, each
/// within a range, and a kind is only which ways and how far. The value says
/// nothing about where it goes, so one elbow serves both arms:
/// <see cref="Physics.Join"/> says where. Every axis and every measure is the
/// first body's, and turns when it does. Joined to the world, it is the world
/// that moves as that body sees it, so a body falling down a slider reads as
/// the world sliding up.
/// </remarks>
public sealed class Joint
{
    private Joint(PhysicsJointKind kind) => Kind = kind;

    public PhysicsJointKind Kind { get; }

    /// <summary>The ratio a gear, a rack or a pulley ties its two sides at.</summary>
    public double Ratio { get; private init; } = 1.0;

    /// <summary>Whether a path keeps the second body's x axis along the line.</summary>
    public bool Follow { get; private init; }

    /// <summary>
    /// The range of each of the six ways, or null where the kind leaves it
    /// alone: metres along the frame's axes, then radians about them.
    /// </summary>
    public JointLimit? AlongX { get; private init; }

    /// <inheritdoc cref="AlongX"/>
    public JointLimit? AlongY { get; private init; }

    /// <inheritdoc cref="AlongX"/>
    public JointLimit? AlongZ { get; private init; }

    /// <inheritdoc cref="AlongX"/>
    public JointLimit? AboutX { get; private init; }

    /// <inheritdoc cref="AlongX"/>
    public JointLimit? AboutY { get; private init; }

    /// <inheritdoc cref="AlongX"/>
    public JointLimit? AboutZ { get; private init; }

    /// <summary>A cone's widest swing, in radians.</summary>
    public double Swing { get; private init; }

    /// <summary>A motor's speed, in metres or radians a second.</summary>
    public double Speed { get; private init; }

    /// <summary>A motor's greatest force or torque. None is no motor.</summary>
    public double Strength { get; private init; }

    /// <summary>What a motor holds its way at, in place of a speed.</summary>
    public JointGoal? Goal { get; private init; }

    /// <summary>A spring on each way.</summary>
    public JointSprings Springs { get; private init; } = JointSprings.None;

    /// <summary>How soft the ends of the limits are.</summary>
    public JointSoftness? SoftLimit { get; private init; }

    /// <summary>The six limits, in the order the ABI names them.</summary>
    public IReadOnlyList<JointLimit?> Limits => [AlongX, AlongY, AlongZ, AboutX, AboutY, AboutZ];

    /// <summary>
    /// Holds the second body exactly where it is relative to the first:
    /// welded. Give it <paramref name="springs"/> to weld it with a give, a
    /// rubbery joint that sags under load and comes back.
    /// </summary>
    public static Joint Fixed(JointSprings? springs = null) =>
        new(PhysicsJointKind.Fixed) { Springs = springs ?? JointSprings.None };

    /// <summary>
    /// Holds the two points together and lets the bodies turn any way about
    /// them: a pendulum, or a ball and socket with no limit. Springs on the
    /// <c>Along</c> ways make the points hold with a give, and on the
    /// <c>About</c> ways pull the turn back to rest.
    /// </summary>
    public static Joint Point(JointSprings? springs = null) =>
        new(PhysicsJointKind.Point) { Springs = springs ?? JointSprings.None };

    /// <summary>
    /// Holds the points together and lets the second body turn about the
    /// frame's x axis only: a door, a wheel, a knee. The limit is the angle it
    /// may turn, in radians, within a half turn either way.
    /// </summary>
    /// <remarks>
    /// A motor turns it at <paramref name="speed"/> radians a second, never
    /// harder than <paramref name="strength"/> newton-metres. No strength is
    /// no motor, and a speed of nought with a little strength is friction in
    /// the joint. A <paramref name="goal"/> makes the motor hold an angle
    /// instead of a speed. Springs may put a spring on <c>AboutX</c>, the
    /// hinge's own way, and <paramref name="softLimit"/> makes its limit a
    /// spring about its end rather than a wall.
    /// </remarks>
    // F2: six parameters, all optional and called by name.
    public static Joint Hinge(
        JointLimit? limit = null,
        double speed = 0.0,
        double strength = 0.0,
        JointGoal? goal = null,
        JointSprings? springs = null,
        JointSoftness? softLimit = null) =>
        new(PhysicsJointKind.Hinge)
        {
            AboutX = limit,
            Speed = speed,
            Strength = strength,
            Goal = goal,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
        };

    /// <summary>
    /// Lets the second body move along the frame's x axis only, never turning:
    /// a drawer, a piston, a lift on a rail. The limit is how far, in metres.
    /// </summary>
    /// <remarks>
    /// A motor moves it at <paramref name="speed"/> metres a second, never
    /// harder than <paramref name="strength"/> newtons. A
    /// <paramref name="goal"/> makes it hold a place instead of a speed.
    /// Springs may put a spring on <c>AlongX</c>: a box hung on one settles a
    /// rest length plus gravity over omega squared below its hook.
    /// <paramref name="softLimit"/> makes the ends of the limit springs rather
    /// than walls.
    /// </remarks>
    // F2: six parameters, all optional and called by name.
    public static Joint Slider(
        JointLimit? limit = null,
        double speed = 0.0,
        double strength = 0.0,
        JointGoal? goal = null,
        JointSprings? springs = null,
        JointSoftness? softLimit = null) =>
        new(PhysicsJointKind.Slider)
        {
            AlongX = limit,
            Speed = speed,
            Strength = strength,
            Goal = goal,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
        };

    /// <summary>
    /// Keeps a point on each body a distance apart, with both free to turn.
    /// With no limit it is a rod as long as the points were apart when it was
    /// made. With one it is a range, in metres: a limit of nought to two is a
    /// two metre rope, slack until it is taut.
    /// </summary>
    /// <remarks>
    /// A spring on <c>AlongX</c> makes it elastic, pulled to its rest length,
    /// which is the length it was made at unless it says otherwise.
    /// </remarks>
    public static Joint Distance(JointLimit? limit = null, JointSprings? springs = null, JointSoftness? softLimit = null) =>
        new(PhysicsJointKind.Distance)
        {
            AlongX = limit,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
        };

    /// <summary>
    /// Holds the points together and lets the second body's x axis swing up to
    /// <paramref name="swing"/> radians from the first's, in any direction: a
    /// shoulder or a hip. <paramref name="twist"/> bounds how far it turns
    /// about that axis as well.
    /// </summary>
    /// <remarks>
    /// A spring on <c>AboutY</c> pulls the swing back towards the first body's
    /// x axis as it leaves it. <c>AboutZ</c> has no way of its own on a cone,
    /// because it swings as one. A spring on <c>AboutX</c> pulls the twist back.
    /// </remarks>
    public static Joint Cone(double swing, JointLimit? twist = null, JointSprings? springs = null, JointSoftness? softLimit = null) =>
        new(PhysicsJointKind.Cone)
        {
            AboutX = twist,
            Swing = swing,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
        };

    /// <summary>
    /// Each of the six ways set on its own: free unless limited, and locked
    /// where a limit's low is its high. For whatever the other kinds do not
    /// cover.
    /// </summary>
    /// <remarks>
    /// Turning is measured as a twist about x and then a swing, whose turns
    /// about y and z are limited separately. Within a quarter turn that is the
    /// angle it looks like. Further, the three stop being independent, which
    /// is true of any three angles.
    /// </remarks>
    // F2: eight parameters, one per way and all optional, called by name.
    public static Joint SixAxis(
        JointLimit? alongX = null,
        JointLimit? alongY = null,
        JointLimit? alongZ = null,
        JointLimit? aboutX = null,
        JointLimit? aboutY = null,
        JointLimit? aboutZ = null,
        JointSprings? springs = null,
        JointSoftness? softLimit = null) =>
        new(PhysicsJointKind.SixAxis)
        {
            AlongX = alongX,
            AlongY = alongY,
            AlongZ = alongZ,
            AboutX = aboutX,
            AboutY = aboutY,
            AboutZ = aboutZ,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
        };

    /// <summary>
    /// Ties how far the second body turns to how far the first does: it turns
    /// <paramref name="ratio"/> times as far, about its own x axis. Meshed
    /// gears turn opposite ways, so a ratio is negative for them.
    /// </summary>
    /// <remarks>
    /// Both bodies are usually hinged to something already, and the gear only
    /// makes them turn together. Turns are counted over many revolutions, which
    /// an angle read from a pose cannot do, so the ratio holds after the wheels
    /// have gone round a hundred times. A spring on <c>AlongX</c>, the only way
    /// a coupling has, makes the teeth yield under load and come back.
    /// <see cref="Physics.Join"/> takes <c>axisB</c> where the second body's
    /// axis is not the frame's own.
    /// </remarks>
    public static Joint Gear(double ratio, JointSprings? springs = null) =>
        new(PhysicsJointKind.Gear) { Ratio = ratio, Springs = springs ?? JointSprings.None };

    /// <summary>
    /// Ties how far the first body slides to how far the second turns: the
    /// first, the rack, moves <paramref name="ratio"/> metres along the frame's
    /// x for each radian the second, the pinion, turns about its own x.
    /// </summary>
    /// <remarks>
    /// The rack's direction is fixed in the world from where the joint is made,
    /// so hinge or slide the rack to what it runs on. A spring on
    /// <c>AlongX</c> makes the teeth give.
    /// </remarks>
    public static Joint Rack(double ratio, JointSprings? springs = null) =>
        new(PhysicsJointKind.Rack) { Ratio = ratio, Springs = springs ?? JointSprings.None };

    /// <summary>
    /// A rope over two fixed points. The length from the first point to the
    /// first body's point, plus <paramref name="ratio"/> times the length from
    /// the second point to the second body's, stays what it was when made. As
    /// one side goes down the other comes up, by the ratio times as much.
    /// </summary>
    /// <remarks>
    /// Both ends are given to <see cref="Physics.Join"/>: <c>at</c> on the
    /// first body, <c>to</c> on the second, and the two fixed points as
    /// <c>over</c>. It holds in both directions, so it is a rod and not a
    /// rope: it pushes as well as pulls. A spring on <c>AlongX</c> makes it
    /// stretch.
    /// </remarks>
    public static Joint Pulley(double ratio = 1.0, JointSprings? springs = null) =>
        new(PhysicsJointKind.Pulley) { Ratio = ratio, Springs = springs ?? JointSprings.None };

    /// <summary>
    /// Lets the second body slide along a bent line, a rail, a track or a
    /// chute, and holds it on it. The line is <see cref="Physics.Join"/>'s
    /// <c>path</c>, in the world as it stands when the joint is made, and it is
    /// carried with the first body.
    /// </summary>
    /// <remarks>
    /// The limit is how far along it may go, in metres from the first point. A
    /// motor takes it along at <paramref name="speed"/> metres a second, never
    /// harder than <paramref name="strength"/> newtons, and a
    /// <paramref name="goal"/> holds a place. A spring on <c>AlongX</c> pulls
    /// it to a place along the line, and <paramref name="softLimit"/> makes the
    /// ends of the limit springs. Past the ends the line goes on straight and
    /// the distance along it keeps counting, so a limit at the line's length
    /// stops what runs off the end. Without one it runs off. The second body
    /// keeps its own turn unless <paramref name="follow"/> is set, when its x
    /// axis is kept along the line as well and it is free to roll about it. The
    /// corners of the line are sharp, and a loop is closed by repeating its
    /// first point.
    /// </remarks>
    // F2: seven parameters, all optional and called by name.
    public static Joint Path(
        JointLimit? limit = null,
        double speed = 0.0,
        double strength = 0.0,
        JointGoal? goal = null,
        JointSprings? springs = null,
        JointSoftness? softLimit = null,
        bool follow = false) =>
        new(PhysicsJointKind.Path)
        {
            AlongX = limit,
            Speed = speed,
            Strength = strength,
            Goal = goal,
            Springs = springs ?? JointSprings.None,
            SoftLimit = softLimit,
            Follow = follow,
        };
}
