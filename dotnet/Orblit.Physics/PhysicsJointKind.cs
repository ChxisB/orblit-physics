namespace Orblit.Physics;

/// <summary>
/// Which kind of joint, as the ABI numbers them. Named for the package, as
/// <see cref="PhysicsMotion"/> is, because a scene document has a joint kind
/// of its own and a game imports both.
/// </summary>
public enum PhysicsJointKind : uint
{
    Fixed = 1,
    Point = 2,
    Hinge = 3,
    Slider = 4,
    Distance = 5,
    Cone = 6,
    SixAxis = 7,
    Gear = 8,
    Rack = 9,
    Pulley = 10,
    Path = 11,
}
