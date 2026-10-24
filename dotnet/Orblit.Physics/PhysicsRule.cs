namespace Orblit.Physics;

/// <summary>
/// What is different about the contact between two particular bodies, in place
/// of what their own friction, restitution and mass would say. A rule that
/// changes nothing is a removal, so at least one field is given.
/// </summary>
public sealed class PhysicsRule
{
    // F2: six fields, all optional and called by name.
    public PhysicsRule(
        double? friction = null,
        double? restitution = null,
        double moveScaleA = 1.0,
        double moveScaleB = 1.0,
        bool ignore = false,
        IReadOnlyList<double>? oneWay = null)
    {
        if (friction is null && restitution is null && moveScaleA == 1.0 && moveScaleB == 1.0 && !ignore && oneWay is null)
        {
            throw new ArgumentException("A rule that changes nothing is not a rule.", nameof(friction));
        }

        Friction = friction;
        Restitution = restitution;
        MoveScaleA = moveScaleA;
        MoveScaleB = moveScaleB;
        Ignore = ignore;
        OneWay = oneWay;
    }

    /// <summary>Replaces the pair's friction outright.</summary>
    public double? Friction { get; }

    /// <summary>Replaces the pair's restitution outright.</summary>
    public double? Restitution { get; }

    /// <summary>
    /// How much of this contact's push the first body takes, in this contact
    /// and no other. One leaves a body as it is, zero makes it immovable to the
    /// other, and two makes it move as if it were half the mass. This is how a
    /// platform carries a crate without the crate slowing it.
    /// </summary>
    public double MoveScaleA { get; }

    /// <summary>How much of this contact's push the second body takes. See <see cref="MoveScaleA"/>.</summary>
    public double MoveScaleB { get; }

    /// <summary>
    /// The two bodies pass through each other. There is no contact and no touch
    /// event, and a trigger does not see the other one. The other fields are
    /// not read. A character walks through a body it is told to ignore.
    /// </summary>
    public bool Ignore { get; }

    /// <summary>
    /// Makes the first body a one-way platform and the second what passes
    /// through it, going up along this direction, three numbers in the
    /// world's axes. The platform holds the second body only while it is on
    /// that side and not rising through it, so a body thrown up from below
    /// passes, and the same body coming down, or standing on it, does not. The
    /// rule is made for the two bodies named, so a platform that lets many
    /// bodies through has one for each. A character named second jumps up
    /// through it and lands on it, and reads only this and
    /// <see cref="Ignore"/>.
    /// </summary>
    public IReadOnlyList<double>? OneWay { get; }
}
