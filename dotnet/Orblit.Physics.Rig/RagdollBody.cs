namespace Orblit.Physics.Rig;

/// <summary>
/// One rigid body of a ragdoll, standing for a bone or a short run of them.
/// </summary>
/// <remarks>
/// The body is a capsule from the head of <see cref="From"/> to the tail of
/// <see cref="Bone"/>, so a forearm is one bone and a spine of three bones is
/// one body with <see cref="From"/> the lowest and <see cref="Bone"/> the
/// highest. Bones between them, and every bone below <see cref="Bone"/> that
/// has no body of its own, keep their animated pose against the body: that is
/// the map from a coarse physical skeleton to the full one.
/// The joint sits at the head of <see cref="From"/> and joins this body to
/// <see cref="Parent"/>, which is another body's <see cref="Bone"/> name. A
/// root has neither a parent nor a joint.
/// </remarks>
public sealed class RagdollBody
{
    // F2: seven parameters, all but the first two optional and called by name.
    public RagdollBody(
        string bone,
        double radius,
        string? @from = null,
        string? parent = null,
        RagdollJoint? joint = null,
        double mass = 1.0,
        double friction = 0.5)
    {
        if (!double.IsFinite(radius) || radius <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "A radius is above 0.");
        }

        if (!double.IsFinite(mass) || mass <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(mass), mass, "A mass is above 0.");
        }

        if ((parent is null) != (joint is null))
        {
            throw new ArgumentException($"Body \"{bone}\" needs both a parent and a joint, or neither.", nameof(joint));
        }

        Bone = bone;
        From = @from ?? bone;
        Parent = parent;
        Joint = joint;
        Radius = radius;
        Mass = mass;
        Friction = friction;
    }

    /// <summary>The last bone the body covers, whose tail is the body's far end.</summary>
    public string Bone { get; }

    /// <summary>The first bone it covers, whose head is where its joint sits.</summary>
    public string From { get; }

    /// <summary>The <see cref="Bone"/> of the body this one hangs from, or null for the root.</summary>
    public string? Parent { get; }

    public RagdollJoint? Joint { get; }

    /// <summary>How thick the capsule is, in the model's units.</summary>
    public double Radius { get; }

    public double Mass { get; }

    public double Friction { get; }
}
