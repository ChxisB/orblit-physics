using Orblit.Rig;

namespace Orblit.Physics.Rig;

/// <summary>
/// The bodies of a ragdoll and how they hang together.
/// </summary>
/// <remarks>
/// Parents are listed before the bodies that hang from them, and there is one
/// root. <see cref="Check"/> says whether an armature has the bones, in the
/// order the spec expects.
/// </remarks>
public sealed class RagdollSpec
{
    public RagdollSpec(IReadOnlyList<RagdollBody> bodies)
    {
        if (bodies.Count == 0)
        {
            throw new ArgumentException("Give at least one body.", nameof(bodies));
        }

        var seen = new HashSet<string>();
        foreach (var body in bodies)
        {
            if (body.Parent is not null && !seen.Contains(body.Parent))
            {
                throw new ArgumentException(
                    $"Body \"{body.Bone}\" hangs from \"{body.Parent}\", which is not listed before it.",
                    nameof(bodies));
            }

            if (!seen.Add(body.Bone))
            {
                throw new ArgumentException($"Two bodies end at bone \"{body.Bone}\".", nameof(bodies));
            }
        }

        if (bodies.Count(b => b.Parent is null) != 1)
        {
            throw new ArgumentException("A ragdoll has one root body.", nameof(bodies));
        }

        if (bodies[0].Parent is not null)
        {
            throw new ArgumentException("List the root body first.", nameof(bodies));
        }

        Bodies = Array.AsReadOnly(bodies.ToArray());
    }

    public IReadOnlyList<RagdollBody> Bodies { get; }

    /// <summary>
    /// Throws an <see cref="ArmatureException"/> if a bone is missing, a body's
    /// <see cref="RagdollBody.From"/> is not above its bone, or a body hangs
    /// from one with no bone above its own.
    /// </summary>
    public void Check(Armature armature)
    {
        var covered = new Dictionary<string, string>();
        foreach (var body in Bodies)
        {
            foreach (var name in ChainOf(armature, body))
            {
                if (covered.TryGetValue(name, out var other))
                {
                    throw new ArmatureException(
                        $"Bone \"{name}\" is covered by the bodies ending at \"{other}\" and \"{body.Bone}\".");
                }

                covered[name] = body.Bone;
            }

            CheckParent(armature, body);
        }
    }

    /// <summary>The bones a body covers, from its <see cref="RagdollBody.From"/> down to its bone.</summary>
    public static List<string> ChainOf(Armature armature, RagdollBody body)
    {
        foreach (var name in new[] { body.From, body.Bone })
        {
            if (!armature.Contains(name))
            {
                throw new ArmatureException(
                    $"No bone called \"{name}\" in this armature, so the ragdoll has no body for it.");
            }
        }

        var path = new List<string>();
        var bone = body.Bone;
        while (bone is not null)
        {
            path.Add(bone);
            if (bone == body.From)
            {
                path.Reverse();
                return path;
            }

            bone = armature[bone]?.Parent;
        }

        throw new ArmatureException(
            $"Bone \"{body.From}\" is not above \"{body.Bone}\", so no body can run from one to the other.");
    }

    private void CheckParent(Armature armature, RagdollBody body)
    {
        if (body.Parent is null)
        {
            return;
        }

        // A body may hang from any bone the parent body covers, which is how a
        // hip hangs from the middle of a torso that runs from hips to spine.
        var held = ChainOf(armature, Bodies.First(b => b.Bone == body.Parent));
        var name = armature[body.From]?.Parent;
        while (name is not null)
        {
            if (held.Contains(name))
            {
                return;
            }

            name = armature[name]?.Parent;
        }

        throw new ArmatureException(
            $"Body \"{body.Bone}\" hangs from \"{body.Parent}\", which has no bone above \"{body.From}\" in this armature.");
    }
}
