using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// Runs a <see cref="Ragdoll"/> inside a <see cref="ModifierStack"/>.
/// </summary>
/// <remarks>
/// Put it after the modifiers that shape the animated pose and before the ones
/// that should follow what the ragdoll does, such as spring bones. Each frame it
/// sends the animated pose to the bodies, then writes the bones of every free
/// body from where the bodies are.
/// Step the physics world once a frame with the same <c>dt</c> the modifier frame
/// carries. The bones written are where the last step left the bodies, and the
/// animated bodies are given the velocity that reaches this pose in the step
/// that follows.
/// When a character stands up the bones jump from where the ragdoll lay to where
/// the get-up animation starts. <see cref="Recover"/> hides that: for the seconds
/// it is given, the output is mixed from the last frame the ragdoll was drawn.
/// </remarks>
public sealed class RagdollPose(Ragdoll ragdoll) : Modifier
{
    /// <summary>Every bone's world matrix as of the last frame a body was free.</summary>
    private Dictionary<string, Matrix4> _last = [];

    /// <summary>The matrices a recovery mixes away from.</summary>
    private Dictionary<string, Matrix4> _from = [];
    private double _total;
    private double _left;

    public Ragdoll Ragdoll { get; } = ragdoll;

    /// <summary>Whether the output is still being mixed from a fallen pose.</summary>
    public bool Recovering => _left > 0;

    /// <summary>
    /// Mixes the output from the last frame the ragdoll was drawn to the animated
    /// pose over <paramref name="seconds"/>, easing in and out.
    /// </summary>
    /// <remarks>
    /// Call it in the frame the bodies are made animated again, before the next
    /// solve. With nothing drawn yet, or no time, it does nothing.
    /// </remarks>
    public void Recover(double seconds)
    {
        if (seconds <= 0 || _last.Count == 0)
        {
            _left = 0;
            return;
        }

        _from = _last;
        _total = seconds;
        _left = seconds;
    }

    public override void Check(Armature armature) => Ragdoll.Spec.Check(armature);

    public override void Solve(Pose pose, ModifierFrame frame)
    {
        Ragdoll.FollowPose(pose, frame);
        Ragdoll.WritePose(pose, frame);
        MixFromFallen(pose, frame);
        Remember(pose, frame);
    }

    public override void Reset()
    {
        _last = [];
        _left = 0;
    }

    private void MixFromFallen(Pose pose, ModifierFrame frame)
    {
        if (_left <= 0)
        {
            return;
        }

        _left = Math.Max(0, _left - frame.Dt);
        var progress = 1 - (_left / _total);
        var amount = progress * progress * (3 - (2 * progress));
        var fallen = FallenPose(pose.Armature, frame);
        foreach (var bone in pose.Armature.Bones)
        {
            var before = fallen[bone.Name];
            var now = pose[bone.Name];
            now.Location.SetFrom(before.Location + ((now.Location - before.Location) * amount));
            now.Rotation = Turns.Slerp(before.Rotation, now.Rotation, amount);
            now.Scale.SetFrom(before.Scale + ((now.Scale - before.Scale) * amount));
        }

        pose.Refresh();
    }

    /// <summary>
    /// The remembered world matrices as a pose of their own, in the model's space
    /// as it stands now, so a model that has moved is mixed from where the bones
    /// really were.
    /// </summary>
    private Pose FallenPose(Armature armature, ModifierFrame frame)
    {
        var fallen = new Pose(armature);
        fallen.Evaluate();
        foreach (var name in armature.EvaluationOrder)
        {
            if (_from.TryGetValue(name, out var world))
            {
                BoneEdit.PlaceBone(fallen, name, frame.WorldToModel.Multiplied(world));
            }
        }

        return fallen;
    }

    private void Remember(Pose pose, ModifierFrame frame)
    {
        if (!Ragdoll.HasFree)
        {
            return;
        }

        _last = pose.Armature.Bones.ToDictionary(
            bone => bone.Name,
            bone => frame.ModelToWorld.Multiplied(pose.WorldOf(bone.Name)));
    }
}
