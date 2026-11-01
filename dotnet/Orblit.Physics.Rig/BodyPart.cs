using Orblit.Rig;
using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// One body of a ragdoll with everything worked out from the rest pose, so
/// that nothing is measured from the armature again after it is built.
/// </summary>
internal sealed class BodyPart
{
    private BodyPart()
    {
    }

    public required RagdollBody Body { get; init; }

    public required int Index { get; init; }

    /// <summary>The body's name in the physics world, which is also its joint's.</summary>
    public required ulong Id { get; init; }

    /// <summary>The part this one hangs from, or null for the root.</summary>
    public required int? Parent { get; init; }

    public List<int> Children { get; } = [];

    /// <summary>A capsule in the world's units.</summary>
    public required Shape Shape { get; init; }

    /// <summary>The body's centre and turn as seen from the head of <see cref="RagdollBody.From"/>.</summary>
    public required Frame Offset { get; init; }

    /// <summary>The first bone's frame at rest, in the model's space.</summary>
    public required Frame RestBone { get; init; }

    /// <summary>The body's frame at rest, in the model's space.</summary>
    public required Frame RestBody { get; init; }

    /// <summary>
    /// The turn from the joint's frame to the first bone's own, which puts the
    /// joint's x axis along the bone for a ball and along the hinge for a hinge.
    /// </summary>
    public required Quaternion AxisTurn { get; init; }

    /// <summary>The body's rest frame in its parent body's, lengths in world units.</summary>
    public required Frame RestInParent { get; init; }

    /// <summary>The joint's frame in its parent body's, lengths in world units.</summary>
    public required Frame JointInParent { get; init; }

    public RagdollMode Mode { get; set; } = RagdollMode.Animated;

    /// <summary>Whether the joint now standing has springs on it.</summary>
    public bool Sprung { get; set; }

    /// <summary>The time on the ragdoll's clock this part goes limp, or null.</summary>
    public double? LimpAt { get; set; }

    public string Bone => Body.Bone;

    /// <summary>The turn of the joint's frame at rest in the model's space.</summary>
    public Quaternion RestFrame => RestBone.Rotation * AxisTurn;

    /// <summary>
    /// Works a body out of <paramref name="armature"/>'s rest pose. Lengths are
    /// in the world's units, scaled by <paramref name="scale"/>, and
    /// <paramref name="parent"/> is the part this one hangs from.
    /// </summary>
    // F2: six parameters, built in one place.
    public static BodyPart Of(
        Armature armature,
        RagdollBody body,
        int index,
        ulong id,
        BodyPart? parent,
        double scale)
    {
        var restBone = Frame.FromMatrix(armature.RestOf(body.From));
        // The spec was checked against the armature, so the bone is there.
        var last = armature[body.Bone]!;
        var tail = armature.RestOf(body.Bone).Transformed3(new Vector3(0, last.Length, 0));
        var span = tail - restBone.Position;
        var length = span.Length;

        // The capsule stands along the line from head to far tail, which is the
        // bone's own Y when the run of bones is straight and the nearest to it
        // when it is not.
        var back = new Frame(Vector3.Zero(), restBone.Rotation.Conjugated());
        var along = length < 1e-9 ? new Vector3(0, 1, 0) : back.Turn(span / length);
        var centre = back.Turn(span * 0.5);
        var offset = new Frame(centre, Ik.RotationBetween(new Vector3(0, 1, 0), along));
        var restBody = restBone.Then(offset);

        var twist = body.Joint is RagdollHinge hinge ? hinge.Axis : new Vector3(0, 1, 0);
        var axisTurn = Ik.RotationBetween(new Vector3(1, 0, 0), twist);
        var frame = new Frame(restBone.Position, restBone.Rotation * axisTurn);

        return new BodyPart
        {
            Body = body,
            Index = index,
            Id = id,
            Parent = parent?.Index,
            Shape = Shape.Capsule(
                body.Radius * scale,
                Math.Max(0.0, (length / 2) - body.Radius) * scale),
            Offset = offset,
            RestBone = restBone,
            RestBody = restBody,
            AxisTurn = axisTurn,
            RestInParent = parent is null
                ? restBody
                : restBody.Within(parent.RestBody).Scaled(scale),
            JointInParent = parent is null
                ? frame
                : frame.Within(parent.RestBody).Scaled(scale),
        };
    }

    /// <summary>
    /// Where each way of the joint is pulled to for <paramref name="turn"/>,
    /// the second body's frame seen from the first's, kept inside what the
    /// joint allows.
    /// </summary>
    /// <remarks>
    /// Six places in the order the joint's ways are named. A null leaves a way
    /// where it is, which is every way a pose has no say in.
    /// </remarks>
    public double?[] RestsFor(Quaternion turn)
    {
        var angles = Turns.Angles(turn);
        return Body.Joint switch
        {
            RagdollHinge hinge => [null, null, null, Within(angles[0], hinge.Low, hinge.High), null, null],
            RagdollBall ball =>
            [
                null,
                null,
                null,
                Within(angles[0], ball.TwistLow, ball.TwistHigh),
                Within(angles[1], -ball.Swing, ball.Swing),
                Within(angles[2], -ball.Swing, ball.Swing),
            ],
            _ => new double?[6],
        };
    }

    private static double Within(double value, double low, double high) =>
        Math.Min(high, Math.Max(low, value));
}
