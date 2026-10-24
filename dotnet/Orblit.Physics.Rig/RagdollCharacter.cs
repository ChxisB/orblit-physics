using Orblit.VectorMath;

namespace Orblit.Physics.Rig;

/// <summary>
/// A walking character that can be knocked into its ragdoll and got up again.
/// </summary>
/// <remarks>
/// While it stands the character is a capsule the physics world moves by
/// sweeping, and the ragdoll is animated, carried along with the pose.
/// <see cref="Knock"/> takes the capsule out and lets the bodies go limp from the
/// bone that was hit. <see cref="StandUp"/> puts the capsule down where the
/// character lies and makes the bodies animated again.
/// The ragdoll has to be built with the capsule's layers in <c>ignore</c>, so its
/// bodies take no notice of the capsule. The capsule is made here with the
/// ragdoll's layer taken out of what it cares about.
/// </remarks>
public sealed class RagdollCharacter : IDisposable
{
    private bool _standing = true;

    // F2: eleven parameters, the first three required and the rest called by name.
    public RagdollCharacter(
        Ragdoll ragdoll,
        ulong id,
        Vector3 at,
        RagdollPose? pose = null,
        Shape? shape = null,
        Layers? layers = null,
        double stepHeight = 0.3,
        double steepest = Math.PI / 4,
        double skin = 0.01,
        double strength = 500.0,
        double friction = 0.5)
    {
        Ragdoll = ragdoll;
        Id = id;
        Pose = pose;
        Shape = shape ?? Shape.Capsule(0.3, 0.6);
        Layers = layers ?? Layers.Everything;
        StepHeight = stepHeight;
        Steepest = steepest;
        Skin = skin;
        Strength = strength;
        Friction = friction;
        if ((ragdoll.Layers.Cares & Layers.Is) != 0)
        {
            throw new ArgumentException(
                "The ragdoll still cares about the character's layer, so its bodies "
                + $"would push the capsule. Build it with ignore: {Layers.Is}.",
                nameof(layers));
        }

        Add(at);
    }

    public Ragdoll Ragdoll { get; }

    /// <summary>Where the output is mixed back from a fall, or null to cut straight to the animated pose.</summary>
    public RagdollPose? Pose { get; }

    /// <summary>The capsule's id in the physics world.</summary>
    public ulong Id { get; }

    public Shape Shape { get; }

    public Layers Layers { get; }

    public double StepHeight { get; }

    public double Steepest { get; }

    public double Skin { get; }

    public double Strength { get; }

    public double Friction { get; }

    /// <summary>Whether the character has been knocked down and not yet got up.</summary>
    public bool IsDown => !_standing;

    private Physics World => Ragdoll.Physics;

    /// <summary>
    /// Knocks the character down: the capsule goes, and <paramref name="bone"/>'s
    /// body goes limp with <paramref name="impulse"/> and the rest after it,
    /// <paramref name="spread"/> seconds a joint. Hitting a character that is
    /// already down only hits the bodies again.
    /// </summary>
    public void Knock(string bone, Vector3 impulse, Vector3? at = null, double spread = 0.04)
    {
        if (_standing)
        {
            World.Remove(Id);
            _standing = false;
        }

        Ragdoll.Hit(bone, impulse, at, spread);
    }

    /// <summary>
    /// Gets the character up under the ragdoll's root, and answers the point on
    /// the ground it stands at. The model is the game's to put there.
    /// </summary>
    /// <remarks>
    /// A ray goes down <paramref name="reach"/> metres from the root. If it meets
    /// no ground the character stays down and the answer is null. Every body is
    /// made animated, and the pose is mixed from the fall over
    /// <paramref name="blend"/> seconds.
    /// Does nothing and answers null for a character that is standing.
    /// </remarks>
    public Vector3? StandUp(double reach = 2, double blend = 0.4)
    {
        if (_standing)
        {
            return null;
        }

        var ground = World.Cast(
            Storage.Of(Ragdoll.RootPosition),
            [0.0, -1.0, 0.0],
            reach,
            layers: Ragdoll.Layers);
        if (ground is null)
        {
            return null;
        }

        var feet = new Vector3(ground.At[0], ground.At[1], ground.At[2]);
        Add(feet);
        Ragdoll.SetAll(RagdollMode.Animated);
        Pose?.Recover(blend);
        _standing = true;
        return feet;
    }

    /// <summary>Takes the capsule out of the world if it is there.</summary>
    public void Dispose()
    {
        if (_standing)
        {
            World.Remove(Id);
        }

        _standing = false;
    }

    /// <summary>Stands the capsule with its feet at <paramref name="feet"/> and a skin above the ground.</summary>
    private void Add(Vector3 feet)
    {
        var centre = feet.Y + Shape.Y + Shape.X + Skin;
        World.AddCharacter(
            Id,
            Shape,
            [feet.X, centre, feet.Z],
            stepHeight: StepHeight,
            steepest: Steepest,
            skin: Skin,
            strength: Strength,
            friction: Friction,
            layers: new Layers(Layers.Is, Layers.Cares & ~Ragdoll.Layer));
    }
}
