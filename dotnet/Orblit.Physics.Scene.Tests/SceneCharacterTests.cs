using Orblit.Motion;
using Orblit.Scene;
using Orblit.Sequence;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneCharacterTests : SceneTestBase
{
    /// <summary>Two metres along Z a second, looping, with the hips doing the walking.</summary>
    private static ClipDocument WalkClip() => new(
        "Walk",
        1,
        WhenDone.Loop,
        rootMotion: new RootMotion(bone: "hips"),
        channels:
        [
            new ClipChannel<Vector3>(
                "",
                "position",
                ChannelKind.Vector,
                [new Key<Vector3>(0, new Vector3(0, 1, 0), Hold.Linear), new Key<Vector3>(1, new Vector3(0, 1, 2))],
                bone: "hips"),
            new ClipChannel<Quaternion>(
                "",
                "rotation",
                ChannelKind.Rotation,
                [new Key<Quaternion>(0, Quaternion.Identity(), Hold.Linear), new Key<Quaternion>(1, Quaternion.Identity())],
                bone: "hips"),
        ]);

    private static RootMotionWalk Walking()
    {
        var player = new ClipPlayer(WalkClip());
        player.Play();
        return new RootMotionWalk(player);
    }

    private static SceneEntity Solid(string id, Vector3 size, Vector3 at) => Entity(
        id,
        new BodyComponent(size: size, motion: BodyMotion.Fixed),
        at);

    /// <summary>
    /// A floor, a step 0.2 m tall from z 1 to 2, a beam from z 3 to 6 with its underside 1.2 m up, and a
    /// hero at the origin, written out and read back as a file so that nothing here is only in memory.
    /// </summary>
    private static SceneDocument World(double crouchHeight = 1.0)
    {
        var built = SceneOf(
            Floor(),
            Solid("step", new Vector3(2, 0.2, 1), new Vector3(0, 0.1, 1.5)),
            Solid("beam", new Vector3(2, 0.2, 3), new Vector3(0, 1.3, 4.5)),
            Entity(
                "hero",
                new BodyComponent(
                    BodyShape.Capsule,
                    motion: BodyMotion.Character,
                    radius: 0.3,
                    height: 1.8,
                    centre: new Vector3(0, 0.9, 0),
                    crouchHeight: crouchHeight),
                new Vector3(0, 0.01, 0)));
        var load = SceneDocument.Decode(built.Encode());
        Assert.False(load.HasProblems, string.Join("; ", load.Problems));
        return load.Document;
    }

    private static double FeetOf(ScenePhysics scene) => TransformOf(scene.Document, "hero").Position.Y;

    private static double DepthOf(ScenePhysics scene) => TransformOf(scene.Document, "hero").Position.Z;

    [Fact]
    public void ACharacterInAFileIsSweptAndTheFileKeepsItsNumbers()
    {
        var scene = Make(World());
        var body = (BodyComponent)scene.Document["hero"]![SceneComponents.Body]!;

        Assert.Equal(BodyMotion.Character, body.Motion);
        Assert.Equal(1.0, body.CrouchHeight);
        Assert.NotNull(scene.Physics.FootingOf(scene.BodyOf("hero")!.Value));
        Assert.Null(scene.Physics.FootingOf(scene.BodyOf("step")!.Value));
    }

    [Fact]
    public void RootMotionTakesItUpAStepThenUnderABeamWhereItCannotStand()
    {
        var scene = Make(World());

        Assert.True(scene.Crouch("hero", true));
        Assert.True(scene.IsCrouching("hero"));
        scene.Walk("hero", Walking());

        // Two metres a second, so three quarters of one is the step's middle.
        Run(scene, 0.75);
        Near(1.5, DepthOf(scene), 0.15, "on the step");
        Near(0.2, FeetOf(scene), 0.03, "feet on its top");

        Run(scene, 1.25);
        Assert.True(DepthOf(scene) > 3.2, "under the beam");
        Near(0.0, FeetOf(scene), 0.03, "off the step again");
        Assert.False(scene.Crouch("hero", false), "a 1.8 m capsule is not 1.2 m tall");
        Assert.True(scene.IsCrouching("hero"));

        Run(scene, 1.5);
        Assert.True(DepthOf(scene) > 6.3, "out the far side");
        Assert.True(scene.Crouch("hero", false));
        Assert.False(scene.IsCrouching("hero"));
        Near(0.0, FeetOf(scene), 0.03, "feet stayed put");
    }

    [Fact]
    public void AStandingCharacterIsStoppedByTheBeamItCannotGoUnder()
    {
        var scene = Make(World());
        scene.Walk("hero", Walking());

        Run(scene, 4.0);

        Assert.True(DepthOf(scene) < 2.8, "stopped short of the beam");
        var footing = scene.Physics.FootingOf(scene.BodyOf("hero")!.Value)!;
        Assert.True(footing.Wall);
    }

    [Fact]
    public void ABodyThatIsNotACharacterHasNoStanceToTake()
    {
        var scene = Make(World());

        Assert.False(scene.Crouch("step", true));
        Assert.False(scene.Crouch("nobody", true));
    }

    [Fact]
    public void ACharacterWithNoCrouchHeightCannotCrouch()
    {
        var scene = Make(World(crouchHeight: 0));

        Assert.False(scene.Crouch("hero", true));
        Assert.False(scene.IsCrouching("hero"));
    }

    [Fact]
    public void ASnapshotKeepsACrouchAndARestoreStandsItWhereItWas()
    {
        var scene = Make(World());
        scene.Crouch("hero", true);
        var snapshot = scene.Snapshot();
        scene.Crouch("hero", false);
        Assert.False(scene.IsCrouching("hero"));

        scene.Restore(snapshot);
        snapshot.Dispose();

        Assert.True(scene.IsCrouching("hero"));
    }
}
