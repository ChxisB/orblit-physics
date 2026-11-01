using Orblit.Motion;
using Orblit.Sequence;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene.Tests;

public sealed class RootMotionWalkAddedTests
{
    /// <summary>
    /// Two metres a second along Z for two seconds, with a half turn about <paramref name="up"/> over the
    /// same time, all of it given to the character.
    /// </summary>
    private static ClipDocument Clip(Vector3 up) => new(
        "Walk",
        2,
        WhenDone.Loop,
        rootMotion: new RootMotion(bone: "hips", turns: true, up: up),
        channels:
        [
            new ClipChannel<Vector3>(
                "",
                "position",
                ChannelKind.Vector,
                [new Key<Vector3>(0, new Vector3(0, 1, 0), Hold.Linear), new Key<Vector3>(2, new Vector3(0, 1, 4))],
                bone: "hips"),
            new ClipChannel<Quaternion>(
                "",
                "rotation",
                ChannelKind.Rotation,
                [
                    new Key<Quaternion>(0, Quaternion.Identity(), Hold.Linear),
                    new Key<Quaternion>(2, Quaternion.AxisAngle(up, Math.PI)),
                ],
                bone: "hips"),
        ]);

    private static ClipPlayer Playing(Vector3 up)
    {
        var player = new ClipPlayer(Clip(up));
        player.Play();
        return player;
    }

    private static Quaternion YawQuarter() => Quaternion.AxisAngle(new Vector3(0, 1, 0), Math.PI / 2);

    private static double Gap(Quaternion a, Quaternion b) =>
        Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y) + Math.Abs(a.Z - b.Z) + Math.Abs(a.W - b.W);

    private static void AssertTurn(Quaternion expected, Quaternion actual)
    {
        Assert.InRange(actual.X - expected.X, -1e-9, 1e-9);
        Assert.InRange(actual.Y - expected.Y, -1e-9, 1e-9);
        Assert.InRange(actual.Z - expected.Z, -1e-9, 1e-9);
        Assert.InRange(actual.W - expected.W, -1e-9, 1e-9);
    }

    [Fact]
    public void AFacingOfNothingWalksAlongTheClipsOwnAxis()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)));

        var ask = walk.Ask(0.5, null);

        Assert.Equal(0.0, ask[0], 1e-9);
        Assert.Equal(2.0, ask[2], 1e-9);
    }

    [Fact]
    public void AHeadingTurnsTheDirectionTheClipWalks()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)), YawQuarter());

        var ask = walk.Ask(0.5, null);

        // A quarter turn about Y takes forward along Z to along X.
        Assert.Equal(2.0, ask[0], 1e-9);
        Assert.Equal(0.0, ask[2], 1e-9);
    }

    [Fact]
    public void AScaleStretchesTheDistancesOfTheClip()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)), scale: 2.0);

        var ask = walk.Ask(0.5, null);

        Assert.Equal(4.0, ask[2], 1e-9);
    }

    [Fact]
    public void TheClipsTurnIsAppliedAfterTheHeadingWhichKeepsItsOrder()
    {
        var up = new Vector3(1, 0, 0);
        var heading = YawQuarter();
        var turned = Playing(up).Advance(1.0).Moved.Rotation;
        Assert.True(Gap(heading * turned, turned * heading) > 0.1, "the two orders must differ for this to prove anything");
        var walk = new RootMotionWalk(Playing(up), heading);

        walk.Ask(1.0, null);

        AssertTurn((heading * turned).Normalized(), walk.Heading);
    }

    [Fact]
    public void AHeadingThatDriftedFromUnitLengthIsMadeUnitAgain()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)), new Quaternion(0, 0, 0, 2));

        walk.Ask(0.5, null);

        Assert.Equal(1.0, walk.Heading.Length, 1e-9);
    }

    [Fact]
    public void WhatTheFootingSaysOfTheFallIsKeptAndGravityIsAddedToIt()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)));
        var footing = new PhysicsFooting(1, [0.0, 1.0, 0.0], [0.0, -3.0, 0.0], [0.0, 0.0, 0.0], 0, false);

        var falling = walk.Ask(0.5, footing);
        var free = new RootMotionWalk(Playing(new Vector3(0, 1, 0))).Ask(0.5, null);

        Assert.Equal(-3.0 + (-9.81 * 0.5), falling[1], 1e-9);
        Assert.Equal(-9.81 * 0.5, free[1], 1e-9);
    }

    [Fact]
    public void AGravityOfTheGameIsTheOneTheWalkAdds()
    {
        var walk = new RootMotionWalk(Playing(new Vector3(0, 1, 0)), gravity: [0.0, -2.0, 0.0]);

        var ask = walk.Ask(0.5, null);

        Assert.Equal(-1.0, ask[1], 1e-9);
    }
}
