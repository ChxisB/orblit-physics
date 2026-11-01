using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class CharacterTests : IDisposable
{
    private readonly Harness _h = new();
    private readonly RagdollCharacter _character;

    public CharacterTests()
    {
        _character = new RagdollCharacter(
            _h.Ragdoll,
            800,
            Vector3.Zero(),
            _h.Modifier,
            layers: new Layers(Is: Human.CharacterLayer));
    }

    public void Dispose()
    {
        _character.Dispose();
        _h.Dispose();
    }

    private float[]? Capsule() => _h.Physics.TransformOf(800);

    private void KnockOnForearm() =>
        _character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);

    private void AssertEveryBody(RagdollMode mode)
    {
        foreach (var bone in _h.Ragdoll.Bones)
        {
            Assert.True(_h.Ragdoll.ModeOf(bone) == mode, bone);
        }
    }

    [Fact]
    public void StandingKeepsTheCapsuleWhileFollowingThePose()
    {
        _h.Run(1);

        var capsule = Capsule()!;
        Assert.Equal(0, capsule[0], 0.02);
        Assert.Equal(0, capsule[2], 0.02);
        Assert.Equal(0.6 + 0.3 + 0.01, capsule[1], 0.05);
        Assert.False(_character.IsDown);
    }

    [Fact]
    public void RefusesALayerTheRagdollStillCaresAbout()
    {
        Assert.ThrowsAny<ArgumentException>(() => new RagdollCharacter(
            _h.Ragdoll,
            801,
            Vector3.Zero(),
            layers: new Layers(Is: 4)));
    }

    [Fact]
    public void StandUpOnAStandingCharacterGivesNullAndTheCapsuleStays()
    {
        Assert.Null(_character.StandUp());
        Assert.NotNull(Capsule());
    }

    [Fact]
    public void AHitTakesTheCapsuleOutAndGoesLimpFromTheHitBone()
    {
        KnockOnForearm();

        Assert.True(_character.IsDown);
        Assert.Null(Capsule());
        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("forearm.L"));
        Assert.Equal(RagdollMode.Animated, _h.Ragdoll.ModeOf("head"));

        _h.Run(0.6);
        AssertEveryBody(RagdollMode.Limp);
    }

    [Fact]
    public void LiesOnTheGroundThenStandsUp()
    {
        KnockOnForearm();
        _h.Run(4);
        Assert.True(_h.Ragdoll.IsAtRest());

        var feet = _character.StandUp();

        Assert.NotNull(feet);
        Assert.Equal(0, feet!.Y, 1e-3);
        Assert.NotNull(Capsule());
        Assert.False(_character.IsDown);
        AssertEveryBody(RagdollMode.Animated);
    }

    [Fact]
    public void StaysDownWhenTheGroundIsOutOfReach()
    {
        KnockOnForearm();
        _h.Run(2);

        Assert.Null(_character.StandUp(reach: 0.01));

        Assert.True(_character.IsDown);
        Assert.Null(Capsule());
    }

    [Fact]
    public void ASecondBlowWhileDownHitsTheBodiesAgain()
    {
        KnockOnForearm();
        _h.Run(2);

        _character.Knock("head", new Vector3(0, 0, -4), spread: 0);
        _h.Run(0.5);

        Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf("head"));
    }

    [Fact]
    public void TheOutputIsMixedFromTheFallToTheGetUp()
    {
        KnockOnForearm();
        _h.Run(3);
        var fallenHead = _h.HeadOf("head");

        var feet = _character.StandUp(blend: 0.4)!;
        _h.Model = Matrix4.TranslationValues(feet.X, feet.Y, feet.Z);

        Assert.True(_h.Modifier.Recovering);
        _h.Frame();
        Less((_h.HeadOf("head") - fallenHead).Length, 0.05);

        _h.Run(0.6);
        Assert.False(_h.Modifier.Recovering);
        var head = _h.HeadOf("head");
        Assert.Equal(feet.X, head.X, 1e-3);
        Assert.Equal(feet.Y + 1.45, head.Y, 1e-3);
        Assert.Equal(feet.Z, head.Z, 1e-3);
    }
}
