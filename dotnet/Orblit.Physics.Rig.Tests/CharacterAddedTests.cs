using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class CharacterAddedTests : IDisposable
{
    private readonly Harness _h = new();

    public void Dispose() => _h.Dispose();

    private RagdollCharacter Stand(Vector3? at = null, RagdollPose? pose = null, Shape? shape = null, double skin = 0.01) =>
        new(
            _h.Ragdoll,
            800,
            at ?? Vector3.Zero(),
            pose,
            shape,
            new Layers(Is: Human.CharacterLayer),
            skin: skin);

    [Fact]
    public void StandsItsCapsuleASkinAboveTheFeet()
    {
        using var character = Stand(new Vector3(2, 1, -3));

        var t = _h.Physics.TransformOf(800)!;

        Assert.Equal(2, t[0], 1e-6);
        Assert.Equal(1 + 0.6 + 0.3 + 0.01, t[1], 1e-6);
        Assert.Equal(-3, t[2], 1e-6);
    }

    [Fact]
    public void ACustomShapeAndSkinSetTheHeightOfTheCentre()
    {
        using var character = Stand(shape: Shape.Capsule(0.2, 0.5), skin: 0.05);

        Assert.Equal(0.5 + 0.2 + 0.05, _h.Physics.TransformOf(800)![1], 1e-6);
    }

    [Fact]
    public void HasTheDocumentedDefaults()
    {
        using var character = Stand();

        Assert.Equal(0.3, character.StepHeight);
        Assert.Equal(Math.PI / 4, character.Steepest);
        Assert.Equal(0.01, character.Skin);
        Assert.Equal(500, character.Strength);
        Assert.Equal(0.5, character.Friction);
        Assert.Equal(0.3, character.Shape.X);
        Assert.Equal(0.6, character.Shape.Y);
        Assert.Equal(800UL, character.Id);
        Assert.Null(character.Pose);
        Assert.Same(_h.Ragdoll, character.Ragdoll);
    }

    [Fact]
    public void KnockingTwiceTakesTheCapsuleOutOnlyOnce()
    {
        using var character = Stand();

        character.Knock("head", new Vector3(0, 0, 1));
        character.Knock("head", new Vector3(0, 0, 1));

        Assert.True(character.IsDown);
        Assert.Null(_h.Physics.TransformOf(800));
    }

    [Fact]
    public void AKnockAtAPointTurnsTheBodyItHit()
    {
        using var character = Stand();
        using var other = new Harness();
        using var second = new RagdollCharacter(
            other.Ragdoll,
            800,
            Vector3.Zero(),
            layers: new Layers(Is: Human.CharacterLayer));

        character.Knock("forearm.L", new Vector3(0, 0, 5), new Vector3(0.75, 1.4, 0), spread: 0);
        second.Knock("forearm.L", new Vector3(0, 0, 5), spread: 0);
        _h.Physics.Step(Harness.Dt);
        other.Physics.Step(Harness.Dt);

        var tip = _h.Physics.VelocityOf(_h.Ragdoll.IdOf("forearm.L"))!;
        var middle = other.Physics.VelocityOf(other.Ragdoll.IdOf("forearm.L"))!;
        Greater(Math.Abs(tip[4]), Math.Abs(middle[4]) * 1.5);
    }

    [Fact]
    public void StandUpPutsTheCapsuleUnderTheRootAtTheGround()
    {
        using var character = Stand(pose: _h.Modifier);
        character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);
        _h.Run(4);
        var root = _h.Ragdoll.RootPosition;

        var feet = character.StandUp()!;

        var t = _h.Physics.TransformOf(800)!;
        Assert.Equal(root.X, feet.X, 1e-3);
        Assert.Equal(root.Z, feet.Z, 1e-3);
        Assert.Equal(feet.X, t[0], 1e-5);
        Assert.Equal(feet.Y + 0.91, t[1], 1e-5);
        Assert.Equal(feet.Z, t[2], 1e-5);
    }

    [Fact]
    public void StandUpWithNoPoseGetsTheCharacterUpAllTheSame()
    {
        using var character = Stand();
        character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);
        _h.Run(3);

        Assert.NotNull(character.StandUp());

        Assert.False(character.IsDown);
        Assert.False(_h.Modifier.Recovering);
    }

    [Fact]
    public void StandUpStartsTheMixOnlyWhenThePoseWasGiven()
    {
        using var character = Stand(pose: _h.Modifier);
        character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);
        _h.Run(3);

        character.StandUp(blend: 0.4);

        Assert.True(_h.Modifier.Recovering);
    }

    [Fact]
    public void DisposeTakesTheCapsuleOutAndNothingTwice()
    {
        var character = Stand();

        character.Dispose();
        character.Dispose();

        Assert.Null(_h.Physics.TransformOf(800));
    }

    [Fact]
    public void ADownCharacterHasNoCapsuleToDispose()
    {
        var character = Stand();
        character.Knock("head", new Vector3(0, 0, 1));

        character.Dispose();

        Assert.Null(_h.Physics.TransformOf(800));
    }
}
