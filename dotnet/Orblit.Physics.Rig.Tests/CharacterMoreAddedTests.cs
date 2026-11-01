using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class CharacterMoreAddedTests : IDisposable
{
    private readonly Harness _h = new();

    public void Dispose() => _h.Dispose();

    private RagdollCharacter Stand(
        RagdollPose? pose = null,
        double stepHeight = 0.3,
        double steepest = Math.PI / 4,
        double strength = 500) =>
        new(
            _h.Ragdoll,
            800,
            Vector3.Zero(),
            pose,
            layers: new Layers(Is: Human.CharacterLayer),
            stepHeight: stepHeight,
            steepest: steepest,
            strength: strength);

    private void WalkRight()
    {
        for (var i = 0; i < 120; i++)
        {
            _h.Physics.Drive(800, velocity: [2.0, 0.0, 0.0]);
            _h.Physics.Step(Harness.Dt);
        }
    }

    private double X(ulong id) => _h.Physics.TransformOf(id)![0];

    [Fact]
    public void TheSettingsItWasGivenAreTheOnesItKeeps()
    {
        using var character = new RagdollCharacter(
            _h.Ragdoll,
            800,
            Vector3.Zero(),
            layers: new Layers(Is: Human.CharacterLayer),
            stepHeight: 0.5,
            steepest: 0.9,
            skin: 0.02,
            strength: 100,
            friction: 0.2);

        Assert.Equal(0.5, character.StepHeight);
        Assert.Equal(0.9, character.Steepest);
        Assert.Equal(0.02, character.Skin);
        Assert.Equal(100, character.Strength);
        Assert.Equal(0.2, character.Friction);
    }

    [Theory]
    [InlineData(0.1, false)]
    [InlineData(0.3, true)]
    public void ACharacterClimbsAStepNoHigherThanItsStepHeight(double stepHeight, bool climbs)
    {
        _h.Physics.Add(901, Shape.Box(0.5, 0.1, 2), PhysicsMotion.Fixed, at: [1.5, 0.1, 0.0]);
        using var character = Stand(stepHeight: stepHeight);

        WalkRight();

        if (climbs)
        {
            Greater(X(800), 3.5, "distance walked");
        }
        else
        {
            Less(X(800), 1.0, "distance walked");
        }
    }

    [Theory]
    [InlineData(0.4, false)]
    [InlineData(1.0, true)]
    public void ACharacterClimbsASlopeNoSteeperThanItsSteepest(double steepest, bool climbs)
    {
        _h.Physics.Add(
            901,
            Shape.Box(2, 0.1, 2),
            PhysicsMotion.Fixed,
            at: [3.0, 0.0, 0.0],
            rotation: Storage.Of(Quaternion.AxisAngle(new Vector3(0, 0, 1), 0.6)));
        using var character = Stand(steepest: steepest);

        WalkRight();

        if (climbs)
        {
            Greater(X(800), 3.9, "distance walked");
        }
        else
        {
            Less(X(800), 3.0, "distance walked");
        }
    }

    [Theory]
    [InlineData(500, true)]
    [InlineData(1, false)]
    public void ACharacterPushesAHeavyBoxOnlyAsHardAsItsStrengthAllows(double strength, bool pushes)
    {
        _h.Physics.Add(902, Shape.Box(0.5, 0.5, 0.5), PhysicsMotion.Free, at: [1.5, 0.5, 0.0], mass: 100);
        using var character = Stand(strength: strength);

        WalkRight();

        if (pushes)
        {
            Greater(X(902), 1.6, "box");
        }
        else
        {
            Less(X(902), 1.51, "box");
        }
    }

    [Fact]
    public void AKnockWithNoSpreadMakesEveryBodyLimpAtOnce()
    {
        using var character = Stand();

        character.Knock("head", new Vector3(0, 0, 1), spread: 0);

        foreach (var bone in _h.Ragdoll.Bones)
        {
            Assert.Equal(RagdollMode.Limp, _h.Ragdoll.ModeOf(bone));
        }
    }

    [Fact]
    public void StandUpOnlyLooksAsFarDownAsItIsToldTo()
    {
        _h.Physics.SetGravity([0.0, 0.0, 0.0]);
        using var character = Stand();
        character.Knock("head", Vector3.Zero(), spread: 0);
        _h.Run(0.2);

        Assert.Null(character.StandUp(reach: 0.5));
        Assert.True(character.IsDown);

        var feet = character.StandUp(reach: 2);

        Assert.NotNull(feet);
        Near(0, feet.Y, 1e-3, "ground");
        Assert.False(character.IsDown);
    }

    [Theory]
    [InlineData(1.0, 0.6, true)]
    [InlineData(0.4, 0.6, false)]
    [InlineData(0.4, 0.2, true)]
    public void TheMixBackFromTheFallLastsAsLongAsTheBlendAsksFor(double blend, double seconds, bool mixing)
    {
        using var character = Stand(pose: _h.Modifier);
        character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);
        _h.Run(3);
        character.StandUp(blend: blend);

        _h.Run(seconds);

        Assert.Equal(mixing, _h.Modifier.Recovering);
    }

    [Fact]
    public void TheMixBackFromTheFallLastsFortyHundredthsOfASecondByDefault()
    {
        using var character = Stand(pose: _h.Modifier);
        character.Knock("forearm.L", new Vector3(0, 0, 6), spread: 0.05);
        _h.Run(3);
        character.StandUp();

        _h.Run(0.2);
        Assert.True(_h.Modifier.Recovering);

        _h.Run(0.4);
        Assert.False(_h.Modifier.Recovering);
    }

    [Theory]
    [InlineData(0.01)]
    [InlineData(0.3)]
    public void ACharacterStopsItsSkinShortOfAWall(double skin)
    {
        _h.Physics.Add(901, Shape.Box(0.1, 2, 2), PhysicsMotion.Fixed, at: [3.0, 1.0, 0.0]);
        using var character = new RagdollCharacter(
            _h.Ragdoll,
            800,
            Vector3.Zero(),
            layers: new Layers(Is: Human.CharacterLayer),
            skin: skin);

        WalkRight();

        Near(2.9 - 0.3 - skin, X(800), 0.05, "where it stopped");
    }

    [Fact]
    public void DisposingAStandingCharacterOnceTakesTheCapsuleOutAndLeavesItDown()
    {
        var character = Stand();

        character.Dispose();

        Assert.False(_h.Physics.Alive(800));
        Assert.True(character.IsDown);
    }
}
