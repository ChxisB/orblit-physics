using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class PhysicsAddedTests : PhysicsTestBase
{
    [Theory]
    [InlineData(typeof(NativeCommand), 128)]
    [InlineData(typeof(NativeEvent), 56)]
    [InlineData(typeof(NativeCast), 96)]
    [InlineData(typeof(NativeHit), 40)]
    [InlineData(typeof(NativeGround), 64)]
    [InlineData(typeof(NativeFooting), 56)]
    public void EachStructIsTheSizeTheSolverReads(Type type, int size)
    {
        Assert.Equal(size, Marshal.SizeOf(type));
    }

    [Fact]
    public void ReadIntoRefusesAnOutputThatCannotHoldTheLastRow()
    {
        World.Add(2, Shape.Sphere(0.5));

        Assert.Throws<ArgumentException>(() => World.ReadInto([2], new float[6]));
        Assert.Throws<ArgumentException>(() => World.ReadInto([2], new float[10], offset: 4));
        Assert.Throws<ArgumentException>(() => World.ReadInto([2], new float[10], offset: -1));
        Assert.Equal(1, World.ReadInto([2], new float[7]));
        Assert.Equal(0, World.ReadInto([], Array.Empty<float>()));
    }

    [Fact]
    public void ReadIntoAnswersZeroForAStrideTooShortEvenWhenTheOutputIsSmall()
    {
        World.Add(2, Shape.Sphere(0.5));

        Assert.Equal(0, World.ReadInto([2], new float[3], stride: 6));
    }

    [Fact]
    public void ACastMeetsWhatCaresAboutItsOwnLayerAsWellAsWhatItCaresAbout()
    {
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), PhysicsMotion.Fixed, at: [0.0, 5.0, 0.0], layers: new Layers(Is: 1, Cares: 4));

        Assert.Equal(2UL, World.Cast([0.0, 10.0, 0.0], [0.0, -1.0, 0.0], 100.0, layers: new Layers(Is: 4, Cares: 0))?.Body);
        Assert.Null(World.Cast([0.0, 10.0, 0.0], [0.0, -1.0, 0.0], 100.0, layers: new Layers(Is: 8, Cares: 0)));
    }

    [Fact]
    public void GroundMeetsWhatCaresAboutItsLayerAndWhatItCaresAbout()
    {
        // Ground on bit two that watches bit one. A body on bit one that
        // watches nothing is met because the ground watches it, and a body on
        // bit three that watches bit two is met because it watches the ground.
        // One that does neither falls through.
        Assert.True(World.LayGround(
            7,
            HeightsOf(5),
            columns: 5,
            rows: 5,
            spacing: 2.0,
            at: [-4.0, 0.0, -4.0],
            layers: new Layers(Is: 2, Cares: 1)));
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 2.0, 0.0], layers: new Layers(Is: 1, Cares: 0));
        World.Add(3, Shape.Sphere(0.5), at: [2.0, 2.0, 0.0], layers: new Layers(Is: 4, Cares: 2));
        World.Add(4, Shape.Sphere(0.5), at: [-2.0, 2.0, 0.0], layers: new Layers(Is: 4, Cares: 0));

        Run(2);
        Assert.Equal(0.5, HeightOf(2), 0.05);
        Assert.Equal(0.5, HeightOf(3), 0.05);
        Assert.True(HeightOf(4) < -1.0);
    }

    [Fact]
    public void ACommandBufferThatFillsIsSentAndKeepsItsOrder()
    {
        for (ulong id = 1; id <= 200; id++)
        {
            World.Add(id, Shape.Sphere(0.5), at: [id, 0.0, 0.0]);
        }

        Assert.Equal(200, World.Count);
        Assert.Equal(137.0, World.TransformOf(137)![0]);
    }

    [Fact]
    public void ACharacterAddedLastInAFullBufferIsStillACharacter()
    {
        // The body goes in the final slot and its character command is the
        // first of the next send.
        for (ulong id = 1; id < 64; id++)
        {
            World.Add(id, Shape.Sphere(0.5), at: [id, 0.0, 0.0]);
        }

        World.AddCharacter(64, at: [0.0, 5.0, 0.0]);

        Assert.NotNull(World.FootingOf(64));
    }

    [Fact]
    public void ADisposedWorldRefusesEveryKindOfCall()
    {
        var gone = new Physics();
        gone.Dispose();

        Assert.Throws<InvalidOperationException>(() => gone.Alive(1));
        Assert.Throws<InvalidOperationException>(() => gone.TransformOf(1));
        Assert.Throws<InvalidOperationException>(() => gone.FootingOf(1));
        Assert.Throws<InvalidOperationException>(() => gone.Events);
        Assert.Throws<InvalidOperationException>(() => gone.SetStance(1, Shape.Sphere(1.0)));
        Assert.Throws<InvalidOperationException>(() => gone.LayGround(1, new double[4], 2, 2));
        Assert.Throws<InvalidOperationException>(() => gone.Cast([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0));
    }
}
