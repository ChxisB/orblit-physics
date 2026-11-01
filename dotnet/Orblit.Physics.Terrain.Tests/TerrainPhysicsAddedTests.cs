using Orblit.Terrain;
using TerrainData = Orblit.Terrain.Terrain;

namespace Orblit.Physics.Terrain.Tests;

public class TerrainPhysicsAddedTests : TerrainTestBase
{
    /// <summary>A terrain of flat regions, eight texels across, at the given places.</summary>
    private static TerrainData Flat(double spacing = 1.0, params (long X, long Z)[] places)
    {
        var terrain = new TerrainData(regionSize: 8, spacing: spacing);
        foreach (var (x, z) in places)
        {
            terrain.AddRegion(new RegionKey(x, z));
        }

        return terrain;
    }

    private static TerrainData Block(int reach)
    {
        var terrain = new TerrainData(regionSize: 8);
        for (var z = -reach; z <= reach; z++)
        {
            for (var x = -reach; x <= reach; x++)
            {
                terrain.AddRegion(new RegionKey(x, z));
            }
        }

        return terrain;
    }

    /// <summary>
    /// How far a box set flat on a slope has moved after two seconds, on ground
    /// given the friction, or the ground's own when none is given.
    /// </summary>
    private static double Slid(double? friction, double slope = 0.6)
    {
        using var world = new Physics();
        var terrain = new TerrainData(regionSize: 8);
        foreach (var key in new[] { new RegionKey(0, 0), new RegionKey(1, 0) })
        {
            terrain.FillHeights(key, (x, _) => slope * x);
        }

        var ground = friction is { } given
            ? new TerrainPhysics(world, terrain, friction: given)
            : new TerrainPhysics(world, terrain);
        ground.Sync(x: 8, z: 4, radius: 1);
        var tilt = Math.Atan(slope);
        world.Add(
            2,
            Shape.Box(0.3, 0.3, 0.3),
            at: [8.0 - (0.3 * Math.Sin(tilt)), (8.0 * slope) + (0.3 * Math.Cos(tilt)) + 0.01, 4.0],
            rotation: [0.0, 0.0, Math.Sin(tilt / 2), Math.Cos(tilt / 2)],
            friction: 1.0);
        Steps(world, 120);
        return Math.Abs(world.TransformOf(2)![0] - 8.0);
    }

    /// <summary>How high a ball dropped from three metres has risen after its first landing.</summary>
    private static double Rebounded(double restitution)
    {
        using var world = new Physics();
        var terrain = new TerrainData(regionSize: 8);
        terrain.AddRegion(new RegionKey(0, 0));
        new TerrainPhysics(world, terrain, restitution: restitution).Sync(x: 4, z: 4, radius: 1);
        world.Add(2, Shape.Sphere(0.5), at: [4.0, 3.0, 4.0], restitution: 0.5);
        var rising = false;
        var peak = 0.0;
        for (var i = 0; i < 180; i++)
        {
            world.Step(1.0 / 60);
            rising |= world.VelocityOf(2)![1] > 0;
            peak = rising ? Math.Max(peak, world.TransformOf(2)![1]) : peak;
        }

        return peak;
    }

    private static void Steps(Physics world, int count)
    {
        for (var i = 0; i < count; i++)
        {
            world.Step(1.0 / 60);
        }
    }

    [Fact]
    public void TheGroundHasTheFrictionItWasGiven()
    {
        // The box grips at 1.0 and the slope is 0.6, so only the ground's own
        // friction decides whether it holds, whatever way the two are mixed.
        var slick = Slid(friction: 0.0);
        var grip = Slid(friction: 1.0);

        Assert.True(slick > 2.0, $"slick ground: {slick}");
        Assert.True(grip < 0.5, $"gripping ground: {grip}");
    }

    [Fact]
    public void TheGroundHasTheRestitutionItWasGiven()
    {
        var dull = Rebounded(restitution: 0.0);
        var lively = Rebounded(restitution: 1.0);

        Assert.True(lively > dull + 0.3, $"{lively} against {dull}");
    }

    [Fact]
    public void TheGroundMeetsWhatCaresAboutItsLayerAndWhatItCaresAbout()
    {
        // As for any ground: on bit two and watching bit one. A body on bit one
        // that watches nothing lands, and one on bit three that watches nothing
        // falls through.
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)), layers: new Layers(Is: 2, Cares: 1));
        ground.Sync(x: 4, z: 4, radius: 1);
        World.Add(2, Shape.Sphere(0.5), at: [2.0, 2.0, 4.0], layers: new Layers(Is: 1, Cares: 0));
        World.Add(3, Shape.Sphere(0.5), at: [5.0, 2.0, 4.0], layers: new Layers(Is: 4, Cares: 0));

        Run(2);

        Assert.Equal(0.5, HeightOf(2), 0.05);
        Assert.True(HeightOf(3) < -1.0);
    }

    [Fact]
    public void ByDefaultTheGroundMeetsEveryLayer()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)));
        ground.Sync(x: 4, z: 4, radius: 1);
        World.Add(2, Shape.Sphere(0.5), at: [2.0, 2.0, 4.0], layers: new Layers(Is: 4, Cares: 0));

        Run(2);

        Assert.Equal(0.5, HeightOf(2), 0.05);
    }

    [Theory]
    [InlineData(5.0, 6.0)]
    [InlineData(6.0, 5.0)]
    [InlineData(19.0, 6.0)]
    [InlineData(18.0, 5.0)]
    [InlineData(5.0, 18.0)]
    [InlineData(6.0, 19.0)]
    [InlineData(19.0, 18.0)]
    [InlineData(18.0, 19.0)]
    public void ARegionIsNearWhenTheStraightLineToItsSquareIsWithinTheRadius(double x, double z)
    {
        // The square is 8 to 16 each way. Every point here is 3 off one side
        // and 2 off the other, which is 3.606 to the corner.
        var ground = new TerrainPhysics(World, Flat(1.0, (1, 1)));

        Assert.Equal(0, ground.Sync(x, z, radius: 3.6));
        Assert.Equal(1, ground.Sync(x, z, radius: 3.62));
    }

    [Fact]
    public void ARegionIsNearByTheStraightLineToItsCorner()
    {
        // Three off in each way: 4.24 metres to the corner, not 3 and not 6.
        var ground = new TerrainPhysics(World, Flat(1.0, (1, 1)));

        Assert.Equal(0, ground.Sync(x: 5, z: 5, radius: 4.2));
        Assert.Equal(1, ground.Sync(x: 5, z: 5, radius: 4.25));
    }

    [Fact]
    public void ARegionAtTheRadiusExactlyIsLaid()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0), (1, 0)));

        Assert.Equal(2, ground.Sync(x: 4, z: 4, radius: 4));
    }

    [Fact]
    public void ARegionIsTakenUpOnlyOncePastTheRadiusAndItsWidth()
    {
        // Eight texels at a spacing of two is sixteen metres a region.
        var ground = new TerrainPhysics(World, Flat(2.0, (0, 0)));
        ground.Sync(x: 8, z: 8, radius: 1);

        // 16 + 1 = 17 metres from the region's edge is the limit, and 17 stays.
        ground.Sync(x: 33, z: 8, radius: 1);
        Assert.Equal([new RegionKey(0, 0)], ground.Laid);

        ground.Sync(x: 33.5, z: 8, radius: 1);
        Assert.Empty(ground.Laid);
    }

    [Fact]
    public void TheWidthOfARegionIsItsTexelsTimesTheSpacing()
    {
        var ground = new TerrainPhysics(World, Flat(0.5, (0, 0)));
        ground.Sync(x: 2, z: 2, radius: 1);

        // A region is 4 metres across here, so it goes at 1 + 4 = 5 past its edge.
        ground.Sync(x: 9, z: 2, radius: 1);
        Assert.Equal([new RegionKey(0, 0)], ground.Laid);

        ground.Sync(x: 9.5, z: 2, radius: 1);
        Assert.Empty(ground.Laid);
    }

    [Fact]
    public void ARadiusOfNothingLaysTheRegionTheDotIsIn()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0), (1, 0)));

        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 0));
        Assert.Equal([new RegionKey(0, 0)], ground.Laid);
    }

    [Theory]
    [InlineData(double.NaN, 0.0, 1.0, "x")]
    [InlineData(double.PositiveInfinity, 0.0, 1.0, "x")]
    [InlineData(0.0, double.NaN, 1.0, "z")]
    [InlineData(0.0, double.NegativeInfinity, 1.0, "z")]
    [InlineData(0.0, 0.0, double.NaN, "radius")]
    [InlineData(0.0, 0.0, double.PositiveInfinity, "radius")]
    [InlineData(0.0, 0.0, -0.5, "radius")]
    public void ASyncNeedsAPointAndARadiusThatAreNumbers(double x, double z, double radius, string name)
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)));

        var refused = Assert.Throws<ArgumentOutOfRangeException>(() => ground.Sync(x, z, radius));

        Assert.Equal(name, refused.ParamName);
    }

    [Fact]
    public void ASyncThatIsRefusedLaysAndTakesUpNothing()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0), (5, 0)));
        ground.Sync(x: 4, z: 4, radius: 1);

        Assert.Throws<ArgumentOutOfRangeException>(() => ground.Sync(x: 44, z: 4, radius: -1));

        Assert.Equal([new RegionKey(0, 0)], ground.Laid);
    }

    [Theory]
    [InlineData(0, 0, 1)]
    [InlineData(-1, -1, 1)]
    [InlineData(0, -1, 1)]
    [InlineData(1, -1, 1)]
    [InlineData(-1, 0, 1)]
    [InlineData(1, 0, 1)]
    [InlineData(-1, 1, 1)]
    [InlineData(0, 1, 1)]
    [InlineData(1, 1, 1)]
    [InlineData(2, 0, 0)]
    [InlineData(-2, 0, 0)]
    [InlineData(0, 2, 0)]
    [InlineData(0, -2, 0)]
    [InlineData(2, 2, 0)]
    [InlineData(-2, -2, 0)]
    public void ARegionIsLaidAgainWhenItOrOneOfItsEightNeighboursChanges(int dx, int dz, int expected)
    {
        var terrain = Block(2);
        var ground = new TerrainPhysics(World, terrain);
        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));

        terrain.RegionAt(new RegionKey(dx, dz))!.SetHeight(3, 3, 2.0);

        Assert.Equal(expected, ground.Sync(x: 4, z: 4, radius: 1));
        Assert.Equal(0, ground.Sync(x: 4, z: 4, radius: 1));
    }

    [Fact]
    public void ARegionTakenOutAndPutBackIsAChange()
    {
        var terrain = Flat(1.0, (0, 0), (1, 0));
        var ground = new TerrainPhysics(World, terrain);
        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));

        terrain.RemoveRegion(new RegionKey(1, 0));
        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));

        terrain.AddRegion(new RegionKey(1, 0));
        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));
        Assert.Equal(0, ground.Sync(x: 4, z: 4, radius: 1));
    }

    [Fact]
    public void WhileTheStrokeGoesOnWhatIsNearIsStillLaidAndWhatIsFarStillTakenUp()
    {
        var terrain = Flat(1.0, (0, 0), (1, 0), (6, 0));
        var ground = new TerrainPhysics(World, terrain);
        ground.Sync(x: 4, z: 4, radius: 1, refresh: false);
        terrain.RegionAt(new RegionKey(0, 0))!.SetHeight(3, 3, 2.0);

        Assert.Equal(1, ground.Sync(x: 12, z: 4, radius: 1, refresh: false));
        Assert.Equal([new RegionKey(0, 0), new RegionKey(1, 0)], ground.Laid);

        ground.Sync(x: 52, z: 4, radius: 1, refresh: false);
        Assert.Equal([new RegionKey(6, 0)], ground.Laid);
    }

    [Fact]
    public void ARegionLaidAgainKeepsItsPlaceAndOneTakenUpLosesIt()
    {
        var terrain = Flat(1.0, (0, 0), (1, 0), (2, 0));
        var ground = new TerrainPhysics(World, terrain);
        ground.Sync(x: 12, z: 4, radius: 5);
        Assert.Equal([new RegionKey(0, 0), new RegionKey(1, 0), new RegionKey(2, 0)], ground.Laid);

        terrain.RegionAt(new RegionKey(0, 0))!.SetHeight(3, 3, 2.0);
        ground.Sync(x: 12, z: 4, radius: 5);
        Assert.Equal([new RegionKey(0, 0), new RegionKey(1, 0), new RegionKey(2, 0)], ground.Laid);

        terrain.RemoveRegion(new RegionKey(0, 0));
        terrain.AddRegion(new RegionKey(0, 0));
        terrain.RemoveRegion(new RegionKey(1, 0));
        ground.Sync(x: 12, z: 4, radius: 5);
        Assert.Equal([new RegionKey(0, 0), new RegionKey(2, 0)], ground.Laid);

        terrain.AddRegion(new RegionKey(1, 0));
        ground.Sync(x: 12, z: 4, radius: 5);
        Assert.Equal([new RegionKey(0, 0), new RegionKey(2, 0), new RegionKey(1, 0)], ground.Laid);
    }

    [Fact]
    public void ARegionNotLaidHasNoBodyAndNoOneIsItsBody()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0), (3, 0)));
        ground.Sync(x: 4, z: 4, radius: 1);

        Assert.Equal(TerrainPhysics.IdOf(new RegionKey(0, 0)), ground.BodyOf(new RegionKey(0, 0)));
        Assert.Null(ground.BodyOf(new RegionKey(3, 0)));
        Assert.Null(ground.RegionOf(TerrainPhysics.IdOf(new RegionKey(3, 0))));
    }

    [Fact]
    public void ANumberingThatGivesZeroLaysNothing()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)), numbering: _ => 0);

        Assert.Equal(0, ground.Sync(x: 4, z: 4, radius: 1));
        Assert.Empty(ground.Laid);
        Assert.Null(ground.BodyOf(new RegionKey(0, 0)));
        Assert.Equal(0, World.Count);
    }

    [Fact]
    public void ANumberIsPackedFromTheTwoNumbersOfTheKey()
    {
        Assert.Equal(0xC000_0000_0000_0000UL, TerrainPhysics.IdOf(new RegionKey(0, 0)));
        Assert.Equal(0xC000_0000_1000_0002UL, TerrainPhysics.IdOf(new RegionKey(1, 2)));
        Assert.Equal(0xC0FF_FFFF_FFFF_FFFFUL, TerrainPhysics.IdOf(new RegionKey(-1, -1)));
        Assert.Equal(TerrainPhysics.IdOf(new RegionKey(5, 7)), TerrainPhysics.IdOf(new RegionKey(5 + (1L << 28), 7 + (1L << 28))));
    }

    [Fact]
    public void ATerrainWithNoRegionsLaysNothing()
    {
        var ground = new TerrainPhysics(World, new TerrainData(regionSize: 8));

        Assert.Equal(0, ground.Sync(x: 4, z: 4, radius: 100));
        Assert.Empty(ground.Laid);
    }

    [Fact]
    public void ClearingTwiceIsClearingOnce()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)));
        ground.Sync(x: 4, z: 4, radius: 1);

        ground.Clear();
        ground.Clear();

        Assert.Empty(ground.Laid);
        Assert.Equal(0, World.Count);
    }

    [Fact]
    public void ARegionLaidAfterClearingIsLaidAsNew()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (0, 0)));
        ground.Sync(x: 4, z: 4, radius: 1);
        ground.Clear();

        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));
        Assert.True(World.Alive(TerrainPhysics.IdOf(new RegionKey(0, 0))));
    }

    [Fact]
    public void ByDefaultTheGroundHasAFrictionOfAHalf()
    {
        // A slope of nine in ten is steep enough for the box to slide further
        // on ground of 0.4 than of 0.5, so a default of anything else shows.
        var byDefault = Slid(friction: null, slope: 0.9);

        Assert.Equal(Slid(friction: 0.5, slope: 0.9), byDefault);
        Assert.True(Slid(friction: 0.4, slope: 0.9) > byDefault + 0.5);
    }

    [Fact]
    public void TheGroundOfARegionEndsWhereTheRegionDoes()
    {
        // The margin of a neighbour's heights is laid in the field but never
        // stood on, so a ray a hand past the region's own edge meets nothing.
        var ground = new TerrainPhysics(World, Block(1));
        ground.Sync(x: 4, z: 4, radius: 1);

        Assert.Equal(0.0, CastAt(7.9, 4.0));
        Assert.Equal(0.0, CastAt(0.1, 4.0));
        Assert.Equal(0.0, CastAt(4.0, 7.9));
        Assert.Equal(0.0, CastAt(4.0, 0.1));
        Assert.Null(CastAt(8.1, 4.0));
        Assert.Null(CastAt(-0.1, 4.0));
        Assert.Null(CastAt(4.0, 8.1));
        Assert.Null(CastAt(4.0, -0.1));
    }

    [Fact]
    public void ARegionBelowZeroIsLaidWhereThePointIsInIt()
    {
        var ground = new TerrainPhysics(World, Flat(1.0, (-13, -9)));

        Assert.Equal(1, ground.Sync(x: -100, z: -68, radius: 1));
        Assert.Equal([new RegionKey(-13, -9)], ground.Laid);
    }
}
