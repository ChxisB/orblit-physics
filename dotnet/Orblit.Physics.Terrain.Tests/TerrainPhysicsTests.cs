using Orblit.Terrain;
using TerrainData = Orblit.Terrain.Terrain;

namespace Orblit.Physics.Terrain.Tests;

public class TerrainPhysicsTests : TerrainTestBase
{
    [Fact]
    public void TheGroundLaidIsTheGroundDrawnHolesAndAll()
    {
        // Small regions, a spacing that is not one, a region on each side of the
        // origin and bumps that are not symmetric in anything: every way a texel
        // could land a place away from where it belongs.
        var terrain = new TerrainData(regionSize: 8, spacing: 0.75);
        static double Bumps(double x, double z) =>
            Math.Sin(x * 0.9) + (0.6 * Math.Cos((z * 1.3) + 0.4)) + (0.05 * x);
        foreach (var key in new[]
        {
            new RegionKey(-1, -1), new RegionKey(0, -1), new RegionKey(-1, 0), new RegionKey(0, 0), new RegionKey(1, 0),
        })
        {
            terrain.FillHeights(key, Bumps);
        }

        terrain.RegionAt(new RegionKey(0, 0))!.SetCover(3, 5, Cover.Auto.WithHole(true));

        var ground = new TerrainPhysics(World, terrain);
        Assert.Equal(5, ground.Sync(x: 0, z: 0, radius: 20));

        var random = new Random(7);
        var holes = 0;
        for (var n = 0; n < 600; n++)
        {
            var x = -7.0 + (random.NextDouble() * 20.0);
            var z = -7.0 + (random.NextDouble() * 14.0);
            var drawn = terrain.HeightAt(x, z);
            var met = CastAt(x, z);
            if (drawn is null)
            {
                holes++;
                Assert.True(met is null, $"nothing is drawn at ({x}, {z})");
            }
            else
            {
                Assert.True(met is not null, $"nothing is met at ({x}, {z})");
                Assert.Equal(drawn.Value, met!.Value, 1e-3);
            }
        }

        // The hole, the gap where region (1, -1) is missing, and the far edge of
        // region (1, 0), which has no region after it to finish its last cells.
        Assert.True(holes > 20);
        Assert.True(CastAt(3.4 * 0.75, 5.2 * 0.75) is null, "the hole");
    }

    [Fact]
    public void ACrateInAValleyAlongTheLineBetweenRegionsSitsInIt()
    {
        // The valley's floor is the line x = 8, the last texel of region 0 and
        // the first of region 1. Each region knows the other's slope only
        // through its margin; without it, neither would hold the crate up
        // against the other side.
        var terrain = new TerrainData(regionSize: 8);
        foreach (var key in new[] { new RegionKey(0, 0), new RegionKey(1, 0) })
        {
            terrain.FillHeights(key, (x, _) => Math.Abs(x - 8) * 0.5);
        }

        new TerrainPhysics(World, terrain).Sync(x: 8, z: 4, radius: 1);

        World.Add(2, Shape.Box(0.4, 0.4, 0.4), at: [8.0, 2.0, 4.0], friction: 0.8);
        Run(3);

        var at = World.TransformOf(2)!;
        Assert.True(Math.Abs(at[0] - 8.0) <= 0.02, "still in the middle");
        // Corners 0.4 either side of the floor, on slopes of one in two.
        Assert.Equal(0.6, at[1], 0.03);
        Assert.True(Math.Abs(World.VelocityOf(2)![0]) < 0.01);
    }

    [Fact]
    public void SyncLaysWhatComesNearAndTakesUpWhatGoesFar()
    {
        var terrain = new TerrainData(regionSize: 8);
        for (var x = 0; x < 6; x++)
        {
            terrain.AddRegion(new RegionKey(x, 0));
        }

        var ground = new TerrainPhysics(World, terrain);
        static RegionKey Key(int x) => new(x, 0);

        Assert.Equal(1, ground.Sync(x: 4, z: 4, radius: 1));
        Assert.Equal([Key(0)], ground.Laid);
        Assert.True(World.Alive(TerrainPhysics.IdOf(Key(0))));

        // Region 1 starts 4 metres off, region 3 is 4 past the end of region 2,
        // and region 0 ends 12 back: further than the radius but not by a
        // region's width, so it stays.
        Assert.Equal(3, ground.Sync(x: 20, z: 4, radius: 5));
        Assert.Equal([Key(0), Key(1), Key(2), Key(3)], ground.Laid.OrderBy(it => it.X));

        // Nothing has changed, so nothing is laid again.
        Assert.Equal(0, ground.Sync(x: 20, z: 4, radius: 5));

        // Region 0 now ends 28 back, region 2 has gone from the terrain, and
        // region 5 starts 4 metres on.
        terrain.RemoveRegion(Key(2));
        ground.Sync(x: 36, z: 4, radius: 5);
        Assert.Equal([Key(3), Key(4), Key(5)], ground.Laid.OrderBy(it => it.X));
        Assert.False(World.Alive(TerrainPhysics.IdOf(Key(0))));
        Assert.False(World.Alive(TerrainPhysics.IdOf(Key(2))));
        Assert.Equal(TerrainPhysics.IdOf(Key(3)), ground.BodyOf(Key(3)));
        Assert.Equal(Key(4), ground.RegionOf(TerrainPhysics.IdOf(Key(4))));
        Assert.Null(ground.RegionOf(TerrainPhysics.IdOf(Key(0))));

        ground.Clear();
        Assert.Empty(ground.Laid);
        Assert.Equal(0, World.Count);

        Assert.Throws<ArgumentOutOfRangeException>(() => ground.Sync(x: double.NaN, z: 0, radius: 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => ground.Sync(x: 0, z: 0, radius: -1));
    }

    [Fact]
    public void AnEditLaysTheRegionAgainAndTheNeighboursThatSeeIt()
    {
        var terrain = new TerrainData(regionSize: 8);
        var here = terrain.AddRegion(new RegionKey(0, 0));
        var next = terrain.AddRegion(new RegionKey(1, 0));
        terrain.AddRegion(new RegionKey(3, 0));
        var ground = new TerrainPhysics(World, terrain);
        Assert.Equal(2, ground.Sync(x: 8, z: 4, radius: 1));

        World.Add(2, Shape.Box(0.3, 0.3, 0.3), at: [4.0, 1.0, 4.0]);
        Run(3);
        Assert.True(World.Asleep(2));

        // Raised under a sleeping crate: it is woken and pushed up with it.
        terrain.FillHeights(new RegionKey(0, 0), (_, _) => 1.0);
        Assert.Equal(2, ground.Sync(x: 8, z: 4, radius: 1));
        Run(1);
        Assert.Equal(1.3, HeightOf(2), 0.03);

        // Region 3 is two away from region 1, so a change to it reaches no one.
        terrain.RegionAt(new RegionKey(3, 0))!.SetHeight(0, 0, 5);
        Assert.Equal(0, ground.Sync(x: 8, z: 4, radius: 1));

        // Held off while a stroke is drawn, laid once it is done.
        next.SetHeight(0, 4, 2.0);
        here.SetHeight(4, 4, 1.5);
        Assert.Equal(0, ground.Sync(x: 8, z: 4, radius: 1, refresh: false));
        // x = 8 is region 1's first texel, and region 1 has not been raised.
        Assert.Equal(0.0, CastAt(8.0, 4.0)!.Value, 1e-4);
        Assert.Equal(2, ground.Sync(x: 8, z: 4, radius: 1));
        Assert.Equal(2.0, CastAt(8.0, 4.0)!.Value, 1e-4);
    }

    [Fact]
    public void BodiesAreNumberedWhereNoOneElseNumbersThem()
    {
        var seen = new HashSet<ulong>();
        for (var x = -3; x <= 3; x++)
        {
            for (var z = -3; z <= 3; z++)
            {
                var id = TerrainPhysics.IdOf(new RegionKey(x, z));
                // Well clear of -1, which is where a game's player often is.
                Assert.True(unchecked((long)id) < -(1L << 61));
                seen.Add(id);
            }
        }

        Assert.Equal(49, seen.Count);
        foreach (var corner in new[]
        {
            new RegionKey(-(1L << 27), -(1L << 27)), new RegionKey((1L << 27) - 1, (1L << 27) - 1),
        })
        {
            Assert.True(unchecked((long)TerrainPhysics.IdOf(corner)) < -(1L << 61));
        }

        var terrain = new TerrainData(regionSize: 4);
        terrain.AddRegion(new RegionKey(0, 0));
        var ground = new TerrainPhysics(World, terrain, numbering: key => (ulong)(100 + key.X));
        ground.Sync(x: 0, z: 0, radius: 1);
        Assert.True(World.Alive(100));
        Assert.Equal(new RegionKey(0, 0), ground.RegionOf(100));
    }
}
