using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftSurfaceAddedTests : IDisposable
{
    private const ulong Floor = 1;
    private const ulong Gale = 7;
    private const ulong Draught = 8;

    private readonly Physics _physics = new();

    public void Dispose()
    {
        _physics.Dispose();
        GC.SuppressFinalize(this);
    }

    private PhysicsSurface FloorSurface(Layers? floorLayers = null, Layers? layers = null, ulong ignore = 0)
    {
        _physics.Add(Floor, Shape.Box(5, 1, 5), PhysicsMotion.Fixed, at: [0, -1, 0], layers: floorLayers);
        _physics.Step(1.0 / 60);
        return new PhysicsSurface(_physics, layers, ignore);
    }

    private static SoftContact? Drop(PhysicsSurface surface) =>
        surface.Sweep(new Vector3(0, 1, 0), new Vector3(0, -1, 0), 2, 0.01);

    [Fact]
    public void AFloorAnswersTheSweepWithItsFaceAndWhichWayItLooks()
    {
        var hit = Drop(FloorSurface());
        Assert.NotNull(hit);
        Assert.Equal(0, hit.Normal.X, 6);
        Assert.Equal(1, hit.Normal.Y, 6);
        Assert.Equal(0, hit.Normal.Z, 6);
        Assert.Equal(0, hit.At.Y, 2);
    }

    [Fact]
    public void ASweepThatStopsShortOfTheFloorMeetsNothing()
    {
        var surface = FloorSurface();
        Assert.Null(surface.Sweep(new Vector3(0, 1, 0), new Vector3(0, -1, 0), 0.5, 0.01));
        Assert.Null(surface.Sweep(new Vector3(0, 1, 0), new Vector3(0, 1, 0), 2, 0.01));
    }

    [Fact]
    public void ABodyOnTheIgnoreListIsPassedThrough()
    {
        var surface = FloorSurface(ignore: Floor);
        Assert.Equal(Floor, surface.Ignore);
        Assert.Null(Drop(surface));
    }

    [Fact]
    public void ALayerTheSurfaceDoesNotCareAboutIsPassedThrough()
    {
        var surface = FloorSurface(new Layers(Is: 2, Cares: 0), new Layers(Is: 1, Cares: 4));
        Assert.Null(Drop(surface));
        Assert.Equal(4u, surface.Layers.Cares);
    }

    [Fact]
    public void ALayerTheSurfaceCaresAboutIsMet()
    {
        Assert.NotNull(Drop(FloorSurface(new Layers(Is: 2, Cares: 0), new Layers(Is: 1, Cares: 2))));
    }

    [Fact]
    public void ASurfaceBuiltWithoutLayersMeetsEverything()
    {
        var surface = FloorSurface();
        Assert.Equal(Layers.Everything, surface.Layers);
        Assert.Equal(0UL, surface.Ignore);
        Assert.Same(_physics, surface.Physics);
    }

    [Fact]
    public void ATriggerIsNeverMet()
    {
        _physics.Add(Gale, Shape.Box(5, 1, 5), PhysicsMotion.Fixed, at: [0, -1, 0], trigger: true);
        _physics.Step(1.0 / 60);
        Assert.Null(Drop(new PhysicsSurface(_physics)));
    }

    [Fact]
    public void WindIsTheSumOfTheZonesAPointIsIn()
    {
        _physics.Add(Gale, Shape.Box(2, 2, 2), PhysicsMotion.Fixed, at: [0, 0, 0], trigger: true);
        _physics.Add(Draught, Shape.Box(2, 2, 2), PhysicsMotion.Fixed, at: [0, 0, 0], trigger: true);
        _physics.Step(1.0 / 60);
        var zones = new Dictionary<ulong, Vector3> { [Gale] = new(1, 2, 3), [Draught] = new(10, 20, 30) };
        var wind = new PhysicsWind(_physics, zones).WindAt(Vector3.Zero());
        Assert.Equal(11, wind.X);
        Assert.Equal(22, wind.Y);
        Assert.Equal(33, wind.Z);
    }

    [Fact]
    public void ATriggerThatIsNotInTheMapAndAPointOutsideItBlowNothing()
    {
        _physics.Add(Gale, Shape.Box(2, 2, 2), PhysicsMotion.Fixed, at: [0, 0, 0], trigger: true);
        _physics.Step(1.0 / 60);
        var other = new PhysicsWind(_physics, new Dictionary<ulong, Vector3> { [Draught] = new(5, 5, 5) });
        Assert.Equal(0, other.WindAt(Vector3.Zero()).Length);
        var far = new PhysicsWind(_physics, new Dictionary<ulong, Vector3> { [Gale] = new(5, 5, 5) });
        Assert.Equal(0, far.WindAt(new Vector3(50, 0, 0)).Length);
        Assert.Equal(5 * Math.Sqrt(3), far.WindAt(Vector3.Zero()).Length, 9);
    }

    [Fact]
    public void ABodyPointTurnsWithTheBodyAndIsACopyOfTheOffset()
    {
        _physics.Add(3, Shape.Box(1, 1, 1), PhysicsMotion.Fixed, at: [10, 0, 0]);
        _physics.Step(1.0 / 60);
        var offset = new Vector3(1, 0, 0);
        var point = BodyPoint.Of(_physics, 3, offset);
        offset.X = 99;
        var first = point();
        Assert.Equal(11, first.X, 6);
        first.X = 0;
        Assert.Equal(11, point().X, 6);
    }

    [Fact]
    public void ABodyPointToABodyThatIsGoneThrows()
    {
        var point = BodyPoint.Of(_physics, 3, Vector3.Zero());
        Assert.Throws<InvalidOperationException>(() => point());
    }
}
