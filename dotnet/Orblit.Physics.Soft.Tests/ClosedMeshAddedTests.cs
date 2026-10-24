namespace Orblit.Physics.Soft.Tests;

public class ClosedMeshAddedTests : SoftTestBase
{
    [Fact]
    public void EdgesComeOutLowerIndexFirstInTheOrderTheyAreMet()
    {
        var mesh = new ClosedMesh(4, TetrahedronTriangles());
        (int, int)[] expected = [(0, 2), (0, 1), (1, 3), (0, 3), (1, 2), (2, 3)];
        Assert.Equal(expected, mesh.Edges.Select(e => (e.Low, e.High)).ToArray());
    }

    [Theory]
    [InlineData(4, -1)]
    [InlineData(-1, 4)]
    public void TheFirstCornerOutsideTheParticlesInReadingOrderIsTheOneNamed(int first, int second)
    {
        var triangles = TetrahedronTriangles();
        triangles[0] = first;
        triangles[1] = second;
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => new ClosedMesh(4, triangles));
        Assert.Equal((object)first, error.ActualValue);
    }
}
