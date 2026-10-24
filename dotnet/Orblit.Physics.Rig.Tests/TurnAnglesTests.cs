using Orblit.VectorMath;

namespace Orblit.Physics.Rig.Tests;

public sealed class TurnAnglesTests : IDisposable
{
    private readonly Physics _physics = new();

    public TurnAnglesTests()
    {
        _physics.Add(1, Shape.Sphere(0.1), PhysicsMotion.Driven);
        _physics.Add(2, Shape.Sphere(0.1), PhysicsMotion.Driven);
        _physics.Join(1, Joint.SixAxis(), 1, [0.0, 0.0, 0.0], 2);
    }

    public void Dispose() => _physics.Dispose();

    /// <summary>What the world reads for a second body turned to <paramref name="turn"/> against a first that has not moved.</summary>
    private IReadOnlyList<double> Read(Quaternion turn)
    {
        _physics.Place(2, [0.0, 0.0, 0.0], Storage.Of(turn));
        return _physics.JointStateOf(1)!.Angles;
    }

    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    [Fact]
    public void ATwistIsReadAboutX()
    {
        var angles = Turns.Angles(About(1, 0, 0, 0.4));

        Assert.Equal(0.4, angles[0], 1e-9);
        Assert.Equal(0, angles[1], 1e-9);
        Assert.Equal(0, angles[2], 1e-9);
    }

    [Fact]
    public void ASwingIsReadAboutYAndZ()
    {
        Assert.Equal(0.7, Turns.Angles(About(0, 1, 0, 0.7))[1], 1e-9);
        Assert.Equal(-0.5, Turns.Angles(About(0, 0, 1, -0.5))[2], 1e-9);
    }

    [Fact]
    public void AgreesWithTheSolverForASpreadOfTurns()
    {
        var turns = new[]
        {
            About(1, 0, 0, 0.4),
            About(0, 1, 0, 0.7),
            About(0, 0, 1, -0.5),
            About(0, 1, 0, 0.5) * About(1, 0, 0, 0.3),
            About(0, 0, 1, 0.9) * About(1, 0, 0, -0.6),
            About(1, 1, 0, 1.1),
            About(0, 1, 1, -1.4),
            About(1, 2, 3, 0.8),
            About(0, 1, 0, 2.2) * About(1, 0, 0, 0.4),
        };

        foreach (var turn in turns)
        {
            var mine = Turns.Angles(turn);
            var world = Read(turn);
            for (var i = 0; i < 3; i++)
            {
                Assert.Equal(world[i], mine[i], 2e-3);
            }
        }
    }

    [Fact]
    public void TakesTheShorterWayWhenTheQuaternionIsTheLongOne()
    {
        var turn = About(1, 0, 0, 0.4);
        var longWay = new Quaternion(-turn.X, -turn.Y, -turn.Z, -turn.W);

        Assert.Equal(0.4, Turns.Angles(longWay)[0], 1e-9);
    }
}
