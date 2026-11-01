using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene.Tests;

public class PoseTests
{
    private static Matrix3 TurnOf(Vector3 degrees) => ScenePose.RotationOf(degrees).GetRotation();

    /// <summary>Whether two rotations are the same turn, whatever angles wrote them.</summary>
    private static void AssertSameTurn(Matrix3 expected, Matrix3 actual, double within = 1e-9)
    {
        for (var i = 0; i < 9; i++)
        {
            Assert.InRange(actual[i] - expected[i], -within, within);
        }
    }

    /// <summary>A pose as the world reports it: translation xyz then rotation xyzw.</summary>
    private static float[] PoseOf(Quaternion turn, Vector3? at = null)
    {
        var p = at ?? Vector3.Zero();
        return [(float)p.X, (float)p.Y, (float)p.Z, (float)turn.X, (float)turn.Y, (float)turn.Z, (float)turn.W];
    }

    private static Quaternion AboutY(double angle) =>
        Quaternion.AxisAngle(new Vector3(0, 1, 0), Doubles.Radians(angle));

    /// <summary>Degrees between the rotations of two poses, whichever sign each is written with.</summary>
    private static double Apart(float[] a, float[] b)
    {
        var dot = 0.0;
        for (var i = 3; i < 7; i++)
        {
            dot += (double)a[i] * b[i];
        }

        return Doubles.Degrees(2 * Math.Acos(Math.Clamp(Math.Abs(dot), 0.0, 1.0)));
    }

    private static float[] Blended(float[] from, float[] to, double alpha)
    {
        var output = new float[7];
        ScenePose.BlendPose(from, to, alpha, output);
        return output;
    }

    private static readonly float[] BlendFrom = PoseOf(
        Quaternion.FromRotation(TurnOf(new Vector3(30, 40, 50))),
        new Vector3(1, 2, 3));

    private static readonly float[] BlendTo = PoseOf(
        Quaternion.FromRotation(TurnOf(new Vector3(35, 60, 20))),
        new Vector3(11, -18, 33));

    [Fact]
    public void AnglesAreAnglesForTheSameRotationWhateverItWas()
    {
        var random = new Random(7);
        for (var i = 0; i < 500; i++)
        {
            var angles = new Vector3(
                (random.NextDouble() * 720) - 360,
                (random.NextDouble() * 720) - 360,
                (random.NextDouble() * 720) - 360);
            var turn = TurnOf(angles);

            AssertSameTurn(turn, TurnOf(ScenePose.EulerOf(turn)));
            AssertSameTurn(turn, TurnOf(ScenePose.EulerOf(turn, angles)));
        }
    }

    [Fact]
    public void AnglesAreTheAnglesItWasGivenWhenThoseAreNearby()
    {
        var random = new Random(11);
        for (var i = 0; i < 500; i++)
        {
            var angles = new Vector3(
                (random.NextDouble() * 720) - 360,
                (random.NextDouble() * 170) - 85,
                (random.NextDouble() * 720) - 360);
            var found = ScenePose.EulerOf(TurnOf(angles), angles);

            Assert.Equal(angles.X, found.X, 1e-6);
            Assert.Equal(angles.Y, found.Y, 1e-6);
            Assert.Equal(angles.Z, found.Z, 1e-6);
        }
    }

    [Fact]
    public void AnglesTakeTheOtherWritingWhenThatIsTheOneNearby()
    {
        // (180, 150, 180) is (0, 30, 0) written the other way round.
        var near = new Vector3(179, 151, 181);
        var found = ScenePose.EulerOf(TurnOf(new Vector3(0, 30, 0)), near);

        Assert.Equal(180, found.X, 1e-6);
        Assert.Equal(150, found.Y, 1e-6);
        Assert.Equal(180, found.Z, 1e-6);
    }

    [Fact]
    public void AnglesKeepCountingPastAWholeTurn()
    {
        // A wheel on its fourth revolution says so, rather than starting again.
        var found = ScenePose.EulerOf(TurnOf(new Vector3(0, 0, 1090)), new Vector3(0, 0, 1085));

        Assert.Equal(1090, found.Z, 1e-6);
    }

    [Fact]
    public void TurnedAQuarterAboutYXKeepsTheAngleItHad()
    {
        // Here X and Z turn about the same axis, and a crate turned to face sideways would otherwise
        // show two angles that are only rounding.
        Vector3[] cases = [new(0, 90, 0), new(30, 90, 10), new(0, -90, 45), new(-20, -90, 70)];
        foreach (var angles in cases)
        {
            var turn = TurnOf(angles);

            var alone = ScenePose.EulerOf(turn);
            AssertSameTurn(turn, TurnOf(alone));
            Assert.Equal(0, alone.X);

            var kept = ScenePose.EulerOf(turn, angles);
            AssertSameTurn(turn, TurnOf(kept));
            Assert.Equal(angles.X, kept.X, 1e-9);
            Assert.Equal(angles.Z, kept.Z, 1e-6);
        }
    }

    [Fact]
    public void TurnedAQuarterAndLeftWithASettledTiltKeepTheAngles()
    {
        // A box that landed flat is a few hundred-thousandths of a radian off the angles it was given,
        // which is nothing and must not read as turned about X and back about Z.
        Vector3[] cases = [new(0, 90, 0), new(0, -90, 0), new(0, 270, 0), new(30, 90, 10)];
        foreach (var angles in cases)
        {
            var tilted = TurnOf(angles);
            tilted.Multiply(Matrix3.RotationZ(4e-5));
            tilted.Multiply(Matrix3.RotationX(-3e-6));

            var kept = ScenePose.EulerOf(tilted, angles);
            Assert.Equal(angles.X, kept.X, 1e-9);
            Assert.Equal(angles.Y, kept.Y, 1e-2);
            AssertSameTurn(tilted, TurnOf(kept), 1e-4);

            // With nothing known of the angles, nothing is held back.
            AssertSameTurn(tilted, TurnOf(ScenePose.EulerOf(tilted)));
        }
    }

    [Fact]
    public void JustShortOfAQuarterTurnAboutYAreStillTheRotation()
    {
        foreach (var off in new[] { 1e-3, 1e-5, 1e-7, 1e-9 })
        {
            var angles = new Vector3(40, 90 - off, -25);
            var turn = TurnOf(angles);

            // Without the angles it had, X is let go a millionth short of the quarter turn, and the
            // rotation is right to within that.
            AssertSameTurn(turn, TurnOf(ScenePose.EulerOf(turn)), 1e-6);
            AssertSameTurn(turn, TurnOf(ScenePose.EulerOf(turn, angles)));
        }
    }

    [Fact]
    public void BlendIsTheFirstPoseAt0AndTheSecondAt1()
    {
        var start = Blended(BlendFrom, BlendTo, 0);
        var end = Blended(BlendFrom, BlendTo, 1);

        for (var i = 0; i < 7; i++)
        {
            Assert.Equal(BlendFrom[i], start[i], 1e-6);
            Assert.Equal(BlendTo[i], end[i], 1e-6);
        }
    }

    [Fact]
    public void BlendMovesInAStraightLine()
    {
        var output = Blended(BlendFrom, BlendTo, 0.25);

        Assert.Equal(3.5, output[0], 1e-6);
        Assert.Equal(-3, output[1], 1e-6);
        Assert.Equal(10.5, output[2], 1e-6);
    }

    [Fact]
    public void BlendTurnsHalfwayThroughHalfTheTurn()
    {
        var output = Blended(PoseOf(AboutY(0)), PoseOf(AboutY(90)), 0.5);

        var expected = PoseOf(AboutY(45));
        for (var i = 3; i < 7; i++)
        {
            Assert.Equal(expected[i], output[i], 1e-6);
        }
    }

    [Fact]
    public void BlendTurnsAtAnEvenRate()
    {
        var start = PoseOf(AboutY(0));
        var end = PoseOf(AboutY(90));

        foreach (var alpha in new[] { 0.1, 0.25, 0.5, 0.75, 0.9 })
        {
            var output = Blended(start, end, alpha);

            Assert.Equal(90 * alpha, Apart(output, start), 0.01);
        }
    }

    [Fact]
    public void BlendIsAUnitRotationAllTheWayThrough()
    {
        for (var i = 0; i <= 20; i++)
        {
            var output = Blended(BlendFrom, BlendTo, i / 20.0);

            var length = Math.Sqrt(
                ((double)output[3] * output[3]) + ((double)output[4] * output[4])
                + ((double)output[5] * output[5]) + ((double)output[6] * output[6]));
            Assert.Equal(1, length, 1e-6);
        }
    }

    [Fact]
    public void BlendTakesTheShortWayWhenOneRotationIsWrittenNegated()
    {
        // Q and -Q are the same turn. Written the far way, the blend between them would go nearly all
        // the way round the other side.
        var near = PoseOf(AboutY(10));
        for (var i = 3; i < 7; i++)
        {
            near[i] = -near[i];
        }

        var output = Blended(PoseOf(AboutY(0)), near, 0.5);

        var expected = PoseOf(AboutY(5));
        for (var i = 3; i < 7; i++)
        {
            Assert.Equal(expected[i], output[i], 1e-6);
        }
    }

    [Fact]
    public void BlendCopiesARotationThatHasNotChangedAsItIs()
    {
        // Not a unit rotation, so any arithmetic on it shows. A body at rest has the same turn in both
        // poses and must not read as having moved.
        float[] still = [0, 0, 0, 0.5f, 0.5f, 0.5f, 0.6f];
        float[] moved = [4, 0, 0, 0.5f, 0.5f, 0.5f, 0.6f];

        var output = Blended(still, moved, 0.3);

        Assert.Equal(still[3..], output[3..]);
        Assert.Equal(1.2, output[0], 1e-6);
    }

    [Fact]
    public void BlendGoesStraightAcrossBetweenRotationsTooCloseForAnArc()
    {
        var start = PoseOf(AboutY(0));
        var end = PoseOf(AboutY(0.05));

        var output = Blended(start, end, 0.5);

        Assert.All(output, value => Assert.True(float.IsFinite(value)));
        Assert.Equal((start[4] + end[4]) / 2, output[4], 1e-6);
        Assert.Equal(1, output[6], 1e-6);
    }

    [Fact]
    public void BlendWritesIntoTheSevenFloatsItIsGivenAndNoOthers()
    {
        var all = new float[21];
        Array.Fill(all, 9f);

        ScenePose.BlendPose(BlendFrom, BlendTo, 0.5, all.AsSpan(7, 7));

        Assert.All(all[..7], value => Assert.Equal(9f, value));
        Assert.All(all[14..], value => Assert.Equal(9f, value));
        Assert.Equal(Blended(BlendFrom, BlendTo, 0.5), all[7..14]);
    }

    [Fact]
    public void WhereAnEntityIsIsUnderEveryOneOfItsParents()
    {
        var document = new SceneDocument(
            "Scene",
            entities:
            [
                new SceneEntity(
                    "cart",
                    "Cart",
                    components: new Dictionary<string, SceneComponent>
                    {
                        [SceneComponents.Transform] = new TransformComponent(
                            new Vector3(10, 0, 0),
                            new Vector3(0, 90, 0),
                            Vector3.All(2)),
                    }),
                new SceneEntity(
                    "wheel",
                    "Wheel",
                    "cart",
                    components: new Dictionary<string, SceneComponent>
                    {
                        [SceneComponents.Transform] = new TransformComponent(new Vector3(1, 0, 0)),
                    }),
            ]);

        // A quarter turn about Y takes +X to -Z, and the cart's scale doubles the distance.
        var at = ScenePose.WorldOf(document, "wheel").GetTranslation();
        Assert.Equal(10, at.X, 1e-9);
        Assert.Equal(-2, at.Z, 1e-9);
    }

    [Fact]
    public void AnEntityItDoesNotKnowOrNothingIsWhereTheWorldIs()
    {
        var document = new SceneDocument("Scene", entities: []);

        AssertIdentity(ScenePose.WorldOf(document, "nobody"));
        AssertIdentity(ScenePose.WorldOf(document, null));
    }

    [Fact]
    public void AFolderWithNoTransformMovesNothingInIt()
    {
        var document = new SceneDocument(
            "Scene",
            entities:
            [
                new SceneEntity("folder", "Folder"),
                new SceneEntity(
                    "crate",
                    "Crate",
                    "folder",
                    components: new Dictionary<string, SceneComponent>
                    {
                        [SceneComponents.Transform] = new TransformComponent(new Vector3(1, 2, 3)),
                    }),
            ]);

        Assert.True(new Vector3(1, 2, 3).Equals(ScenePose.WorldOf(document, "crate").GetTranslation()));
    }

    private static void AssertIdentity(Matrix4 matrix)
    {
        var identity = Matrix4.Identity();
        for (var i = 0; i < 16; i++)
        {
            Assert.Equal(identity[i], matrix[i]);
        }
    }
}
