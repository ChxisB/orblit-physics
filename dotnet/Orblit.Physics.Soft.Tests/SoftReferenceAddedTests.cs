using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

/// <summary>
/// Whole runs checked against numbers taken from an independent run of the
/// original solver, for the parts that no short argument can predict.
/// </summary>
public class SoftReferenceAddedTests : SoftTestBase
{
    private static readonly double[] SquashedBalloon =
    [
        0.19756522420586364, -0.14122525655974585, -0.05282385413484964,
        0.6404174334810662, 0.03933565431155997, 0.8254188745040875,
        0.704110611045688, 0.6162790007122201, -0.46407130393884155,
        -0.5420932687326298, 0.48561060153596103, 0.19147628356959298,
    ];

    private static readonly double[] LiftedCloth =
    [
        -0.08432100732724168, 0.10454871195741695, -0.03414214818385138,
        0.4030246038700837, 0.03867057993984241, -0.1571518894910973,
        0.8900503081040969, -0.07922497313059172, -0.10016864657347455,
        0.012407376936258053, 0.5892477613326991, 0.06453703867744467,
        0.5017674856052304, 0.497127468406617, 0.0000664126433435854,
        0.9908990684671863, 0.41562332026529036, -0.043400671041692915,
        0.11173433034803305, 1.0826737945594926, 0.015930208613488095,
        0.5955746798210132, 0.9846627567026307, 0.10208899656386916,
        1.0788631541753269, 0.8666705799666047, 0.15224069879197197,
    ];

    private static void PinAll(SoftBody body)
    {
        for (var i = 0; i < body.Count; i++)
        {
            body.Pin(i);
        }
    }

    private static void ReleaseAll(SoftBody body)
    {
        for (var i = 0; i < body.Count; i++)
        {
            body.Release(i);
        }
    }

    private static void SamePoints(SoftBody body, double[] expected)
    {
        for (var i = 0; i < body.Count; i++)
        {
            Near(new Vector3(expected[i * 3], expected[(i * 3) + 1], expected[(i * 3) + 2]), body.PositionOf(i), 9);
        }
    }

    [Fact]
    public void ASquashedBalloonWithGiveSpringsBackAsTheOriginalSolverDoes()
    {
        var spec = new SoftSpec(gravity: Vector3.Zero(), iterations: 3);
        var balloon = SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles(), volumeCompliance: 1e-5, spec: spec);
        PinAll(balloon);
        balloon.Place(3, new Vector3(0, 0, 0.5));
        ReleaseAll(balloon);

        Run(balloon, 3 * Dt);
        SamePoints(balloon, SquashedBalloon);
        Assert.Equal(0.16660469090734317, balloon.Volume!.Value, 9);
    }

    [Fact]
    public void ALiftedClothSettlesAsTheOriginalSolverDoes()
    {
        var cloth = SoftBody.Cloth(
            3,
            3,
            Vector3.Zero(),
            new Vector3(0.5, 0, 0),
            new Vector3(0, 0.5, 0),
            spec: new SoftSpec(gravity: Vector3.Zero()));
        PinAll(cloth);
        cloth.Place(1, new Vector3(0.5, 0, 0.3));
        cloth.Place(3, new Vector3(0, 0.5, -0.3));
        ReleaseAll(cloth);

        Run(cloth, 5 * Dt);
        SamePoints(cloth, LiftedCloth);
    }
}
