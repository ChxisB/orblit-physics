using Orblit.Scene;

namespace Orblit.Physics.Scene.Tests;

/// <summary>Owns the scenes a test makes and lets them go when the test ends.</summary>
public abstract class SceneTestBase : IDisposable
{
    private readonly List<ScenePhysics> _made = [];

    public void Dispose()
    {
        foreach (var scene in _made)
        {
            scene.Dispose();
        }

        GC.SuppressFinalize(this);
    }

    // F2: the constructor's options, passed by name.
    protected ScenePhysics Make(
        SceneDocument document,
        double step = 1.0 / 60,
        int maxSteps = 8,
        bool smooth = false,
        PhysicsSettings? settings = null)
    {
        var scene = new ScenePhysics(document, settings, step, maxSteps, smooth);
        _made.Add(scene);
        return scene;
    }
}
