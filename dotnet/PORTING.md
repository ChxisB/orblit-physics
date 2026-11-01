# Porting Dart to C#

The rules are the engine's `dotnet/PORTING.md`, in the `orblit` repository. A
port keeps the Dart shape and names, and a port is done when `PARITY.md` shows
the two test counts equal and the fixture test is green.

The port is finished and the Dart is deleted. The tag `dart-final` holds the
last Dart commit. This file stays as the record of how the port was done.

This repository differs in three ways.

- The solver is C++ built by `packages/orblit_physics/CMakeLists.txt`. The
  original was built by a Dart hook from the same sources.
- The engine packages come from a folder. The engine's `tool/pack_local.sh`
  packs them, into `~/.orblit/feed` unless `ORBLIT_FEED` names another.
  `Directory.Packages.props` pins the versions.
- This repository's `tool/pack_local.sh` packs the physics packages into the
  same folder, so a repo such as `orblit-site` restores them too. Run
  `tool/build_native.sh` first, or `Orblit.Physics` ships without the solver.
- A part declares only the native calls it makes.
