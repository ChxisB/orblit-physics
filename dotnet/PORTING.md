# Porting Dart to C#

The rules are the engine's `dotnet/PORTING.md`, in the `orblit` repository. A
port keeps the Dart shape and names, and a port is done when `PARITY.md` shows
the two test counts equal and the fixture test is green.

This repository differs in three ways.

- The solver is C++ built by `packages/orblit_physics/CMakeLists.txt`, not by
  the Dart hook. Both compile the same sources.
- The engine packages come from a folder. The engine's `tool/pack_local.sh`
  packs them, into `~/.orblit/feed` unless `ORBLIT_FEED` names another.
  `Directory.Packages.props` pins the versions.
- A part declares only the native calls it makes.
