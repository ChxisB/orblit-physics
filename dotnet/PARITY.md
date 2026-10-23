# Parity

One row per part of the port. A row is added when a part is built.

Declared counts the test declarations in the source. Run counts the tests that
execute, because a loop or a theory runs one declaration many times. A package
is done when the two Run counts match.

| Package | Dart declared | Dart run | C# declared | C# run | Fixtures | Added in C# | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `orblit_physics` part PH0 | 0 | 0 | 0 | 0 | none | `NativeSettingsAddedTests` (3): the size of the settings struct, the defaults the solver fills in, and a world made and destroyed | built. Part PH0: the `dotnet/` tree, the CMake build of the solver, the local feed and `Orblit.Physics` 0.17.0 loading the library. No ported tests. Only macOS ran |
| `orblit_physics` part PH1 | 25 | 25 | 25 | 25 | none | `PhysicsAddedTests` (8 declared, 13 run): the sizes of six native structs, a read output too short for its last row, a stride too short, a full command buffer, a character added in the last slot, a disposed world, and the layer words of a cast and of ground | built. Part PH1: bodies, the character, ground, reads and one cast. Only macOS ran. One mutant stays alive: a command buffer filled one slot late writes past its end and shows nowhere |
