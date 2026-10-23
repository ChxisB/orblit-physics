# Parity

One row per part of the port. A row is added when a part is built.

Declared counts the test declarations in the source. Run counts the tests that
execute, because a loop or a theory runs one declaration many times. A package
is done when the two Run counts match.

| Package | Dart declared | Dart run | C# declared | C# run | Fixtures | Added in C# | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `orblit_physics` part PH0 | 0 | 0 | 0 | 0 | none | `NativeSettingsAddedTests` (3): the size of the settings struct, the defaults the solver fills in, and a world made and destroyed | built. Part PH0: the `dotnet/` tree, the CMake build of the solver, the local feed and `Orblit.Physics` 0.17.0 loading the library. No ported tests. Only macOS ran |
| `orblit_physics` part PH1 | 25 | 25 | 25 | 25 | none | `PhysicsAddedTests` (8 declared, 13 run): the sizes of six native structs, a read output too short for its last row, a stride too short, a full command buffer, a character added in the last slot, a disposed world, and the layer words of a cast and of ground | built. Part PH1: bodies, the character, ground, reads and one cast. Only macOS ran. One mutant stays alive: a command buffer filled one slot late writes past its end and shows nowhere |
| `orblit_physics` part PH2 | 31 | 31 | 31 | 31 | none | `PhysicsJointAddedTests` (18 declared, 27 run): which way each factory puts its limit, what each keeps, a locked limit, the numbers a spring, a soft limit and a goal refuse, a cone's widest swing, a break by torque, the collide flag, `Aim` leaving a way alone, a path that follows its line, an undamped spring, a soft limit's damping, a distance joint with no second point, a joint lost with its body, and the arguments `Aim` and a path refuse | built. Part PH2: joints. Only macOS ran. 42 hand made mutants: 39 killed, 1 does not build, 2 alive. Alive: a spring on a way of a distance joint other than its length rests at 0 and the engine barely reads it (3 mm in a metre swing), and an empty path gets one float of room that the engine refuses either way. Two more lived the first run and tests now kill them: the soft limit ratio and the send before `Unjoin`. |
