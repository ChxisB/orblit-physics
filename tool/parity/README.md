# Parity runs

These scripts print the stepped worlds that the C# tests compare against.
Each one prints raw bits, so two runs agree only when every word is equal.

| Script | Package | Pinned in C# |
| --- | --- | --- |
| `stepped_world.dart` | `orblit_physics` | `Orblit.Physics.Tests/Reference/stepped_world.txt` |
| `stepped_cars.dart` | `orblit_physics_vehicle` | `Orblit.Physics.Vehicle.Tests/Reference/stepped_cars.txt` |
| `stepped_cloth.dart` | `orblit_physics_soft` | `Orblit.Physics.Soft.Tests/Reference/stepped_cloth.txt` |

## Run one

A script is a test file, so the native build hook runs for it. From the
package folder:

```sh
export PATH=$HOME/development/flutter/bin:$PATH
dart pub get
cp ../../tool/parity/stepped_world.dart test/zz_parity_test.dart
dart test test/zz_parity_test.dart | grep -E '^(wall|character|chain) '
rm test/zz_parity_test.dart
```

Replace the pattern with `^car ` for the cars and `^cloth ` for the cloth.
The output is the pinned file. The C# tests read the pinned file, so
regenerate it only when a scene changes, and change the matching C# scene in
the same commit.

## What a match means

The C++ solver is built twice: by CMake for C# and by the Dart build hook
for Dart. Equal bits show the two builds agree on the machine that made the
file. They say nothing about Windows or Linux.
