# Contributing

Rigid body physics for Orblit.

Talk about anything large first, in an issue or on the
[Discord](https://discord.gg/5DH7HuDUtJ). Small fixes need no ceremony: just
open the pull request. The engine itself lives in
[ChxisB/orblit](https://github.com/ChxisB/orblit), and its
[CONTRIBUTING](https://github.com/ChxisB/orblit/blob/main/CONTRIBUTING.md) has
the longer version of this.

## Checking your work

```sh
./tool/check.sh
```

That runs the C++ checks on their own first, then analyzes and tests the Dart
package. The C++ checks are worth running by themselves while working on the
solver — `./tool/check_native.sh` compiles and runs them in a second or two,
with no build hook and no Dart in the way.

A change to the solver wants a case in
`packages/orblit_physics/src/orblit_physics_test.cpp`, which is where behaviour
is pinned down. The Dart tests exist to prove the boundary works, not to
re-test the maths.

## Licence

This repository is under MPL-2.0. Opening a pull request means you are
offering your change under that same licence, and that you wrote it or
otherwise have the right to contribute it.

There is no CLA to sign and no copyright to assign. You keep the copyright on
what you write. It is simply licensed the same way as the rest of the
repository.
