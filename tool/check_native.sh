#!/bin/bash
# Compiles and runs the solver's own C++ checks, without Dart in the way.
set -euo pipefail
cd "$(dirname "$0")/../packages/orblit_physics"
clang++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Isrc \
  src/orblit_physics.cpp src/world.cpp src/collide.cpp src/solver.cpp \
  src/orblit_physics_test.cpp -o /tmp/orblit_physics_check
/tmp/orblit_physics_check
