#!/bin/bash
# Compiles and runs the solver's own C++ checks, without Dart in the way.
set -euo pipefail
cd "$(dirname "$0")/../packages/orblit_physics"
# Every source there is, the test file among them — it is the one with a main.
# A glob rather than a list because a list is a thing to forget to add to, and
# forgetting shows up as a linker error nobody expects from adding a file.
clang++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Isrc \
  src/*.cpp -o /tmp/orblit_physics_check
/tmp/orblit_physics_check
