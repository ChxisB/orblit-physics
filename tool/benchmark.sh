#!/bin/bash
# Builds and runs the solver benchmark, without Dart in the way.
#
# Not part of check.sh: the numbers belong to the machine, so nothing here can
# fail a build. Run it before and after a change to the solver, on the same
# machine, and compare.
set -euo pipefail
cd "$(dirname "$0")/../packages/orblit_physics"

sources=()
for file in src/*.cpp; do
  case "$file" in
    *_test.cpp) ;;
    *) sources+=("$file") ;;
  esac
done

out="${TMPDIR:-/tmp}/orblit_physics_benchmark"
clang++ -std=c++17 -O2 -Iinclude -Isrc "${sources[@]}" bench/benchmark.cpp -o "$out"

echo "Machine: $(sysctl -n machdep.cpu.brand_string 2>/dev/null || uname -m)"
echo "Compiler: $(clang++ --version | head -1)"
echo
"$out" "$@"
