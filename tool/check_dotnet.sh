#!/bin/bash
# Runs the solver's C++ checks, then builds, tests and format-checks
# everything under dotnet/.
#
# The analyzer rules from the standard's C# section are build errors, so the
# build is the lint. A build that fails stops the run, because a test over a
# stale binary says nothing. The native libraries are built first, because the
# test projects copy them next to the test assembly.
set -uo pipefail
cd "$(dirname "$0")/../dotnet"

failures=0

step() {
  local label=$1
  shift
  if "$@"; then
    echo "  ok    $label"
  else
    echo "  FAIL  $label"
    failures=$((failures + 1))
    return 1
  fi
}

# The C++ checks use clang++ and a Unix temp folder. A Windows runner builds
# the solver through CMake instead, so the checks wait for a Unix machine.
case "$(uname -s)" in
  MINGW* | MSYS* | CYGWIN*) ;;
  *) step "native check" ../tool/check_native.sh || exit "$failures" ;;
esac
step "native build" ../tool/build_native.sh || exit "$failures"
step "dotnet build" dotnet build --nologo || exit "$failures"
step "dotnet test" dotnet test --no-build
step "dotnet format" dotnet format --verify-no-changes --no-restore

exit "$failures"
