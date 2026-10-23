#!/bin/bash
# Builds the solver the .NET package loads and puts it in
# dotnet/native/runtimes/<rid>/native, where the project files pick them up.
#
# Only the desktop runtime ids are known. iOS, Android and web come later.
set -euo pipefail
cd "$(dirname "$0")/.."
root=$PWD

case "$(uname -s)" in
  Darwin) os=osx ;;
  Linux) os=linux ;;
  MINGW* | MSYS* | CYGWIN*) os=win ;;
  *) echo "Unknown system $(uname -s)." >&2; exit 1 ;;
esac
case "$(uname -m)" in
  arm64 | aarch64) arch=arm64 ;;
  x86_64 | AMD64) arch=x64 ;;
  *) echo "Unknown machine $(uname -m)." >&2; exit 1 ;;
esac
rid=$os-$arch

build() {
  local package=$1
  local dir="$root/dotnet/native/build/$package"
  cmake -S "$root/packages/$package" -B "$dir" -DORBLIT_RID="$rid" -DCMAKE_BUILD_TYPE=Release > /dev/null
  cmake --build "$dir" --config Release > /dev/null
  cmake --install "$dir" --config Release --prefix "$root/dotnet/native" > /dev/null
  echo "  ok    $package -> runtimes/$rid/native"
}

build orblit_physics
