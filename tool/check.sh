#!/bin/bash
# Analyzes and tests everything in the physics repository.
set -uo pipefail
cd "$(dirname "$0")/.."

PACKAGES=(
  packages/orblit_physics
  packages/orblit_physics_scene
)
failures=0

echo "== native =="
if ./tool/check_native.sh > /tmp/orblit_native.log 2>&1; then
  echo "  ok    $(tail -1 /tmp/orblit_native.log)"
else
  echo "  FAIL  native checks"; tail -25 /tmp/orblit_native.log; failures=$((failures+1))
fi

for package in "${PACKAGES[@]}"; do
  echo "== $package =="
  (cd "$package" && dart pub get > /dev/null 2>&1)

  if (cd "$package" && dart analyze > /tmp/orblit_analyze.log 2>&1); then
    echo "  ok    analyze"
  else
    echo "  FAIL  analyze"; tail -20 /tmp/orblit_analyze.log; failures=$((failures+1))
  fi

  if (cd "$package" && dart test > /tmp/orblit_test.log 2>&1); then
    summary=$(tr '\r' '\n' < /tmp/orblit_test.log | tail -1 \
      | sed -e 's/\x1b\[[0-9;]*m//g' -e 's/^[0-9:]* //')
    echo "  ok    $summary"
  else
    echo "  FAIL  tests"; tail -25 /tmp/orblit_test.log; failures=$((failures+1))
  fi
done

echo
[ "$failures" -eq 0 ] && echo "everything green" || echo "$failures failing step(s)"
exit "$failures"
