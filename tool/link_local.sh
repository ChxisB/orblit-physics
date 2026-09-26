#!/bin/bash
# Points the bridges — orblit_physics_scene and orblit_physics_terrain — at a
# sibling engine checkout instead of the git dependency, so a change to the
# scene or terrain format can be tried here without pushing it. The overrides
# are not committed, so nobody ships a build wired to a local path.
# orblit_physics itself depends on nothing and needs no linking.
set -euo pipefail
cd "$(dirname "$0")/.."

ENGINE=${1:-../orblit}
SCENE="$ENGINE/packages/orblit_scene"
TERRAIN="$ENGINE/packages/orblit_terrain"

if [ ! -f "$SCENE/pubspec.yaml" ]; then
  echo "No engine checkout at $ENGINE."
  echo "Clone ChxisB/orblit beside this one, or pass its path."
  exit 1
fi

cat > packages/orblit_physics_scene/pubspec_overrides.yaml <<YAML
# Written by tool/link_local.sh. Not committed.
dependency_overrides:
  orblit_scene:
    path: $(cd "$SCENE" && pwd)
YAML

cat > packages/orblit_physics_terrain/pubspec_overrides.yaml <<YAML
# Written by tool/link_local.sh. Not committed.
dependency_overrides:
  orblit_terrain:
    path: $(cd "$TERRAIN" && pwd)
YAML

(cd packages/orblit_physics_scene && dart pub get > /dev/null)
echo "orblit_physics_scene -> $(cd "$SCENE" && pwd)"
(cd packages/orblit_physics_terrain && dart pub get > /dev/null)
echo "orblit_physics_terrain -> $(cd "$TERRAIN" && pwd)"
