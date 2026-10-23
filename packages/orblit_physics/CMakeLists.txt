# Builds the solver for the .NET packages. The Dart build uses hook/build.dart
# and does not read this file. Both compile the same sources.
#
# Usage: cmake -S packages/orblit_physics -B <build dir> -DORBLIT_RID=osx-arm64
#        cmake --build <build dir> --config Release
#        cmake --install <build dir> --config Release --prefix dotnet/native
cmake_minimum_required(VERSION 3.20)
project(orblit_physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The header has no export macro. MSVC exports nothing without one.
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(ORBLIT_RID "" CACHE STRING "Runtime id, for example osx-arm64, win-x64 or linux-x64")
if(ORBLIT_RID STREQUAL "")
  message(FATAL_ERROR "Pass -DORBLIT_RID=<runtime id>.")
endif()

add_library(orblit_physics SHARED
  src/orblit_physics.cpp
  src/world.cpp
  src/collide.cpp
  src/cast.cpp
  src/solver.cpp
  src/joint.cpp
  src/character.cpp
  src/heightfield.cpp
  src/inspect.cpp
  src/hull.cpp
  src/compound.cpp
  src/bounds_tree.cpp
  src/triangle_mesh.cpp
  src/mesh_world.cpp
  src/compound_world.cpp
  src/convex.cpp
  src/gjk.cpp
  src/convex_collide.cpp)
target_include_directories(orblit_physics PUBLIC include PRIVATE src)

install(TARGETS orblit_physics
  LIBRARY DESTINATION runtimes/${ORBLIT_RID}/native
  RUNTIME DESTINATION runtimes/${ORBLIT_RID}/native)
