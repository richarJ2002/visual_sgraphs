# Geometric module aggregation: each object subdirectory declares exactly
# its own sources; this file aggregates them into the module lists consumed
# by the root CMakeLists.txt. No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/Methods/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Plane/CMakeLists.txt)

set(VS_GRAPHS_GEOMETRIC_GEOMETRIC_SOURCES
  ${VS_GRAPHS_GEOMETRIC_METHODS_SOURCES}
  ${VS_GRAPHS_GEOMETRIC_PLANE_SOURCES}
)
