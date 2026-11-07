# SceneGraph object package: the class header and the helpers shared by its
# methods live here; the methods are split into the lifecycle, mutations and
# queries subpackages, each declaring exactly its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/lifecycle/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/mutations/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/queries/CMakeLists.txt)

set(VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/findFloorInBuildings.cc
  ${VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_LIFECYCLE_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_MUTATIONS_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_QUERIES_SOURCES}
)
