# Scene graph object packages: each object subdirectory declares exactly its
# own sources; this file aggregates them into the module list consumed by the
# root CMakeLists.txt. SceneMap and Building are data-only and have no sources.
include(${CMAKE_CURRENT_LIST_DIR}/room/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/floor/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/passage/CMakeLists.txt)

set(VS_GRAPHS_SCENEGRAPH_SOURCES
  ${VS_GRAPHS_SCENEGRAPH_ROOM_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_FLOOR_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_PASSAGE_SOURCES}
)
