# Room object sources: split into subpackages, each declaring exactly its
# own translation units.
include(${CMAKE_CURRENT_LIST_DIR}/boundary/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/keyframes/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/passages/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/state/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/walls/CMakeLists.txt)

set(VS_GRAPHS_SCENEGRAPH_ROOM_SOURCES
  ${VS_GRAPHS_SCENEGRAPH_ROOM_BOUNDARY_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_ROOM_KEYFRAMES_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_ROOM_PASSAGES_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_ROOM_STATE_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_ROOM_WALLS_SOURCES}
)
