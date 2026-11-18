# Passage object sources: split into subpackages, each declaring exactly its
# own translation units.
include(${CMAKE_CURRENT_LIST_DIR}/geometry/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/known_side/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/state/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/traversal_evidence/CMakeLists.txt)

set(VS_GRAPHS_SCENEGRAPH_PASSAGE_SOURCES
  ${VS_GRAPHS_SCENEGRAPH_PASSAGE_GEOMETRY_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_PASSAGE_KNOWNSIDE_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_PASSAGE_STATE_SOURCES}
  ${VS_GRAPHS_SCENEGRAPH_PASSAGE_TRAVERSALEVIDENCE_SOURCES}
)
