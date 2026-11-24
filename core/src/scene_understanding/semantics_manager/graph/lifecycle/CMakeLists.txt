# SemanticsManager graph lifecycle methods (scene graphs, the active map):
# exactly this subpackage's translation units.
set(VS_GRAPHS_SEMANTICSMANAGER_GRAPH_LIFECYCLE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearSemantics.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/eraseSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setActiveMap.cc
)
