# SemanticsManager graph locking: the per-map graph lock helpers, exactly this
# subpackage's translation units.
set(VS_GRAPHS_SEMANTICSMANAGER_GRAPH_LOCKING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/copySceneGraphs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/lockAllGraphs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/lockGraphs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/lockMapGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/acquireGraphLocks.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/getHeldGraphLocks.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/releaseGraphLockEntries.cc
)
