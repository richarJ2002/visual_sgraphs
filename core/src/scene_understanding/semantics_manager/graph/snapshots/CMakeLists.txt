# SemanticsManager graph snapshots: plain-data copies of a map's scene graph,
# exactly this subpackage's translation units.
set(VS_GRAPHS_SEMANTICSMANAGER_GRAPH_SNAPSHOTS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/findPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getActiveSceneGraphSnapshot.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getSceneGraphSnapshot.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/captureFloorEntry.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/capturePassageEntry.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/captureRoomEntry.cc
)
