# SemanticsManager graph merging: moves the rooms, passages and storeys of an
# absorbed map into the surviving map, exactly this subpackage's translation
# units.
set(VS_GRAPHS_SEMANTICSMANAGER_GRAPH_MERGING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/reassociateMergedSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/transferSceneGraphContents.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/collapseMergedFloors.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/collectAbsorbedSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/crossesPassablePassageOpening.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/fuseDuplicateRoomsAfterMerge.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/indexAbsorbedRoomWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/mergeFloorEvidenceAndRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/transferFloors.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/transferPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/transferRooms.cc
)
