# SemanticsManager bootstrap: creates the first hierarchy of a map (building,
# floor, room); the recovery after a tracking loss is in map_linking/; exactly
# this subpackage's translation units.
set(VS_GRAPHS_SEMANTICSMANAGER_BOOTSTRAP_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/adoptBootstrapRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createBootstrapRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ensureActiveMapBootstrapHierarchy.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ensureBootstrapFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/linkBootstrapRoomToFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resolveBootstrapCameraPosition.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/findFirstLiveRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/findLiveRoomById.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isLiveConfirmedRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/printBootstrapRecord.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/selectBootstrapRoom.cc
)
