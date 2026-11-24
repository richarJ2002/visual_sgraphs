# Association cycle: the SemanticsManager methods that run the room association
# stage once per cycle, retire orphaned prospective rooms and queue the key
# frames of deleted rooms for erasure; exactly this subpackage's translation
# units.
set(VS_GRAPHS_ROOMASSOCIATION_ASSOCIATIONCYCLE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/requestRoomKeyFrameErasure.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/retireOrphanedProspectiveRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/runRoomAssociation.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/takeKeyFrameErasureRequests.cc
)
