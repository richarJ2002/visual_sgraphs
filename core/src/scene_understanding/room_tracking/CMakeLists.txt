# Room tracking stage (pipeline stage 6): SemanticsManager methods that record
# passage crossings by the camera path, advance the room-state machine and
# report the current room; exactly this package's translation units.
set(VS_GRAPHS_ROOMTRACKING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/getCurrentRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getCurrentRoomId.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updateRoomTrackerState.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updateTraversalEvidence.cc
)
