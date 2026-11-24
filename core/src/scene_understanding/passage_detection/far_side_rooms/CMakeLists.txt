# Far-side rooms: the far-side room handle of each passage -- stale handles
# cleared, confirmed far-side rooms linked, prospective rooms created or
# reused, lost handles restored; exactly this subpackage's translation units.
set(VS_GRAPHS_PASSAGEDETECTION_FARSIDEROOMS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearStaleFarSideHandle.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createFarSideRoomHypotheses.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createProspectiveFarSideRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createProspectiveRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findConfirmedFarSideRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findReusableProspectiveRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resolveConfirmedFarSideRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/restoreFarSideHandle.cc
)
