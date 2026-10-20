# geo_sem_helpers module sources.
set(VS_GRAPHS_UTILS_GEOSEMHELPERS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateGroundPlaneToRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkIfMarkerIsDoorway.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/countGroundPlanePointsWithinWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createBlankRoomCandidate.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createMapFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createMapMarker.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createMapPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createMapPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/markerSemanticAnalysis.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refitMappedPlaneFromCloud.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updateMapPlane.cc
)
