# Wall admission: SemanticsManager methods that offer unowned walls to the
# current room nearest-first and gate each admission (passage aperture
# backstop, admission evidence, face side, visibility rules, boundary
# topology); exactly this subpackage's translation units.
set(VS_GRAPHS_ROOMASSOCIATION_WALLADMISSION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/admitWallToRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateAllWallsToRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/attachAdmittedWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkWallBoundaryTopology.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/enforcePassageApertureBackstop.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getRoomViewpoints.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isWallFaceForeignToRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/sortWallsByCameraDistance.cc
)
