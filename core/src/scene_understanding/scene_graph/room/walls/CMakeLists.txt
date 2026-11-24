# room object, walls subpackage: exactly this subpackage's translation units.
set(VS_GRAPHS_SCENEGRAPH_ROOM_WALLS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getCeilingPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getGroundPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getGroundPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getIsPassageFaceRefused.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getIsSeparatedFrom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getWallNormalTowardRoom_world.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/hasWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/removeInvalidWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/removeWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/replaceGroundPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/replaceWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setGroundPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setWalls.cc
)
