# SceneGraph object sources: exactly this object's translation units.
set(VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/getActiveSceneMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getAllFloorsInIdOrder.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getAllPassagesInIdOrder.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getAllRoomsInIdOrder.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getFloorById.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getPassageById.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getRoomById.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getSceneMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/mirrorMapBuilding.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/mirrorMapIndexes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refreshFromAtlas.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/verifyAgainstAtlas.cc
)
