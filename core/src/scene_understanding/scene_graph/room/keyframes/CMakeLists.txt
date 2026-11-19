# room object, keyframes subpackage: exactly this subpackage's translation
# units.
set(VS_GRAPHS_SCENEGRAPH_ROOM_KEYFRAMES_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/linkKeyFrame.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/transferKeyFrames.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/collectObserverCameraCenters.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/collectObserverKeyFrames.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/findObserverRoomId.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/selectMajorityRoomId.cc
)
