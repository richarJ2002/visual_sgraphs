# MapDrawer object sources: exactly this object's translation units.
set(VS_GRAPHS_MAPDRAWER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/drawCurrentCamera.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/drawKeyFrames.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/drawMapPoints.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getCurrentOpenGLCameraMatrix.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/newParameterLoader.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/parseViewerParamFile.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setCurrentCameraPose.cc
)
