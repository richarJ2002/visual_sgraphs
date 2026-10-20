# SemanticSegmentation object sources: exactly this object's translation units.
set(VS_GRAPHS_SEMANTICSEGMENTATION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addSegmentedFrameToBuffer.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getPlanesFromClassClouds.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getProcessingStats.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isFinished.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/recordTerminalOutcome.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/requestFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/run.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/threshSeparatePointCloud.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updatePlaneData.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updatePlaneSemantics.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/findLargestWallComponent.cc
)
