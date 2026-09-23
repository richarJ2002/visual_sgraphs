# SemanticSegmentation object sources: per-method translation units, §4.5
set(VS_GRAPHS_SEMANTICSEGMENTATION_SOURCES
  core/src/SemanticSegmentation/findLargestWallComponent.cc
  core/src/SemanticSegmentation/methods/addSegmentedFrameToBuffer.cc
  core/src/SemanticSegmentation/methods/checkFinish.cc
  core/src/SemanticSegmentation/methods/construct/semanticsegmentation.cc
  core/src/SemanticSegmentation/methods/getPlanesFromClassClouds.cc
  core/src/SemanticSegmentation/methods/getProcessingStats.cc
  core/src/SemanticSegmentation/methods/isFinished.cc
  core/src/SemanticSegmentation/methods/recordTerminalOutcome.cc
  core/src/SemanticSegmentation/methods/requestFinish.cc
  core/src/SemanticSegmentation/methods/run.cc
  core/src/SemanticSegmentation/methods/setFinish.cc
  core/src/SemanticSegmentation/methods/threshSeparatePointCloud.cc
  core/src/SemanticSegmentation/methods/updatePlaneData.cc
  core/src/SemanticSegmentation/methods/updatePlaneSemantics.cc
)
