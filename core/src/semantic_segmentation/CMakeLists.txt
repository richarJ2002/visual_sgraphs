# SemanticSegmentation object sources: exactly this object's translation units.
set(VS_GRAPHS_SEMANTICSEGMENTATION_SOURCES
  core/src/semantic_segmentation/methods/addSegmentedFrameToBuffer.cc
  core/src/semantic_segmentation/methods/checkFinish.cc
  core/src/semantic_segmentation/methods/getPlanesFromClassClouds.cc
  core/src/semantic_segmentation/methods/getProcessingStats.cc
  core/src/semantic_segmentation/methods/isFinished.cc
  core/src/semantic_segmentation/methods/recordTerminalOutcome.cc
  core/src/semantic_segmentation/methods/requestFinish.cc
  core/src/semantic_segmentation/methods/run.cc
  core/src/semantic_segmentation/methods/setFinish.cc
  core/src/semantic_segmentation/methods/threshSeparatePointCloud.cc
  core/src/semantic_segmentation/methods/updatePlaneData.cc
  core/src/semantic_segmentation/methods/updatePlaneSemantics.cc
  core/src/semantic_segmentation/private_functions/findLargestWallComponent.cc
)
