# ORBextractor module sources: single ORBextractor object (§4.5) plus
# module-internal helpers (formerly file-static, see private_functions.h).
# No source file is listed twice.
set(VS_GRAPHS_ORBEXTRACTOR_SOURCES
  core/src/ORBextractor/methods/construct.cc
  core/src/ORBextractor/methods/ExtractorNode/divideNode.cc
  core/src/ORBextractor/methods/distributeOctTree.cc
  core/src/ORBextractor/methods/computeKeyPointsOctTree.cc
  core/src/ORBextractor/methods/computeKeyPointsOld.cc
  core/src/ORBextractor/operators/call/withImage.cc
  core/src/ORBextractor/methods/computePyramid.cc
  core/src/ORBextractor/icAngle.cc
  core/src/ORBextractor/computeOrbDescriptor.cc
  core/src/ORBextractor/computeOrientation.cc
  core/src/ORBextractor/compareNodes.cc
  core/src/ORBextractor/computeDescriptors.cc
)
