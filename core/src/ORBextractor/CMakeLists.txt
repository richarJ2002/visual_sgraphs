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
  core/src/ORBextractor/private_functions/icAngle.cc
  core/src/ORBextractor/private_functions/computeOrbDescriptor.cc
  core/src/ORBextractor/private_functions/computeOrientation.cc
  core/src/ORBextractor/private_functions/compareNodes.cc
  core/src/ORBextractor/private_functions/computeDescriptors.cc
)
