# G2oTypes module aggregation: each object subdirectory declares exactly
# its own sources; free SO(3) helpers live directly in this module.
# No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/ImuCamPose/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/InvDepthPoint/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/VertexPose/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeMono/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeMonoOnlyPose/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeStereo/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeStereoOnlyPose/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeInertial/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeInertialGS/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgePriorPoseImu/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgePriorAcc/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgePriorGyro/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/VertexVelocity/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/VertexGyroBias/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/VertexAccBias/CMakeLists.txt)

set(VS_GRAPHS_G2OTYPES_SOURCES
  ${VS_GRAPHS_G2OTYPES_IMUCAMPOSE_SOURCES}
  ${VS_GRAPHS_G2OTYPES_INVDEPTHPOINT_SOURCES}
  ${VS_GRAPHS_G2OTYPES_VERTEXPOSE_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEMONO_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEMONOONLYPOSE_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGESTEREO_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGESTEREOONLYPOSE_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEINERTIAL_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEINERTIALGS_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEPRIORPOSEIMU_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEPRIORACC_SOURCES}
  ${VS_GRAPHS_G2OTYPES_EDGEPRIORGYRO_SOURCES}
  ${VS_GRAPHS_G2OTYPES_VERTEXVELOCITY_SOURCES}
  ${VS_GRAPHS_G2OTYPES_VERTEXGYROBIAS_SOURCES}
  ${VS_GRAPHS_G2OTYPES_VERTEXACCBIAS_SOURCES}
  core/src/G2oTypes/expSO3/fromVector.cc
  core/src/G2oTypes/expSO3/fromComponents.cc
  core/src/G2oTypes/logSO3.cc
  core/src/G2oTypes/inverseRightJacobianSO3/fromVector.cc
  core/src/G2oTypes/inverseRightJacobianSO3/fromComponents.cc
  core/src/G2oTypes/rightJacobianSO3/fromVector.cc
  core/src/G2oTypes/rightJacobianSO3/fromComponents.cc
  core/src/G2oTypes/skew.cc
)
