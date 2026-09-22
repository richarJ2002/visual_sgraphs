# ImuTypes module aggregation: each object subdirectory declares exactly
# its own sources; free IMU-namespace functions live directly in this
# module. No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/Bias/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Calib/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/IntegratedRotation/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Preintegrated/CMakeLists.txt)

set(VS_GRAPHS_IMUTYPES_SOURCES
  ${VS_GRAPHS_IMUTYPES_BIAS_SOURCES}
  ${VS_GRAPHS_IMUTYPES_CALIB_SOURCES}
  ${VS_GRAPHS_IMUTYPES_INTEGRATEDROTATION_SOURCES}
  ${VS_GRAPHS_IMUTYPES_PREINTEGRATED_SOURCES}
  core/src/ImuTypes/normalizeRotation.cc
  core/src/ImuTypes/rightJacobianSO3/fromComponents.cc
  core/src/ImuTypes/rightJacobianSO3/fromVector.cc
  core/src/ImuTypes/inverseRightJacobianSO3/fromComponents.cc
  core/src/ImuTypes/inverseRightJacobianSO3/fromVector.cc
)
