# ImuTypes module aggregation: each object subdirectory declares exactly
# its own sources; free helpers live in private_functions/.
# No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/bias/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/calib/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/preintegrated/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/point/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/integrated_rotation/CMakeLists.txt)

set(VS_GRAPHS_IMUTYPES_HELPER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/normalizeRotation.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/rightJacobianSO3.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/inverseRightJacobianSO3.cc
)

set(VS_GRAPHS_IMUTYPES_SOURCES
  ${VS_GRAPHS_IMUTYPES_BIAS_SOURCES}
  ${VS_GRAPHS_IMUTYPES_CALIB_SOURCES}
  ${VS_GRAPHS_IMUTYPES_PREINTEGRATED_SOURCES}
  ${VS_GRAPHS_IMUTYPES_HELPER_SOURCES}
)
