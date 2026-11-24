# Plane check stage (pipeline stage 2): SemanticsManager methods that apply the
# plane rule table to the intake planes; exactly this package's translation
# units.
set(VS_GRAPHS_PLANECHECKS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeGroundPlaneHeight.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computePlaneToHorizontal.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/filterGroundPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/rejectMisorientedPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/runPlaneChecks.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeRotationBetweenDirections.cc
)

# The Eigen setFromTwoVectors diagnostic deviation (see the file header) only
# holds while Eigen is first included by this file, so never batch it.
set_source_files_properties(
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeRotationBetweenDirections.cc
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON
)
