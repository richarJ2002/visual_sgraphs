# Sim3Solver object sources: exactly this object's translation units.
set(VS_GRAPHS_SIM3SOLVER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkInliers.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeCentroid.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeSim3.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/find.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/fromCameraToImage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getEstimatedRotation.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getEstimatedScale.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getEstimatedTransformation.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getEstimatedTranslation.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/iterate.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/project.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setRansacParameters.cc
)
