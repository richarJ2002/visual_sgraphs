# Pinhole object sources.
set(VS_GRAPHS_CAMERAMODELS_PINHOLE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeProjectionJacobian.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/epipolarConstrain.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isEqual.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/matchAndTriangulate.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/project.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/projectMat.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/readCalibration.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconstructWithTwoViews.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/serialize.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toK.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toK_.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/uncertainty2.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/unproject.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/unprojectEig.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/writeCalibration.cc
)
