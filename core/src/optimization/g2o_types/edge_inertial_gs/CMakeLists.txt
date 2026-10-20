# EdgeInertialGS object sources.
set(VS_GRAPHS_G2OTYPES_EDGEINERTIALGS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeError.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessian.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessian2.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessian3.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessianBiasAcc.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessianBiasGyro.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessianGravityDirection.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getHessianScale.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/linearizeOplus.cc
)
