# MLPnPsolver object sources: exactly this object's translation units.
set(VS_GRAPHS_MLPNPSOLVER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkInliers.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computePose.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/iterate.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/mlpnpJacs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/mlpnp_gn.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/mlpnp_residuals_and_jacs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refine.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/rodrigues2rot.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/rot2rodrigues.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setRansacParameters.cc
)
