# TwoViewReconstruction object sources: exactly this object's translation units.
set(VS_GRAPHS_TWOVIEWRECONSTRUCTION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkFundamental.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkHomography.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkRT.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeF21.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeH21.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/decomposeE.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findFundamental.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findHomography.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/normalize.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconstruct.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconstructF.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconstructH.cc
)
