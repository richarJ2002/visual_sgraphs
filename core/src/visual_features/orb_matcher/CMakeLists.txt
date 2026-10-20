# ORBmatcher object sources: exactly this object's translation units.
set(VS_GRAPHS_ORBMATCHER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeDescriptorDistance.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeThreeMaxima.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/fuse.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/radiusByViewingCos.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/searchByBoW.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/searchByProjection.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/searchBySim3.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/searchForInitialization.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/searchForTriangulation.cc
)
