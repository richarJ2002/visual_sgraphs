# ORBextractor object sources: exactly this object's translation units.
set(VS_GRAPHS_ORBEXTRACTOR_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/call.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeKeyPointsOctTree.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeKeyPointsOld.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computePyramid.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/construct.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/distributeOctTree.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/divideNode.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/compareNodes.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeDescriptors.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeOrbDescriptor.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeOrientation.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/icAngle.cc
)
