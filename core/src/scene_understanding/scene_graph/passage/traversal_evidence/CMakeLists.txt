# passage object, traversal_evidence subpackage: exactly this subpackage's translation units.
set(VS_GRAPHS_SCENEGRAPH_PASSAGE_TRAVERSALEVIDENCE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addTraversalObservation.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getTraversalEvidence.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getTraversalFarToKnownCount.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getTraversalKnownToFarCount.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getTraversalObservationCount.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getTraversalUnknownCount.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getVerdictCounters.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/hasBidirectionalTraversalEvidence.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setTraversalEvidence.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setTraversalObservationCount.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setVerdictCounters.cc
)
