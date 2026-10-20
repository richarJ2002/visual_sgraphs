# ResetCause object sources: exactly this object's translation units.
set(VS_GRAPHS_RESETCAUSE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearResetCause.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/consume.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/consumeResetCause.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/formatResetAttribution.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/hasRetainedCause.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reportResetAttribution.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resetActionToString.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resetCauseToString.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/retain.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/retainResetCause.cc
)
