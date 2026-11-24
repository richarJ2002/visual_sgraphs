# Map linking: SemanticsManager methods for tracking loss and recovery (the
# last known room restored into the new map), per-map temporal reset and
# merge-candidate verification; exactly this package's translation units.
set(VS_GRAPHS_MAPLINKING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/evaluateTopCandidateVerification.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findRoomByMapAndId.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getLastKnownRoomId.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/onTrackingLost.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/onTrackingRecovered.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/recoverRoomAfterTrackingLoss.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resetTemporalStateForMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/restoreRecoveryPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/submitVerificationVerdict.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/createRecoveryPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/linkRecoveryPassage.cc
)
