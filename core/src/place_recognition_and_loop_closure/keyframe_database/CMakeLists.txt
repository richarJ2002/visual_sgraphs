# KeyFrameDatabase object sources: exactly this object's translation units.
set(VS_GRAPHS_KEYFRAMEDATABASE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/add.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/clear.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectBestCandidates.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectCandidates.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectLoopCandidates.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectNBestCandidates.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectRelocalizationCandidates.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/erase.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/serialize.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setORBVocabulary.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/compFirst.cc
)
