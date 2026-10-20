set(VS_GRAPHS_DBOW2_SCORING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/BhattacharyyaScoringClass/computeScore.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ChiSquareScoringClass/computeScore.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/DotProductScoringClass/computeScore.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/GeneralScoringClass/static_members.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/KLScoringClass/computeScore.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/L1ScoringClass/computeScore.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/L2ScoringClass/computeScore.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_SCORING_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
