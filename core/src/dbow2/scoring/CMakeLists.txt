set(VS_GRAPHS_DBOW2_SCORING_SOURCES
  core/src/dbow2/scoring/methods/BhattacharyyaScoringClass/computeScore.cc
  core/src/dbow2/scoring/methods/ChiSquareScoringClass/computeScore.cc
  core/src/dbow2/scoring/methods/DotProductScoringClass/computeScore.cc
  core/src/dbow2/scoring/methods/GeneralScoringClass/static_members.cc
  core/src/dbow2/scoring/methods/KLScoringClass/computeScore.cc
  core/src/dbow2/scoring/methods/L1ScoringClass/computeScore.cc
  core/src/dbow2/scoring/methods/L2ScoringClass/computeScore.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_SCORING_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
