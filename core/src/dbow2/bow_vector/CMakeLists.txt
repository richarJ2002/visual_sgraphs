set(VS_GRAPHS_DBOW2_BOWVECTOR_SOURCES
  core/src/dbow2/bow_vector/methods/addIfNotExist.cc
  core/src/dbow2/bow_vector/methods/addWeight.cc
  core/src/dbow2/bow_vector/methods/normalize.cc
  core/src/dbow2/bow_vector/methods/operators/stream_output/withBowVector.cc
  core/src/dbow2/bow_vector/methods/saveToMatlabFile.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_BOWVECTOR_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
