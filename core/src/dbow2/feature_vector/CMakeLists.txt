set(VS_GRAPHS_DBOW2_FEATUREVECTOR_SOURCES
  core/src/dbow2/feature_vector/methods/addFeature.cc
  core/src/dbow2/feature_vector/methods/operators/stream_output/withFeatureVector.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_FEATUREVECTOR_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
