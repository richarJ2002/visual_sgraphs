set(VS_GRAPHS_DBOW2_VOCABULARY_SOURCES
  core/src/dbow2/vocabulary/methods/explicit_instantiations.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_VOCABULARY_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
