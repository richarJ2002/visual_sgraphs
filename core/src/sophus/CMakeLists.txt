# Sophus move-step template coverage; upstream names remain unchanged.
set(VS_GRAPHS_SOPHUS_SOURCES
  core/src/sophus/methods/explicit_instantiations.cc
)

# Keep emitted template copies after existing sources and out of unity batches
# so their additional instantiations do not change a neighbour's inlining context.
set_source_files_properties(${VS_GRAPHS_SOPHUS_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
