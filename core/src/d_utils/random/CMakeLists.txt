# random package sources.
set(VS_GRAPHS_DUTILS_RANDOM_SOURCES
  core/src/d_utils/random/methods/RandomClass/generateUniformInteger.cc
  core/src/d_utils/random/methods/RandomClass/seedGenerator/fromCurrentTime.cc
  core/src/d_utils/random/methods/RandomClass/seedGenerator/fromGivenSeed.cc
  core/src/d_utils/random/methods/RandomClass/seedGeneratorOnce/fromCurrentTime.cc
  core/src/d_utils/random/methods/RandomClass/seedGeneratorOnce/fromGivenSeed.cc
  core/src/d_utils/random/methods/RandomClass/static_members.cc
  core/src/d_utils/random/methods/UnrepeatedRandomizerClass/createValues.cc
  core/src/d_utils/random/methods/UnrepeatedRandomizerClass/drawNextValue.cc
  core/src/d_utils/random/methods/UnrepeatedRandomizerClass/operators/assignment/fromRandomizer.cc
  core/src/d_utils/random/methods/UnrepeatedRandomizerClass/reset.cc
  core/src/d_utils/random/methods/explicit_instantiations.cc
)

# Every source of this package compiles on its own, outside the unity
# batches: appended to the batched list they would join the last existing
# batch and change its inlining context; kept apart, every existing batch
# keeps its contents.
set_source_files_properties(${VS_GRAPHS_DUTILS_RANDOM_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
