# random package sources.
set(VS_GRAPHS_DUTILS_RANDOM_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/generateUniformInteger.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/seedGenerator/fromCurrentTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/seedGenerator/fromGivenSeed.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/seedGeneratorOnce/fromCurrentTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/seedGeneratorOnce/fromGivenSeed.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/RandomClass/static_members.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/UnrepeatedRandomizerClass/createValues.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/UnrepeatedRandomizerClass/drawNextValue.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/UnrepeatedRandomizerClass/operators/assignment/fromRandomizer.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/UnrepeatedRandomizerClass/reset.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/explicit_instantiations.cc
)

# Every source of this package compiles on its own, outside the unity
# batches: appended to the batched list they would join the last existing
# batch and change its inlining context; kept apart, every existing batch
# keeps its contents.
set_source_files_properties(${VS_GRAPHS_DUTILS_RANDOM_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
