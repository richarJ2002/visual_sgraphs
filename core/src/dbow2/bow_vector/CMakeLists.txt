set(VS_GRAPHS_DBOW2_BOWVECTOR_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addIfNotExist.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/addWeight.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/normalize.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/stream_output/withBowVector.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/saveToMatlabFile.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_BOWVECTOR_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
