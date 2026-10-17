# d_utils module aggregation: each package declares exactly its own sources;
# this file aggregates them into the list consumed by the root CMakeLists.txt.
include(${CMAKE_CURRENT_LIST_DIR}/random/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/timestamp/CMakeLists.txt)

set(VS_GRAPHS_DUTILS_SOURCES
  ${VS_GRAPHS_DUTILS_RANDOM_SOURCES}
  ${VS_GRAPHS_DUTILS_TIMESTAMP_SOURCES}
)
