# timestamp package sources.
set(VS_GRAPHS_DUTILS_TIMESTAMP_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeEarlierTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeLaterTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/formatDateTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/formatElapsedTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getFloatTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getStringTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isEmpty.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/addition/withSeconds.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/addition_assignment/withSeconds.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/equality/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/greater_than/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/greater_than_or_equal/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/less_than/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/less_than_or_equal/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/subtraction/withSeconds.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/subtraction/withTimestamp.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/operators/subtraction_assignment/withSeconds.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setTime/fromEpochTime.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setTime/fromText.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setToCurrentTime.cc
)

# Every source of this package compiles on its own, outside the unity
# batches: appended to the batched list they would join the last existing
# batch and change its inlining context; kept apart, every existing batch
# keeps its contents.
set_source_files_properties(${VS_GRAPHS_DUTILS_TIMESTAMP_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
