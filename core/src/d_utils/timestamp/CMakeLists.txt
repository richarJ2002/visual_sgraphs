# timestamp package sources.
set(VS_GRAPHS_DUTILS_TIMESTAMP_SOURCES
  core/src/d_utils/timestamp/methods/computeEarlierTimestamp.cc
  core/src/d_utils/timestamp/methods/computeLaterTimestamp.cc
  core/src/d_utils/timestamp/methods/formatDateTime.cc
  core/src/d_utils/timestamp/methods/formatElapsedTime.cc
  core/src/d_utils/timestamp/methods/getFloatTime.cc
  core/src/d_utils/timestamp/methods/getStringTime.cc
  core/src/d_utils/timestamp/methods/isEmpty.cc
  core/src/d_utils/timestamp/methods/operators/addition/withSeconds.cc
  core/src/d_utils/timestamp/methods/operators/addition_assignment/withSeconds.cc
  core/src/d_utils/timestamp/methods/operators/equality/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/greater_than/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/greater_than_or_equal/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/less_than/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/less_than_or_equal/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/subtraction/withSeconds.cc
  core/src/d_utils/timestamp/methods/operators/subtraction/withTimestamp.cc
  core/src/d_utils/timestamp/methods/operators/subtraction_assignment/withSeconds.cc
  core/src/d_utils/timestamp/methods/setTime/fromEpochTime.cc
  core/src/d_utils/timestamp/methods/setTime/fromText.cc
  core/src/d_utils/timestamp/methods/setToCurrentTime.cc
)

# Every source of this package compiles on its own, outside the unity
# batches: appended to the batched list they would join the last existing
# batch and change its inlining context; kept apart, every existing batch
# keeps its contents.
set_source_files_properties(${VS_GRAPHS_DUTILS_TIMESTAMP_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
