set(VS_GRAPHS_DBOW2_DESCRIPTORS_SOURCES
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/computeDistance.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/computeMeanDescriptor.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/fromArray8U.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/fromString.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/toArray8U.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/toMat32F.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/toMat8U.cc
  core/src/dbow2/descriptors/methods/ORBDescriptorFunctionsClass/toString.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_DESCRIPTORS_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
