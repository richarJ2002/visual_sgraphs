set(VS_GRAPHS_DBOW2_DESCRIPTORS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/computeDistance.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/computeMeanDescriptor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/fromArray8U.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/fromString.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/toArray8U.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/toMat32F.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/toMat8U.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/ORBDescriptorFunctionsClass/toString.cc
)

# Preserve existing unity batches and isolate library/instantiation context.
set_source_files_properties(${VS_GRAPHS_DBOW2_DESCRIPTORS_SOURCES} PROPERTIES
  SKIP_UNITY_BUILD_INCLUSION ON)
