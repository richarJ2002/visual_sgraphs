# Keep JSON template coverage last and separate from existing unity batches.
set(VS_GRAPHS_NLOHMANN_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/explicit_instantiations.cc
)

set_source_files_properties(${VS_GRAPHS_NLOHMANN_SOURCES}
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
