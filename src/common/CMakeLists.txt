set(VS_GRAPHS_FRONTEND_COMMON_SOURCES
  private_functions/keepFirstPointPerVoxel.cc
  public_functions/common.cc
)
list(TRANSFORM VS_GRAPHS_FRONTEND_COMMON_SOURCES
  PREPEND "${CMAKE_CURRENT_LIST_DIR}/")
