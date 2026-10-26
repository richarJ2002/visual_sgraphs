set(VS_GRAPHS_FRONTEND_COMMON_SOURCES
  private_functions/keepFirstPointPerVoxel.cc
  public_functions/common.cc
  public_functions/lookupWorldFromFrameTransform.cc
  public_functions/setupWallSlabProfileClient.cc
)
list(TRANSFORM VS_GRAPHS_FRONTEND_COMMON_SOURCES
  PREPEND "${CMAKE_CURRENT_LIST_DIR}/")
