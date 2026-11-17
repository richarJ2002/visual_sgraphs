# Plane intake stage (pipeline stage 1): SemanticsManager methods that collect
# the planes and the camera centre a cycle works on; exactly this package's
# translation units.
set(VS_GRAPHS_PLANEINTAKE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/refreshCurrentCameraCenter.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/runPlaneIntake.cc
)
