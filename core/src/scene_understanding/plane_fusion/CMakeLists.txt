# Plane fusion stage (pipeline stage 3): SemanticsManager methods that fuse
# duplicate planes and pair the two faces of a wall; exactly this package's
# translation units.
set(VS_GRAPHS_PLANEFUSION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/logTwinFaceState.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reAssociateSemanticPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconcileWallFacePairs.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/relinkRetiredPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/runPlaneFusion.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/getTwinFaceCheckName.cc
)
