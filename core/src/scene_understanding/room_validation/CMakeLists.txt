# Room validation stage (pipeline stage 7): SemanticsManager methods that close
# room wall loops, validate room boundaries, recompute room centroids and
# reconcile floors and room ground planes; exactly this package's translation
# units.
set(VS_GRAPHS_ROOMVALIDATION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/getUpdatedFloors.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/recomputeRoomCentroidsFromWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reconcileRoomGroundPlanes.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/validateRoomBoundaries.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computePolygonArea_m2.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeRoomObservationGaps.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/tryCloseWallLoop.cc
)
