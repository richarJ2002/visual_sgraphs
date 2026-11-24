# Wall slab profiles: Voxblox free-space slab queries and answers per wall and
# the open passages measured from them; exactly this subpackage's translation
# units.
set(VS_GRAPHS_PASSAGEDETECTION_SLABPROFILES_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/applyPassageVerdicts.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/buildWallSlabQueries.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectPassagesFromFreeSpace.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getWallSlabQueries.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setWallSlabProfiles.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeHorizontalDistance_m.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeMedianValue.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeObservedApertureFraction.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeSurfaceOffset_m.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/countApertureWallPoints.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/measureWallGap.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/refineWallGapFromPoints.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/visitFreeSlabCell.cc
)
