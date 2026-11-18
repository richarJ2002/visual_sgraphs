# Room consolidation: the SemanticsManager method that fuses duplicate room
# hypotheses inside one free-space cluster; exactly this subpackage's
# translation units.
set(VS_GRAPHS_ROOMASSOCIATION_ROOMCONSOLIDATION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeDistanceToCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/consolidateRoomsInFreeSpaceCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isSeedFusableIntoRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/transferDuplicateRoomWalls.cc
)
