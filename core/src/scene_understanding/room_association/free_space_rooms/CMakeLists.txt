# Free-space rooms: SemanticsManager methods that turn Voxblox skeleton
# clusters, split at passages, into rooms and give them their walls; exactly
# this subpackage's translation units.
set(VS_GRAPHS_ROOMASSOCIATION_FREESPACEROOMS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateWallsToRoomsFromFreeSpaceClusters.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getLatestSkeletonCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/partitionFreeSpaceAtPassages.cc
)
