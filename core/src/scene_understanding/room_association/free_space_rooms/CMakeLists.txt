# Free-space rooms: SemanticsManager methods that turn Voxblox skeleton
# clusters, split at passages, into rooms and give them their walls; exactly
# this subpackage's translation units.
set(VS_GRAPHS_ROOMASSOCIATION_FREESPACEROOMS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/admitClusterWallsToRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/applyClusterWallBackstop.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/associateWallsToRoomsFromFreeSpaceClusters.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkWallOwnerTransfer.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/collectAdmissibleWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/confirmClusterRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detachWallFromPreviousOwner.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findClusterWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findFarSideProspectiveRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findOtherWallOwner.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getLatestSkeletonCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isRoomPassageBoundProspective.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isWallSupportedByCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/partitionFreeSpaceAtPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refreshClusterRoomGeometry.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reportDeferredCluster.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resolveClusterRoom.cc
)
