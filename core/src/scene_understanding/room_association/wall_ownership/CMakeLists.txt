# Wall ownership: SemanticsManager methods that keep one room per wall face,
# detach walls beyond confirmed passages, keep the passage side and topology
# rules and retire undefended walls; exactly this subpackage's translation
# units.
set(VS_GRAPHS_ROOMASSOCIATION_WALLOWNERSHIP_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/detachWallsBeyondConfirmedPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/enforcePassageSideInvariant.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/enforcePassageTopologyInvariant.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/enforceUniqueWallOwnership.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findPassageFarSideRoomId.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findRedirectedObserverRoomId.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/hasWallLiveClusterSupport.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isWallAssociatedWithPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isWallOwnedByLiveRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resolvePassageSideWallOwner.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/suppressUndefendedWalls.cc
)
