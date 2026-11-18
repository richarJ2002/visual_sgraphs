# Passage rooms: SemanticsManager methods that link each passage to the rooms
# on its two sides and promote or retire its prospective far-side room;
# exactly this subpackage's translation units.
set(VS_GRAPHS_ROOMASSOCIATION_PASSAGEROOMS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/associatePassagesToRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/dropCrossFloorSideRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findPassageSideRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/findPromotionFarSideRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/linkPassageToSideRooms.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/readPassageLinkGeometry.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/resolveProspectiveRoomPromotion.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/retireProspectiveRoom.cc
)
