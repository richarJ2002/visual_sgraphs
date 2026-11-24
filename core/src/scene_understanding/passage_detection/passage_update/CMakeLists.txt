# Passage update: re-anchoring of existing passages to their door or wall
# planes and merging of duplicates; exactly this subpackage's translation
# units.
set(VS_GRAPHS_PASSAGEDETECTION_PASSAGEUPDATE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/mergeOverlappingPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reanchorOpenPassageToWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refreshClosedPassageFromDoor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/updatePassages.cc
)
