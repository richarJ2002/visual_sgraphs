# Passage creation: the stage entry, door and free-space detection and the
# creation or duplicate refresh of one passage; exactly this subpackage's
# translation units.
set(VS_GRAPHS_PASSAGEDETECTION_PASSAGECREATION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addNewMapPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/computeNewPassageGeometry.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/createMapPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/detectPassages.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/refreshDuplicatePassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/runPassageDetection.cc
)
