# SemanticsManager graph pose correction: moves the rooms, passages and storeys
# of a map with its corrected keyframes, exactly this subpackage's translation
# units.
set(VS_GRAPHS_SEMANTICSMANAGER_GRAPH_POSE_CORRECTION_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/applyPoseCorrectionField.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/selectCorrectionForPoint.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/transformSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/correctFloorPoses.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/correctPassagePoses.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/correctRoomPoses.cc
)
