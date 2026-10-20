# semantic_graph_snapshot module sources.
set(VS_GRAPHS_SEMANTIC_SEMANTICGRAPHSNAPSHOT_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/captureSemanticGraphSnapshot.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/rawPlaneRef.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/appendPassageRef.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/appendRoomRef.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/appendWallRef.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/captureFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/capturePassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/captureRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/captureWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/entityRefForFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/entityRefForPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/entityRefForRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isValueLessForCollisionTiebreak.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/makeKey.cc
)
