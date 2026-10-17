# semantic_graph_snapshot module sources.
set(VS_GRAPHS_SEMANTIC_SEMANTICGRAPHSNAPSHOT_SOURCES
  core/src/semantic/semantic_graph_snapshot/methods/captureSemanticGraphSnapshot.cc
  core/src/semantic/semantic_graph_snapshot/methods/rawPlaneRef.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/appendPassageRef.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/appendRoomRef.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/appendWallRef.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/captureFloor.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/capturePassage.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/captureRoom.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/captureWall.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/entityRefForFloor.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/entityRefForPassage.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/entityRefForRoom.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/isValueLessForCollisionTiebreak.cc
  core/src/semantic/semantic_graph_snapshot/private_functions/makeKey.cc
)
