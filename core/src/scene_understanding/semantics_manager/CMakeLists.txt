# SemanticsManager object sources; the scene graph operations are built from
# the graph and bootstrap subpackages, the pipeline stages from the stage
# packages (plane_intake, plane_checks, plane_fusion, passage_detection,
# room_association, room_tracking, room_validation) and map_linking next to
# this one.
include(${CMAKE_CURRENT_LIST_DIR}/bootstrap/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/graph/CMakeLists.txt)

set(VS_GRAPHS_SEMANTICSMANAGER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/captureUnresolvedWallHypotheses.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/checkFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getSemanticReportCacheEntry.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isFinished.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isSemanticReportCacheAvailable.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/logSemanticDiagnostics.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/requestFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/run.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setFinish.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/buildFiniteWallSegment2d.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/buildWallFaceEvidence.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/buildWallFaceMatchSettings.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeRoomCentreFromWalls.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeRoomWallFootprint.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/crossProduct2d.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/evaluateWallAdmissionEvidence.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/findWallRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/getPlaneVariantName.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/intersectSupportingLines.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isPassageTopologyValid.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isPointOnForeignSideOfWallFace.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isRoomOnKnownSide.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/isWallAtPassageOpening.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/linkRoomToPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/pointToSegmentDistance_m.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/segmentCrossesAperture.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/segmentCrossesForeignWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/segmentCrossesPassageOpening.cc
  ${VS_GRAPHS_SEMANTICSMANAGER_BOOTSTRAP_SOURCES}
  ${VS_GRAPHS_SEMANTICSMANAGER_GRAPH_SOURCES}
  ${VS_GRAPHS_PLANEINTAKE_SOURCES}
  ${VS_GRAPHS_PLANECHECKS_SOURCES}
  ${VS_GRAPHS_PLANEFUSION_SOURCES}
  ${VS_GRAPHS_PASSAGEDETECTION_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_SOURCES}
  ${VS_GRAPHS_ROOMTRACKING_SOURCES}
  ${VS_GRAPHS_ROOMVALIDATION_SOURCES}
  ${VS_GRAPHS_MAPLINKING_SOURCES}
)
