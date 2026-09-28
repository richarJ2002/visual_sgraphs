cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

project(vs_graphs)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Do not force -O3 here: CMake's own per-config defaults already give
# Release "-O3 -DNDEBUG" and Debug "-g". Appending -O3 unconditionally used
# to outrank a Debug build's "-O0 -g" (the compiler takes the last -O flag
# on its command line), producing an optimized binary that claimed to be a
# debuggable Debug build.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# System PCL allocates point buffers with plain malloc. ASan run
# 20260830-150425 showed Eigen's handmade_aligned_free reading eight bytes
# before such a buffer. Use the same malloc/free path across this ABI boundary.
add_compile_definitions(EIGEN_MALLOC_ALREADY_ALIGNED=1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# Find required packages
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(GLEW REQUIRED)
find_package(rclpy REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Pangolin REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(segmenter_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(situational_graphs_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "srv/GetMissionHealth.srv"
  "srv/EstimatorHealth.srv"
  "msg/VSGraphsWallData.msg"
  "msg/VSGraphsRoomData.msg"
  "msg/VSGraphsAllWallsData.msg"
  "msg/VSGraphsAllDetectdetRooms.msg"
  "msg/VSGraphsPassageData.msg"
  "msg/VSGraphsAllPassagesData.msg"
  "msg/VSGraphsFloorData.msg"
  "msg/VSGraphsAllFloorsData.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/core/include
  ${PROJECT_SOURCE_DIR}/core/include/CameraModels
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${segmenter_ros_INCLUDE_DIRS}
)

# Vendored code is not project-owned (CPP_CODING_STANDARD.md sections 1.2 and
# 15.3), so it is exposed as SYSTEM: diagnostics raised inside its headers
# (e.g. g2o's use of the deprecated Eigen::AlignedBit) are not reported.
# Only these roots are affected; project-owned headers under core/include keep
# the full warning set. `core` is the root of the `Thirdparty/...` includes.
include_directories(SYSTEM
  ${PROJECT_SOURCE_DIR}/core
  ${PROJECT_SOURCE_DIR}/core/Thirdparty
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/Sophus
)
include_directories(${OPENGL_INCLUDE_DIRS})

# ROS2 uses ament_target_dependencies instead of catkin_INCLUDE_DIRS
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include third-party CMakeLists
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/CMakeLists.txt)

# Types module sources (objects/methods layout)
include(${PROJECT_SOURCE_DIR}/cmake/auto_sources_Types_Types.cmake)

# CameraModels module sources (object-subpackage layout)
include(${PROJECT_SOURCE_DIR}/core/src/CameraModels/CMakeLists.txt)

# Utils module sources (object-subpackage layout)
include(${PROJECT_SOURCE_DIR}/core/src/Utils/CMakeLists.txt)

# Geometric module sources (Plane + GeometricTools object packages, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Geometric/CMakeLists.txt)

# Semantic object packages (Room/Passage/Floor/Marker/RoomTracker/…, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Semantic/CMakeLists.txt)

# StereoMatchOutlierRejection module sources (single free function, §5.4)
include(${PROJECT_SOURCE_DIR}/core/src/StereoMatchOutlierRejection/CMakeLists.txt)

# DatabaseParser module sources (single DBParser object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/DatabaseParser/CMakeLists.txt)

# MissionHealthTopologyJson module sources (free functions, §5.4)
include(${PROJECT_SOURCE_DIR}/core/src/MissionHealthTopologyJson/CMakeLists.txt)

# ImuTypes module sources (Bias/Calib/IntegratedRotation/Preintegrated, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/ImuTypes/CMakeLists.txt)

# MapDrawer object sources (single MapDrawer object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/MapDrawer/CMakeLists.txt)

# FrameDrawer object sources (single FrameDrawer object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/FrameDrawer/CMakeLists.txt)

# ResetCause module sources (ResetCauseRetention object + free functions)
include(${PROJECT_SOURCE_DIR}/core/src/ResetCause/CMakeLists.txt)

# Sim3Solver object sources (single Sim3Solver object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Sim3Solver/CMakeLists.txt)

# G2oTypes module sources (g2o vertex/edge objects + SO(3) helpers, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/G2oTypes/CMakeLists.txt)

# KeyFrameDatabase module sources (single KeyFrameDatabase object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/KeyFrameDatabase/CMakeLists.txt)

# TwoViewReconstruction object sources (single object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/TwoViewReconstruction/CMakeLists.txt)

# MLPnPsolver object sources (single object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/MLPnPsolver/CMakeLists.txt)

# ORBextractor module sources (single object + internal helpers, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/ORBextractor/CMakeLists.txt)

# ORBmatcher object sources (single object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/ORBmatcher/CMakeLists.txt)

# MapPoint object sources (single object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/MapPoint/CMakeLists.txt)

# Map object sources (single object, SCC construct/ out-of-line, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Map/CMakeLists.txt)

# Frame object sources (single object, SCC construct/ out-of-line, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Frame/CMakeLists.txt)

# KeyFrame object sources (single object, SCC construct/ out-of-line, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/KeyFrame/CMakeLists.txt)

# System object sources (single System object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/System/CMakeLists.txt)

# Tracking object sources (single Tracking object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Tracking/CMakeLists.txt)

# LocalMapping object sources (single LocalMapping object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/LocalMapping/CMakeLists.txt)

# LoopClosing object sources (single LoopClosing object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/LoopClosing/CMakeLists.txt)

# Optimizer module sources (static methods + internal helpers, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Optimizer/CMakeLists.txt)

# SemanticSegmentation object sources (single object + internal helpers, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/SemanticSegmentation/CMakeLists.txt)

# SemanticsManager object sources (single object + internal helpers, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/SemanticsManager/CMakeLists.txt)

# Atlas object sources (single Atlas object + internal helpers, §4.5, LAST)
include(${PROJECT_SOURCE_DIR}/core/src/Atlas/CMakeLists.txt)
# OptimizableTypes module sources (g2o edge objects, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/OptimizableTypes/CMakeLists.txt)

# Viewer object sources (single Viewer object, §4.5)
include(${PROJECT_SOURCE_DIR}/core/src/Viewer/CMakeLists.txt)

# Set library output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/core/lib)

# Create the main library
add_library(${PROJECT_NAME}_lib SHARED
  ${VS_GRAPHS_MISSIONHEALTHTOPOLOGYJSON_SOURCES}
  ${VS_GRAPHS_TRACKING_SOURCES}
  ${VS_GRAPHS_RESETCAUSE_SOURCES}
  ${VS_GRAPHS_LOCALMAPPING_SOURCES}
  ${VS_GRAPHS_LOOPCLOSING_SOURCES}
  ${VS_GRAPHS_ORBEXTRACTOR_SOURCES}
  ${VS_GRAPHS_ORBMATCHER_SOURCES}
  ${VS_GRAPHS_FRAMEDRAWER_SOURCES}
  ${VS_GRAPHS_MAPPOINT_SOURCES}
  ${VS_GRAPHS_KEYFRAME_SOURCES}
  ${VS_GRAPHS_SYSTEM_SOURCES}
  ${VS_GRAPHS_ATLAS_SOURCES}
  ${VS_GRAPHS_MAP_SOURCES}
  ${VS_GRAPHS_MAPDRAWER_SOURCES}
  ${VS_GRAPHS_OPTIMIZER_SOURCES}
  ${VS_GRAPHS_FRAME_SOURCES}
  ${VS_GRAPHS_STEREOMATCHOUTLIERREJECTION_SOURCES}
  ${VS_GRAPHS_KEYFRAMEDATABASE_SOURCES}
  ${VS_GRAPHS_SIM3SOLVER_SOURCES}
  ${VS_GRAPHS_VIEWER_SOURCES}
  ${VS_GRAPHS_IMUTYPES_SOURCES}
  ${VS_GRAPHS_G2OTYPES_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_SOURCES}
  ${VS_GRAPHS_OPTIMIZABLETYPES_SOURCES}
  ${VS_GRAPHS_MLPNPSOLVER_SOURCES}
  ${VS_GRAPHS_GEOMETRIC_SOURCES}
  ${VS_GRAPHS_SEMANTIC_OBJECTS_SOURCES}
  ${VS_GRAPHS_TWOVIEWRECONSTRUCTION_SOURCES}
  ${VS_GRAPHS_UTILS_SOURCES}

  ${VS_GRAPHS_DATABASEPARSER_SOURCES}
  core/src/Semantic/SemanticGraphSnapshot/objects/EntityKey/operators/equality.cc
  core/src/Semantic/SemanticGraphSnapshot/objects/EntityKey/operators/inequality.cc
  core/src/Semantic/SemanticGraphSnapshot/objects/EntityKey/operators/lessThan.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/makeKey.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/entityRefForRoom.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/entityRefForFloor.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/entityRefForPassage.cc
  core/src/Semantic/SemanticGraphSnapshot/public_functions/rawPlaneRef.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/appendWallRef.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/appendRoomRef.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/appendPassageRef.cc
  core/src/Semantic/ValueOrder/doubleTotalOrderKey.cc
  core/src/Semantic/ValueOrder/isDoubleLess.cc
  core/src/Semantic/ValueOrder/isVector3dLess.cc
  core/src/Semantic/ValueOrder/isVector4dLess.cc
  core/src/Semantic/ValueOrder/isEntityRefLess.cc
  core/src/Semantic/ValueOrder/isRawPlaneRefLess.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/isValueLessForCollisionTiebreak/room.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/isValueLessForCollisionTiebreak/wall.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/isValueLessForCollisionTiebreak/passage.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/isValueLessForCollisionTiebreak/floor.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/captureRoom.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/captureWall.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/capturePassage.cc
  core/src/Semantic/SemanticGraphSnapshot/private_functions/captureFloor.cc
  core/src/Semantic/SemanticGraphSnapshot/public_functions/captureSemanticGraphSnapshot.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/axiomClassFor.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/makeFinding.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/sortFindings.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/aggregateFindings.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/resolveRoomEndpoint.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/isRealPassageEndpoint.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/isKnownInvalidPassageEndpointReference.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/roomListsPassageBack.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/countRoomRecordsWithKey.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/countFloorRecordsWithKey.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/countWallRecordsWithKey.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/countPassageRecordsWithKey.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/countMapSnapshotsWithId.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/scanReversePassageEndpoints.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateRoomMalformedPassageReferences.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/canonicalRoomFloorResultFor.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluatePassageFloorAgreement.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/newellNormal.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/planeNormalFallback.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/orientation2d.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/isOnSegmentBoundingBox.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/doSegmentsIntersect.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/hasNonFiniteCoordinate.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/checkRoomBoundaryGeometry.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/isValidBoundaryWallEvidence.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/anyFindingIs.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/appendKeysFromFindings.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxFrame01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOneWall.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxWall01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxWall02.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/findWallByKeyInSnapshot.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOneWallTwin.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxWall03.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOnePassageProvenance.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxPass01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOnePassageCardinality.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxPass02.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOnePassageSlotState.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxPass03.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOnePassageMapAndFloor.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxPass04.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOneRoomCreationProvenance.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxRoom01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxRoom02.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOneRoomBoundary.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxBound01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOneRoomFloorReciprocity.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateOnePassageFloorIdentity.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxFloor01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxLife01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxTxn01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxComp01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/evaluateAxMerge01.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/computeLegacyMapCompleteness.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/computeConservativeMapCompleteness.cc
  core/src/Semantic/SemanticAxiomEvaluator/public_functions/evaluateState.cc
  core/src/Semantic/SemanticAxiomEvaluator/public_functions/evaluateTransition.cc
  core/src/Semantic/SemanticAxiomEvaluator/public_functions/evaluateMapCompleteness.cc
  core/src/Semantic/SemanticAxiomEvaluator/private_functions/makeAxiomCapabilityEntry.cc
  core/src/Semantic/SemanticAxiomEvaluator/public_functions/computeAxiomCapabilityTable.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/axiomCodeName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/axiomResultName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/axiomClassName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/reasonCodeName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/entityKindName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/unavailableReasonName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/capabilityLevelName.cc
  core/src/Semantic/SemanticAxiomEvaluator/EnumNames/missingProofOwnerName.cc
  core/src/Semantic/SemanticReportCache/update.cc
  core/src/Semantic/SemanticReportCache/getLatest.cc
  core/src/Semantic/SemanticReportCache/isAvailable.cc
  core/src/Semantic/SemanticDiagnostics/private_functions/serializeFiniteAwareDouble.cc
  core/src/Semantic/SemanticDiagnostics/private_functions/findingEvidenceToJson.cc
  core/src/Semantic/SemanticDiagnostics/private_functions/entityKeysToJson.cc
  core/src/Semantic/SemanticDiagnostics/private_functions/violationDetailToJson.cc
  core/src/Semantic/SemanticDiagnostics/public_functions/buildSemanticDiagnosticUpdate.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeDouble.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeVector3d.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeVector4d.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeEntityKey.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeEntityRef.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeRawPlaneRef.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeRoomRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeWallRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializePassageRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeFloorRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeMapSnapshot.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeSnapshot.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeFinding.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeAggregateAxiomResult.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeLegacyMapCompletenessResult.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeMapCompletenessResult.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isFindingLessTotalOrder.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isAggregateAxiomResultLessTotalOrder.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isMapCompletenessResultLessTotalOrder.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isRoomRecordLessTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isRoomRecordLessFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isWallRecordLessTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isWallRecordLessFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isPassageRecordLessTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isPassageRecordLessFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isFloorRecordLessTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isFloorRecordLessFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isMapSnapshotLessTotalOrder.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeOpenPassageHypothesisRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/serializeUnresolvedWallHypothesisRecord.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isOpenPassageHypothesisRecordLessTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isOpenPassageHypothesisRecordLessFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/private_functions/isUnresolvedWallHypothesisRecordLessTotalOrder.cc
  core/src/Semantic/SemanticCanonicalSerialization/public_functions/serializeSnapshotTopologyOnly.cc
  core/src/Semantic/SemanticCanonicalSerialization/public_functions/serializeSnapshotFullGeometry.cc
  core/src/Semantic/SemanticCanonicalSerialization/public_functions/serializeEvaluationReport.cc
  core/src/Semantic/SemanticCanonicalSerialization/public_functions/serializeMapCompletenessResults.cc
  ${VS_GRAPHS_TYPES_TYPES_SOURCES}
  ${VS_GRAPHS_SEMANTICSEGMENTATION_SOURCES}
  ${VS_GRAPHS_SEMANTICSMANAGER_SOURCES}
  core/include/System.h
  core/include/Tracking.h
  core/include/LocalMapping.h
  core/include/LoopClosing.h
  core/include/ORBextractor.h
  core/include/ORBmatcher.h
  core/include/FrameDrawer.h
  core/include/MapPoint.h
  core/include/KeyFrame.h
  core/include/Atlas.h
  core/include/AtlasCurrentMapStatus.h
  core/include/Map.h
  core/include/MapDrawer.h
  core/include/Optimizer.h
  core/include/Frame.h
  core/include/KeyFrameDatabase.h
  core/include/Sim3Solver.h
  core/include/Viewer.h
  core/include/ImuTypes.h
  core/include/G2oTypes.h
  ${VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_OBJECT_HEADERS}
  core/include/OptimizableTypes.h
  core/include/MLPnPsolver.h
  core/include/GeometricTools.h
  core/include/TwoViewReconstruction.h
  core/include/SerializationUtils.h
  ${VS_GRAPHS_UTILS_OBJECT_HEADERS}
  core/include/GeoSemHelpers.h
  core/include/Semantic/Room.h
  core/include/Semantic/Floor.h
  core/include/DatabaseParser.h
  core/include/Semantic/Marker.h
  core/include/Geometric/Plane.h
  core/include/Geometric/PlaneGeometryMetadataSnapshot.h
  core/include/Semantic/Passage.h
  ${VS_GRAPHS_TYPES_TYPES_OBJECT_HEADERS}
  core/include/SemanticSegmentation.h
  core/include/SemanticsManager.h
  core/include/Semantic/RoomTracker.h
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# ROS2 specific: Use ament_target_dependencies for ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  nav_msgs
  message_filters
  image_transport
  rviz_visual_tools
  situational_graphs_msgs
  segmenter_ros
)

# ---------------------------------------------------------------------------
# Common source files
# ---------------------------------------------------------------------------

set(COMMON_SOURCES
  ${PROJECT_SOURCE_DIR}/src/common.cc
  ${PROJECT_SOURCE_DIR}/src/classifySparseMarker.cc
)

# ---------------------------------------------------------------------------
# Dependencies required by the common source files
# ---------------------------------------------------------------------------

set(COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  segmenter_ros
  situational_graphs_msgs
)

# ---------------------------------------------------------------------------
# ROS2 node creation helper
# ---------------------------------------------------------------------------

function(add_ros2_node target_name cpp_file)
  add_executable(${target_name}
    ${COMMON_SOURCES}
    ${cpp_file}
  )

  target_link_libraries(${target_name}
    ${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )

  ament_target_dependencies(${target_name}
    ${COMMON_DEPENDENCIES}
  )

  rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
  )
  target_link_libraries(
    ${target_name}
    "${cpp_typesupport_target}"
  )

  set_target_properties(${target_name} PROPERTIES
    INSTALL_RPATH "\$ORIGIN/.."
  )

  install(
    TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# ---------------------------------------------------------------------------
# ROS2 nodes
# ---------------------------------------------------------------------------

add_ros2_node(
  ros_rgbd
  ${PROJECT_SOURCE_DIR}/src/ros_rgbd.cc
)
target_sources(ros_rgbd PRIVATE
  ${PROJECT_SOURCE_DIR}/src/RgbdObservability.cc
  ${PROJECT_SOURCE_DIR}/src/RgbdAllPointsCadence.cc
)

add_ros2_node(
  ros_rgbd_inertial
  ${PROJECT_SOURCE_DIR}/src/ros_rgbd_inertial.cc
)

# libvs_graphs_lib.so must also be able to find libDBoW2.so and libg2o.so
# in the same install/vs_graphs/lib/ folder.
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  INSTALL_RPATH "\$ORIGIN"
)

# Install other resources
install(
  DIRECTORY launch config core/Vocabulary
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install the main vs_graphs shared library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install third-party shared libraries needed at runtime
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libDBoW2.so*"
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libg2o.so*"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ---------------------------------------------------------------------------
# Deterministic unit tests, no Gazebo.
# RoomTracker is standalone, so the unit targets link its sources directly and
# do not depend on the heavy production library. The multi-process bag-replay
# harness is intentionally deferred.
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
    VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK
  )

  ament_add_gtest(test_unit_skeleton
    test/test_unit_skeleton.cpp
    ${VS_GRAPHS_SEMANTIC_ROOMTRACKER_SOURCES}
  )

  ament_add_gtest(test_RoomTracker
    test/test_RoomTracker.cpp
    ${VS_GRAPHS_SEMANTIC_ROOMTRACKER_SOURCES}
  )

  ament_add_gtest(test_CandidateGen
    test/test_CandidateGen.cpp
    ${VS_GRAPHS_SEMANTIC_SEMANTICCANDIDATES_SOURCES}
  )

  ament_add_gtest(test_atlas_lock_order
    test/test_atlas_lock_order.cpp
  )
  target_link_libraries(test_atlas_lock_order
    ${PROJECT_NAME}_lib
  )

  ament_add_gtest(test_RoomContextPersist
    test/test_RoomContextPersist.cpp
  )
  target_link_libraries(test_RoomContextPersist
    ${PROJECT_NAME}_lib
  )

  ament_add_gtest(test_GeometricVerify
    test/test_GeometricVerify.cpp
  )
  target_link_libraries(test_GeometricVerify
    ${PROJECT_NAME}_lib
  )

  ament_add_gtest(test_SemanticStatus
    test/test_SemanticStatus.cpp
  )
  target_link_libraries(test_SemanticStatus
    ${PROJECT_NAME}_lib
  )

  ament_add_gtest(test_GeometricToolsStatus
    test/test_GeometricToolsStatus.cpp
  )
  target_link_libraries(test_GeometricToolsStatus
    ${PROJECT_NAME}_lib
  )

  ament_add_gtest(test_room_tracker_integration
    test/test_room_tracker_integration.cpp
  )

  # Test-only SemanticsManager methods (*ForTest) live in their own small
  # library on top of the production library; SemanticsManager's layout is the
  # same in both, so the production sources are compiled once.
  add_library(${PROJECT_NAME}_room_tracker_test_lib SHARED
    ${VS_GRAPHS_SEMANTICSMANAGER_TEST_HOOK_SOURCES}
  )
  target_link_libraries(${PROJECT_NAME}_room_tracker_test_lib
    ${PROJECT_NAME}_lib
  )
  ament_target_dependencies(${PROJECT_NAME}_room_tracker_test_lib
    rclcpp
    std_msgs
    cv_bridge
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    pcl_ros
    nav_msgs
    message_filters
    image_transport
    rviz_visual_tools
    situational_graphs_msgs
    segmenter_ros
  )
  set_target_properties(${PROJECT_NAME}_room_tracker_test_lib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
  target_link_libraries(test_room_tracker_integration
    ${PROJECT_NAME}_room_tracker_test_lib
  )
  target_link_options(test_atlas_lock_order PRIVATE
    "-Wl,--export-dynamic-symbol=vsGraphsAtlasLockOrderBeforeMapSnapshot"
  )

  # Exercises SemanticsManager::evaluateTopCandidateVerification
  # against real Atlas/Map/Room/Plane/Floor objects, through the same
  # test-only SemanticsManager library as test_room_tracker_integration.
  ament_add_gtest(test_verification_wiring_integration
    test/test_verification_wiring_integration.cpp
  )
  target_link_libraries(test_verification_wiring_integration
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # Two wall-admission fixes reported directly against a
  # live sim run (ground-aligned height/width gate, wrong-side observation
  # backstop). Reuses the same instrumented production library as above.
  ament_add_gtest(test_WallAdmission
    test/test_WallAdmission.cpp
  )
  target_link_libraries(test_WallAdmission
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  ament_add_gtest(test_SemanticBootstrapPhase1
    test/test_SemanticBootstrapPhase1.cpp
  )
  target_link_libraries(test_SemanticBootstrapPhase1
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # Phase 1: wall-pair (twin-face) reconciliation (axiom (e)). Needs the
  # private-method test hook, so it links the same instrumented production
  # library as test_WallAdmission above.
  ament_add_gtest(test_WallPairing
    test/test_WallPairing.cpp
  )
  target_link_libraries(test_WallPairing
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # Phase 2(ii): front-end floor-flatness reconciliation. Needs the
  # private-method test hook, so it links the same instrumented production
  # library as test_WallAdmission above.
  ament_add_gtest(test_FloorFlatness
    test/test_FloorFlatness.cpp
  )
  target_link_libraries(test_FloorFlatness
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # User rule: a map may hold at most one more room than it has passable
  # passages, enforced at GeoSemHelpers::createBlankRoomCandidate() (a
  # public static method -- no private test hook needed, but reuses the
  # already-built instrumented library for consistency).
  ament_add_gtest(test_RoomCreationBudget
    test/test_RoomCreationBudget.cpp
  )
  target_link_libraries(test_RoomCreationBudget
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # User rule: a wall not part of a room's true closed boundary loop, and
  # not explained by a passage, must be detached rather than silently
  # blocking that room from ever reaching COMPLETE. Needs the private-method
  # test hook.
  ament_add_gtest(test_BoundaryLoopOutlierPruning
    test/test_BoundaryLoopOutlierPruning.cpp
  )
  target_link_libraries(test_BoundaryLoopOutlierPruning
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # User rule: manage incomplete rooms explicitly -- track unobserved
  # angular sectors as situational-awareness data. Needs the private-method
  # test hook.
  ament_add_gtest(test_RoomObservationGaps
    test/test_RoomObservationGaps.cpp
  )
  target_link_libraries(test_RoomObservationGaps
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  ament_add_gtest(test_rgbd_observability
    test/test_rgbd_observability.cpp
    src/RgbdObservability.cc
    src/RgbdAllPointsCadence.cc
    ${VS_GRAPHS_RESETCAUSE_SOURCES}
  )
  target_include_directories(test_rgbd_observability PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/core/include
  )

  ament_add_gtest(test_SparseClusterVerdict
    test/test_SparseClusterVerdict.cpp
    src/classifySparseMarker.cc
  )

  # Phase 0 bug sweep (B1): computeGroundPlaneHeight() empty-cloud guard.
  # Needs the private-method test hook, so it links the same instrumented
  # production library as test_WallAdmission above.
  ament_add_gtest(test_GroundPlaneFilter
    test/test_GroundPlaneFilter.cpp
  )
  target_link_libraries(test_GroundPlaneFilter
    ${PROJECT_NAME}_room_tracker_test_lib
  )

  # Phase 0 bug sweep (B3): Frame::ComputeStereoMatches()'s outlier-rejection
  # step, extracted into a small dependency-free helper so it's testable in
  # isolation, without pulling in Frame's full ORB-SLAM3 dependency chain.
  ament_add_gtest(test_StereoMatchOutlierRejection
    test/test_StereoMatchOutlierRejection.cpp
    ${VS_GRAPHS_STEREOMATCHOUTLIERREJECTION_SOURCES}
  )
  target_include_directories(test_StereoMatchOutlierRejection PRIVATE
    ${PROJECT_SOURCE_DIR}/core/include
  )

  # Phase 0 bug sweep (B4): Optimizer.cc's swapped mono/stereo edge-keyframe
  # vector lookups, extracted into a bounds-checked, dependency-free header.
  ament_add_gtest(test_OptimizerEdgeLookup
    test/test_OptimizerEdgeLookup.cpp
  )
  target_include_directories(test_OptimizerEdgeLookup PRIVATE
    ${PROJECT_SOURCE_DIR}/core/include
  )

  # Regression test: TwoViewReconstruction::reconstructH() computed the
  # triangulated structure of its winning hypothesis but never published it
  # through vP3D, unlike its sibling reconstructF(). The test drives
  # reconstructH() on a synthetic tilted-plane scene and needs the production
  # TwoViewReconstruction and GeometricTools code, so it links the plain
  # production library.
  ament_add_gtest(test_TwoViewReconstruction
    test/test_TwoViewReconstruction.cpp
  )
  target_link_libraries(test_TwoViewReconstruction
    ${PROJECT_NAME}_lib
  )

  # Deterministic, ROS/Gazebo-free fixture builders shared by later
  # semantic-axiom tests. Self-test only exercises plain model construction
  # (no SemanticsManager private methods), so it links the plain production
  # library.
  ament_add_gtest(test_SemanticFixtures
    test/test_SemanticFixtures.cpp
    test/SemanticFixtures.cc
  )
  target_link_libraries(test_SemanticFixtures
    ${PROJECT_NAME}_lib
  )

  # Scoped global wall precision/recall/F1 adapter fixing the comparator's
  # matched-room-only wall denominator. Header-only nlohmann::json parsing
  # plus plain Eigen geometry -- no production library dependency.
  get_filename_component(VS_GRAPHS_WORKSPACE_ROOT
    "${PROJECT_SOURCE_DIR}/../.." ABSOLUTE
  )
  ament_add_gtest(test_GlobalWallMetrics
    test/test_GlobalWallMetrics.cpp
    test/GlobalWallMetrics.cc
  )
  target_compile_definitions(test_GlobalWallMetrics PRIVATE
    VS_GRAPHS_WORKSPACE_ROOT="${VS_GRAPHS_WORKSPACE_ROOT}"
  )

  # Focused, ROS/Gazebo-free tests for the value-only
  # SemanticGraphSnapshot capture contract. Links the production library
  # (captureSemanticGraphSnapshot() and the model classes it reads) plus
  # the shared fixture builders.
  ament_add_gtest(test_SemanticGraphSnapshot
    test/test_SemanticGraphSnapshot.cpp
    test/SemanticFixtures.cc
    test/SemanticGraphSnapshotTestHelpers/findRoomRecord.cc
    test/SemanticGraphSnapshotTestHelpers/findWallRecord.cc
    test/SemanticGraphSnapshotTestHelpers/findPassageRecord.cc
    test/SemanticGraphSnapshotTestHelpers/findMapSnapshot.cc
  )
  # SemanticGraphSnapshotTestHelpers/*.cc quote-includes the sibling
  # SemanticGraphSnapshotTestHelpers.h declaration header one directory up;
  # that quoted include only resolves via the including file's own
  # directory or an explicit include path, so add test/ explicitly.
  target_include_directories(test_SemanticGraphSnapshot PRIVATE
    ${PROJECT_SOURCE_DIR}/test
  )
  target_link_libraries(test_SemanticGraphSnapshot
    ${PROJECT_NAME}_lib
  )

  # Focused, ROS/Gazebo-free tests for the pure SemanticAxiomEvaluator
  # module (evaluateState(), evaluateTransition(),
  # evaluateMapCompleteness(), computeAxiomCapabilityTable()).
  ament_add_gtest(test_SemanticAxiomEvaluator
    test/test_SemanticAxiomEvaluator.cpp
    test/SemanticFixtures.cc
  )
  target_link_libraries(test_SemanticAxiomEvaluator
    ${PROJECT_NAME}_lib
  )

  # Focused, ROS/Gazebo-free tests for versioned canonical JSON
  # serialization of SemanticGraphSnapshot/AxiomEvaluationReport/
  # MapCompletenessResult.
  ament_add_gtest(test_SemanticCanonicalSerialization
    test/test_SemanticCanonicalSerialization.cpp
    test/SemanticFixtures.cc
  )
  target_link_libraries(test_SemanticCanonicalSerialization
    ${PROJECT_NAME}_lib
  )

  # Focused, ROS/Gazebo-free tests for the copied-value
  # SemanticReportCache contract.
  ament_add_gtest(test_SemanticReportCache
    test/test_SemanticReportCache.cpp
    test/SemanticFixtures.cc
  )
  target_link_libraries(test_SemanticReportCache
    ${PROJECT_NAME}_lib
  )

  # Focused, ROS/Gazebo-free tests for the pure mission-health topology
  # schema-2 extension.
  ament_add_gtest(test_MissionHealthTopologyJson
    test/test_MissionHealthTopologyJson.cpp
  )
  target_link_libraries(test_MissionHealthTopologyJson
    ${PROJECT_NAME}_lib
  )

  # Focused, ROS/Gazebo-free tests for the pure SG_AXIOM/SG_VIOLATION
  # diagnostic-update builder.
  ament_add_gtest(test_SemanticDiagnostics
    test/test_SemanticDiagnostics.cpp
  )
  target_link_libraries(test_SemanticDiagnostics
    ${PROJECT_NAME}_lib
  )

  # Deterministic replay of 147 preserved legacy capture_attempt_*.json
  # files through the current evaluator. Produces topology-hash stability
  # proof and malformed-file report.
  ament_add_gtest(test_LegacyCaptureReplay
    test/test_LegacyCaptureReplay.cpp
    test/LegacyCaptureReplay.cc
  )
  target_compile_definitions(test_LegacyCaptureReplay PRIVATE
    VS_GRAPHS_WORKSPACE_ROOT="${VS_GRAPHS_WORKSPACE_ROOT}"
  )
  target_link_libraries(test_LegacyCaptureReplay
    ${PROJECT_NAME}_lib
  )

  # Gate-run traversal repro: synthetic 3-keyframe crossing of a Passage#1-like
  # aperture through updateTraversalEvidence().
  ament_add_gtest(test_PassageTraversalRepro
    test/test_PassageTraversalRepro.cpp
  )
  target_link_libraries(test_PassageTraversalRepro
    ${PROJECT_NAME}_lib
  )
  ament_add_gtest(test_ConsecutiveMapMatcher
    test/test_ConsecutiveMapMatcher.cpp
  )
  target_link_libraries(test_ConsecutiveMapMatcher
    ${PROJECT_NAME}_lib
  )

  # Boost serialization round-trip self-consistency over
  # all Boost sites (Map/MapPoint/Atlas/ImuTypes/KeyFrameDatabase/KeyFrame/
  # SerializationUtils + 3 CameraModels). ROS/Gazebo-free, links production
  # library like test_atlas_lock_order.
  ament_add_gtest(test_SerializationRoundTrip
    test/test_SerializationRoundTrip.cpp
  )
  target_link_libraries(test_SerializationRoundTrip
    ${PROJECT_NAME}_lib
  )

  # Skeleton fault-injection registry + probe-macro test. Test-only
  # TU (never linked into ${PROJECT_NAME}_lib); the toggle is enabled for
  # this target only so production builds keep zero-cost disabled probes.
  ament_add_gtest(test_FaultInjection
    test/test_FaultInjection.cpp
    ${VS_GRAPHS_SEMANTIC_FAULTINJECTION_SOURCES}
  )
  target_compile_definitions(test_FaultInjection PRIVATE
    VS_GRAPHS_ENABLE_FAULT_INJECTION=1
  )

  # Small command-line tool (not a test) that replays a corpus directory and
  # writes the bounded LegacyReplayResult::report JSON to a file, so a
  # Phase-1 verification session can produce legacy_replay_report.json
  # without a gtest harness.
  add_executable(legacy_capture_replay
    test/legacy_capture_replay_main.cc
    test/LegacyCaptureReplay.cc
  )
  target_link_libraries(legacy_capture_replay
    ${PROJECT_NAME}_lib
  )
endif()

# Ament package configuration
ament_package()
