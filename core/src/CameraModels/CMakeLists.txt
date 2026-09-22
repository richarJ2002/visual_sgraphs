# CameraModels module aggregation: each object subdirectory declares exactly
# its own sources; this file aggregates them into the module lists consumed
# by the root CMakeLists.txt. No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/GeometricCamera/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Pinhole/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/KannalaBrandt8/CMakeLists.txt)

set(VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_SOURCES
  ${VS_GRAPHS_CAMERAMODELS_GEOMETRICCAMERA_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_KANNALABRANDT8_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_PINHOLE_SOURCES}
)

set(VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_OBJECT_HEADERS
  core/include/CameraModels/GeometricCamera/objects/GeometricCamera.h
  core/include/CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h
  core/include/CameraModels/Pinhole/objects/Pinhole.h
  core/include/CameraModels/objects.h
  core/include/CameraModels/CameraModels.h
)
