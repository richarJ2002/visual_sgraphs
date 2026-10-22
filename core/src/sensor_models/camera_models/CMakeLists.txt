# CameraModels module aggregation: each object subdirectory declares exactly
# its own sources; this file aggregates them into the module lists consumed
# by the root CMakeLists.txt. No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/geometric_camera/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/pinhole/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/kannala_brandt8/CMakeLists.txt)

set(VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_SOURCES
  ${VS_GRAPHS_CAMERAMODELS_GEOMETRICCAMERA_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_KANNALABRANDT8_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_PINHOLE_SOURCES}
)

set(VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_OBJECT_HEADERS
  ${CMAKE_CURRENT_LIST_DIR}/geometric_camera/geometric_camera.h
  ${CMAKE_CURRENT_LIST_DIR}/geometric_camera/objects/GeometricCameraClass.h
  ${CMAKE_CURRENT_LIST_DIR}/kannala_brandt8/kannala_brandt8.h
  ${CMAKE_CURRENT_LIST_DIR}/kannala_brandt8/objects/KannalaBrandt8Class.h
  ${CMAKE_CURRENT_LIST_DIR}/pinhole/pinhole.h
  ${CMAKE_CURRENT_LIST_DIR}/pinhole/objects/PinholeClass.h
)
