cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

project(vs_graphs)

# Symbol-aware tools (clang-rename, clang-tidy) read the compile database.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Preserve the existing g2o/Eigen allocation ABI on both sides of the library
# boundary; changing C++17 over-aligned allocation for only one side is unsafe.
add_compile_definitions(EIGEN_HAS_CXX17_OVERALIGN=0)

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

# Headers live with their owning source packages; generated and vendored
# include roots are supplied separately below.
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/core/src
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${segmenter_ros_INCLUDE_DIRS}
)

# Vendored code is not project-owned (CPP_CODING_STANDARD.md sections 1.2 and
# 15.3), so it is exposed as SYSTEM: diagnostics raised inside its headers
# (e.g. g2o's use of the deprecated Eigen::AlignedBit) are not reported.
# Only these roots are affected; project-owned headers under core/src keep
# the full warning set, including Sophus reached through legacy headers.
include_directories(SYSTEM
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_generated
  ${PROJECT_SOURCE_DIR}/core/src/dependency_lineage/g2o
)
include_directories(${OPENGL_INCLUDE_DIRS})

# ROS2 uses ament_target_dependencies instead of catkin_INCLUDE_DIRS
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Quarantine the vendored lists' private include roots. Their targets retain
# those includes, while owned targets use only the authorized SYSTEM roots.
get_directory_property(VS_GRAPHS_OWNED_INCLUDE_DIRS INCLUDE_DIRECTORIES)
# Include the relocated g2o CMakeLists
include(${PROJECT_SOURCE_DIR}/core/src/dependency_lineage/g2o/CMakeLists.txt)
set_property(DIRECTORY PROPERTY INCLUDE_DIRECTORIES
  ${VS_GRAPHS_OWNED_INCLUDE_DIRS})

# Keep every library of this package in its own build tree, so the Debug,
# Release and sanitizer builds do not share or overwrite source-tree outputs.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_target_properties(g2o PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Capability dispatchers only set source lists; the main library below
# fixes the flattened source order independently of this visit order.
include(${PROJECT_SOURCE_DIR}/core/src/configuration/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/dependency_lineage/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/estimation/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/execution/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/map_state/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/mapping/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/optimization/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/place_recognition_and_loop_closure/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/scene_understanding/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/sensor_models/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/views_io/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/src/visual_features/CMakeLists.txt)

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
  ${VS_GRAPHS_IMUTYPES_SOURCES}
  ${VS_GRAPHS_G2OTYPES_SOURCES}
  ${VS_GRAPHS_CAMERAMODELS_CAMERAMODELS_SOURCES}
  ${VS_GRAPHS_OPTIMIZABLETYPES_SOURCES}
  ${VS_GRAPHS_MLPNPSOLVER_SOURCES}
  ${VS_GRAPHS_GEOMETRIC_SOURCES}
  ${VS_GRAPHS_SEMANTIC_OBJECTS_SOURCES}
  ${VS_GRAPHS_TWOVIEWRECONSTRUCTION_SOURCES}
  ${VS_GRAPHS_UTILS_UTILS_SOURCES}
  ${VS_GRAPHS_UTILS_CONVERTER_SOURCES}
  ${VS_GRAPHS_UTILS_SETTINGS_SOURCES}
  ${VS_GRAPHS_UTILS_CONFIG_SOURCES}
  ${VS_GRAPHS_UTILS_GEOSEMHELPERS_SOURCES}

  ${VS_GRAPHS_DATABASEPARSER_SOURCES}
  ${VS_GRAPHS_TYPES_SOURCES}
  ${VS_GRAPHS_SEMANTICSEGMENTATION_SOURCES}
  ${VS_GRAPHS_SEMANTICSMANAGER_SOURCES}
  ${VS_GRAPHS_PCLCUSTOM_SOURCES}
  ${VS_GRAPHS_DUTILS_SOURCES}
  ${VS_GRAPHS_DBOW2_BOWVECTOR_SOURCES}
  ${VS_GRAPHS_DBOW2_FEATUREVECTOR_SOURCES}
  ${VS_GRAPHS_DBOW2_DESCRIPTORS_SOURCES}
  ${VS_GRAPHS_DBOW2_SCORING_SOURCES}
  ${VS_GRAPHS_DBOW2_VOCABULARY_SOURCES}
  ${VS_GRAPHS_SOPHUS_SOURCES}
  ${VS_GRAPHS_NLOHMANN_SOURCES}

)

# The library keeps one function per source file; each small file still parses
# the full third-party header set, which dominates the compile time. A unity
# build compiles neighbouring files of the same list together (measured on 16
# KeyFrame methods: 111 s separately, 8 s together, same peak memory). Only
# this library is batched: ROS-generated interface code is not unity-safe.
option(VS_GRAPHS_UNITY_BUILD "Compile the library sources in unity batches" ON)
if(VS_GRAPHS_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_lib PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 16)
  # A source with file-local definitions (anonymous namespace, or a
  # namespace-scope static/const/constexpr variable) compiles on its own, so
  # equal names never meet in one unity unit.
  get_target_property(VS_GRAPHS_LIB_SOURCES ${PROJECT_NAME}_lib SOURCES)
  foreach(source IN LISTS VS_GRAPHS_LIB_SOURCES)
    if(IS_ABSOLUTE "${source}")
      set(scan_source "${source}")
    else()
      set(scan_source "${PROJECT_SOURCE_DIR}/${source}")
    endif()
    file(STRINGS "${scan_source}" file_local
         REGEX "^[ \t]*namespace[ \t]*(\\{.*)?$|^(static|const|constexpr)[ \t][^(]*=")
    if(file_local)
      set_source_files_properties(${source} PROPERTIES
        SKIP_UNITY_BUILD_INCLUSION ON)
    endif()
  endforeach()
endif()

# The core library declares no name that hides another (a local hiding a
# local, parameter or member); keep it that way. The -Werror checks turn
# silent behaviour changes into build errors: an overriding function without
# `override` (renaming a base virtual would leave it unused), self-recursion
# from a half-renamed CRTP accessor, deleting an incomplete type (its
# destructor would not run) and a discarded [[nodiscard]] result.
set(VS_GRAPHS_STRICT_WARNINGS
  -Wall -Wextra -Wpedantic -Wshadow
  -Wsuggest-override -Werror=suggest-override
  -Werror=infinite-recursion
  -Werror=delete-incomplete
  -Werror=unused-result)
target_compile_options(${PROJECT_NAME}_lib PRIVATE ${VS_GRAPHS_STRICT_WARNINGS})

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  g2o
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

include(${PROJECT_SOURCE_DIR}/src/CMakeLists.txt)

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

function(add_ros2_node target_name)
  add_executable(${target_name}
    ${VS_GRAPHS_FRONTEND_COMMON_SOURCES}
    ${VS_GRAPHS_FRONTEND_SPARSE_SOURCES}
    ${ARGN}
  )

  target_compile_options(${target_name} PRIVATE ${VS_GRAPHS_STRICT_WARNINGS})

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
  ${VS_GRAPHS_FRONTEND_RGBD_ENTRY_SOURCES}
  ${VS_GRAPHS_FRONTEND_RGBD_GRABBER_SOURCES}
  ${VS_GRAPHS_FRONTEND_OBSERVABILITY_SOURCES}
)

add_ros2_node(
  ros_rgbd_inertial
  ${VS_GRAPHS_FRONTEND_INERTIAL_ENTRY_SOURCES}
  ${VS_GRAPHS_FRONTEND_INERTIAL_GRABBER_SOURCES}
  ${VS_GRAPHS_FRONTEND_IMU_GRABBER_SOURCES}
)

# libvs_graphs_lib.so must also be able to find the remaining libg2o.so
# in the same install/vs_graphs/lib/ folder.
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  INSTALL_RPATH "\$ORIGIN"
)

# Install other resources
install(
  DIRECTORY launch config core/vocabulary
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
install(TARGETS g2o
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# Ament package configuration
ament_package()
