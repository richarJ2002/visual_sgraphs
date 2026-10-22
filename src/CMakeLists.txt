# Frontend modules own their source lists; the two entry sets never overlap.
include(${CMAKE_CURRENT_LIST_DIR}/common/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/sparse/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/observability/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/image_grabber/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/imu_grabber/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/rgbd/CMakeLists.txt)
