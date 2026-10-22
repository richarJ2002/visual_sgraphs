# dependency_lineage capability dispatcher: each module declares its own sources.
# g2o is not listed: the root includes it first so its private include
# roots stay quarantined.
include(${CMAKE_CURRENT_LIST_DIR}/d_utils/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/dbow2/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/nlohmann/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/pcl_custom/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/sophus/CMakeLists.txt)
