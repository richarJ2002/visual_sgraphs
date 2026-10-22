# scene_understanding capability dispatcher: each module declares its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/geo_sem_helpers/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_segmentation/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantics_manager/CMakeLists.txt)
