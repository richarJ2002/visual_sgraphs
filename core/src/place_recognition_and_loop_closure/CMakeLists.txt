# place_recognition_and_loop_closure capability dispatcher: each module declares its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/keyframe_database/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/loop_closing/CMakeLists.txt)
