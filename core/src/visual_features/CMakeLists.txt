# visual_features capability dispatcher: each module declares its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/orb_extractor/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/orb_matcher/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/stereo_match_outlier_rejection/CMakeLists.txt)
