# estimation capability dispatcher: each module declares its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/converter/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/mlpnp_solver/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/sim3_solver/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/tracking/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/two_view_reconstruction/CMakeLists.txt)
