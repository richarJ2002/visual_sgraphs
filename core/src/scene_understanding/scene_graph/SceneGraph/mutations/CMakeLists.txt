# SceneGraph mutation methods: exactly this subpackage's translation units.
set(VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_MUTATIONS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/addCandidateRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/addDetectedRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/addFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/addPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/eraseFloor.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/erasePassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/eraseRoom.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/promoteCandidateRoom.cc
)
