# SceneGraph lifecycle methods (scene maps, the active map): exactly this
# subpackage's translation units.
set(VS_GRAPHS_SCENEGRAPH_SCENEGRAPH_LIFECYCLE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/acquireReclaimGuard.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/addSceneMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/clearSemantics.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/eraseSceneMap.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getSceneGraph.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/setActiveMap.cc
)
