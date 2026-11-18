# Semantic factors: the semantic bundle-adjustment edge types (one subpackage
# each, namespaces unchanged) and the functions that add them to a bundle
# adjustment; exactly this package's translation units.
include(${CMAKE_CURRENT_LIST_DIR}/EdgeSE3KFPointToPlane/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertex4PlaneProjectSE3Room/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertexNPlaneProjectSE3Room/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertexPlaneParallelism/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertexPlanePerpendicularity/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertexPlaneProjectPointXYZ/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeVertexPlaneProjectSE3KF/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/EdgeWallFaceThickness/CMakeLists.txt)

set(VS_GRAPHS_SEMANTICFACTORS_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/addRoomWallEdges.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/addTwinFaceThicknessEdges.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/indexPlanesById.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/resolveRoomWallPlanes.cc
  ${VS_GRAPHS_SEMANTICFACTORS_EDGESE3KFPOINTTOPLANE_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEX4PLANEPROJECTSE3ROOM_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEXNPLANEPROJECTSE3ROOM_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEXPLANEPARALLELISM_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEXPLANEPERPENDICULARITY_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEXPLANEPROJECTPOINTXYZ_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEVERTEXPLANEPROJECTSE3KF_SOURCES}
  ${VS_GRAPHS_SEMANTICFACTORS_EDGEWALLFACETHICKNESS_SOURCES}
)
