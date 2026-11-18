# Room association stage (pipeline stage 5): SemanticsManager methods that give
# walls to rooms and rooms to passages; split into subpackages, each declaring
# exactly its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/association_cycle/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/free_space_rooms/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/passage_rooms/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/room_consolidation/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/wall_admission/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/wall_ownership/CMakeLists.txt)

set(VS_GRAPHS_ROOMASSOCIATION_SOURCES
  ${VS_GRAPHS_ROOMASSOCIATION_ASSOCIATIONCYCLE_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_FREESPACEROOMS_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_PASSAGEROOMS_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_ROOMCONSOLIDATION_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_WALLADMISSION_SOURCES}
  ${VS_GRAPHS_ROOMASSOCIATION_WALLOWNERSHIP_SOURCES}
)
