# Passage detection stage (pipeline stage 4): SemanticsManager methods that
# find, create, refresh and merge passages and keep the far-side room of each
# passage; split into subpackages, each declaring exactly its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/passage_creation/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/passage_update/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/slab_profiles/CMakeLists.txt)

set(VS_GRAPHS_PASSAGEDETECTION_SOURCES
  ${VS_GRAPHS_PASSAGEDETECTION_PASSAGECREATION_SOURCES}
  ${VS_GRAPHS_PASSAGEDETECTION_PASSAGEUPDATE_SOURCES}
  ${VS_GRAPHS_PASSAGEDETECTION_SLABPROFILES_SOURCES}
)
