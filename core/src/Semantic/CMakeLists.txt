# Semantic object packages (§4.5): each object subdirectory declares exactly
# its own sources; this file aggregates the production ones. FaultInjection is
# test-only and is consumed by the test_FaultInjection target alone.
include(${CMAKE_CURRENT_LIST_DIR}/Room/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Passage/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Floor/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Marker/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/RoomTracker/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SemanticVerify/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SemanticVerifyResult/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SemanticCandidates/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Sha256Digest/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/FaultInjection/CMakeLists.txt)

set(VS_GRAPHS_SEMANTIC_OBJECTS_SOURCES
  ${VS_GRAPHS_SEMANTIC_ROOM_SOURCES}
  ${VS_GRAPHS_SEMANTIC_PASSAGE_SOURCES}
  ${VS_GRAPHS_SEMANTIC_FLOOR_SOURCES}
  ${VS_GRAPHS_SEMANTIC_MARKER_SOURCES}
  ${VS_GRAPHS_SEMANTIC_ROOMTRACKER_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICVERIFY_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICVERIFYRESULT_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICCANDIDATES_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SHA256DIGEST_SOURCES}
)
