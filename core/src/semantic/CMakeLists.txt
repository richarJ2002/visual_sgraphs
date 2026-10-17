# Semantic object packages: each object subdirectory declares exactly
# its own sources; this file aggregates the production ones. FaultInjection is
# test-only and is consumed by the test_FaultInjection target alone.
include(${CMAKE_CURRENT_LIST_DIR}/room/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/passage/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/floor/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/marker/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/room_tracker/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_verify/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_verify_results/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_candidates/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/sha256_digest/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/fault_injection/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_axiom_evaluator/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_canonical_serilization/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_diagnostics/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_graph_snapshot/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/semantic_report_cache/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/value_order/CMakeLists.txt)

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
  ${VS_GRAPHS_SEMANTIC_SEMANTICAXIOMEVALUATOR_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICCANONICALSERILIZATION_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICDIAGNOSTICS_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICGRAPHSNAPSHOT_SOURCES}
  ${VS_GRAPHS_SEMANTIC_SEMANTICREPORTCACHE_SOURCES}
  ${VS_GRAPHS_SEMANTIC_VALUEORDER_SOURCES}
)
