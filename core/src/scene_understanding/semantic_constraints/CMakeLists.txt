# Semantic constraints dispatcher: each rule package declares its own sources.
include(${CMAKE_CURRENT_LIST_DIR}/plane_rules/CMakeLists.txt)

set(VS_GRAPHS_SEMANTICCONSTRAINTS_SOURCES
  ${VS_GRAPHS_SEMANTICCONSTRAINTS_PLANERULES_SOURCES}
)
