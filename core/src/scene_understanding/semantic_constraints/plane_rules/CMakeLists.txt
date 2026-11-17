# Plane rules: one check per file, listed in plane_rule_table.h; exactly this
# package's translation units.
set(VS_GRAPHS_SEMANTICCONSTRAINTS_PLANERULES_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/arePlaneRulesSatisfied.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isGroundTiltAcceptable.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isGroundWithinMainStep.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isUprightTiltAcceptable.cc
)
