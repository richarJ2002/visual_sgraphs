# Plane rules: one check per file, listed in plane_rule_table.h; exactly this
# package's translation units.
set(VS_GRAPHS_SEMANTICCONSTRAINTS_PLANERULES_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/private_functions/computeLevelTilt.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/arePlaneRulesSatisfied.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/assignPlaneLabel.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isGroundTiltAcceptable.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isGroundWithinMainStep.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isUprightTiltAcceptable.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/labelLevelPlane.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/labelUprightPlane.cc
)
