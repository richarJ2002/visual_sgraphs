# Visibility rules: one check per file, listed in visibility_rule_table.h, plus
# the wall-separation test they share; exactly this package's translation
# units.
set(VS_GRAPHS_SEMANTICCONSTRAINTS_VISIBILITYRULES_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/hasSeparatingFiniteWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isSeenThroughAnyWall.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isSeenThroughPassage.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/isSeenThroughSolidWall.cc
)
