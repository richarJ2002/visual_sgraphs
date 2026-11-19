# SemanticsManager semantic monitor: the snapshot captured at the cycle end,
# its evaluation, the report cache and the diagnostics log; exactly this
# subpackage's translation units.
set(VS_GRAPHS_SEMANTICSMANAGER_MONITOR_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/captureMonitorSnapshot.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/captureUnresolvedWallHypotheses.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/getSemanticReportCacheEntry.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/isSemanticReportCacheAvailable.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/logSemanticDiagnostics.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/reportMonitorEvaluation.cc
)
