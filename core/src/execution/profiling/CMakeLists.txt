# Profiling sources: the built-in function timers. Every file compiles to
# nothing (writeTimerReport.cc to an empty function) unless the build defines
# VS_GRAPHS_BUILT_TIMERS.
set(VS_GRAPHS_PROFILING_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/getInstance.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/recordCall.cc
  ${CMAKE_CURRENT_LIST_DIR}/public_functions/writeTimerReport.cc
)

# The report names each timed file relative to the package source root.
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/public_functions/writeTimerReport.cc
  PROPERTIES COMPILE_DEFINITIONS "VS_GRAPHS_SOURCE_ROOT=\"${PROJECT_SOURCE_DIR}\"")
