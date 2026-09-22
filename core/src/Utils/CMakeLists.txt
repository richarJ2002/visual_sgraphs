# Utils module aggregation: each object subdirectory declares exactly
# its own sources; this file aggregates them into the module lists consumed
# by the root CMakeLists.txt. No source file is listed twice.
include(${CMAKE_CURRENT_LIST_DIR}/Converter/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Settings/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Config/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/Utils/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/GeoSemHelpers/CMakeLists.txt)

set(VS_GRAPHS_UTILS_SOURCES
  ${VS_GRAPHS_UTILS_UTILS_SOURCES}
  ${VS_GRAPHS_UTILS_CONVERTER_SOURCES}
  ${VS_GRAPHS_UTILS_SETTINGS_SOURCES}
  ${VS_GRAPHS_UTILS_CONFIG_SOURCES}
  ${VS_GRAPHS_UTILS_GEOSEMHELPERS_SOURCES}
)
