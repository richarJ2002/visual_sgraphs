# Managed source list for the Types module (objects/methods layout).
# Module: Types
# Layout: core/include/Types/objects/*.h (+ Types aggregates) and
#   core/src/Types/methods/<Object>/*.cc.
# This file is included by CMakeLists.txt. The SOURCES list below is
# equivalent to the recursive discovery of core/src/Types (verified with
# scripts/generate_cmake_sources.py --module Types); OBJECT_HEADERS lists
# the object header plus the module aggregates.
# Layout breakdown (.cc per subdir): {'methods': 3}

cmake_minimum_required(VERSION 3.5)

set(VS_GRAPHS_TYPES_TYPES_SOURCES
  core/src/Types/methods/SystemParams/getParams.cc
  core/src/Types/methods/SystemParams/setParams.cc
  core/src/Types/methods/SystemParams/static_members.cc
)

set(VS_GRAPHS_TYPES_TYPES_OBJECT_HEADERS
  core/include/Types/objects/SystemParams.h
  core/include/Types/objects.h
  core/include/Types/Types.h
)
