# MissionHealthTopologyJson module sources: free functions (§5.4), flat
# layout per the Semantic/ValueOrder precedent. No source file is listed
# twice. Former anonymous-namespace helpers are same-namespace internal
# functions so the split translation units can share them.
set(VS_GRAPHS_MISSIONHEALTHTOPOLOGYJSON_SOURCES
  core/src/MissionHealthTopologyJson/entityKeysToJson.cc
  core/src/MissionHealthTopologyJson/finiteAwareDoubleToJson.cc
  core/src/MissionHealthTopologyJson/axiomCapabilitiesToJson.cc
  core/src/MissionHealthTopologyJson/augmentMissionHealthTopologyJsonWithSemantics.cc
)
