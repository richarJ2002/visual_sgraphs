/*!
 * @file            test_MissionHealthTopologyJson.cpp
 *
 * @brief           Unit tests for the mission-health topology JSON
 *                  (MissionHealthTopologyJson).
 */

/*
 * Focused, ROS/Gazebo-free tests for
 * augmentMissionHealthTopologyJsonWithSemantics(), the pure
 * function extending /vs_graphs/get_mission_health's schema-1 topology_json
 * to schema 2.
 */

#include "MissionHealthTopologyJson.h"

#include <chrono>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Checks that an unavailable semantic cache only adds the
 *                  schema number and the availability flag to the schema-1
 *                  JSON.
 */
TEST(MissionHealthTopologyJson, CacheUnavailableOnlyAddsSchemaAndAvailability)
{
    nlohmann::json schema1 = {{"schema", 1}, {"map_id", 3U}, {"rooms", {}}};
    nlohmann::json result{};
    ASSERT_EQ((augmentMissionHealthTopologyJsonWithSemantics(
                  schema1,
                  semantic::SemanticReportCacheEntry(),
                  /*cacheAvailable_in=*/false,
                  result)),
              MissionHealthTopologyJsonStatus::
                  MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS);

    EXPECT_EQ(result["schema"], 2);
    EXPECT_EQ(result["semanticCacheAvailable"], false);
    EXPECT_EQ(result["map_id"], 3U);
    EXPECT_FALSE(result.contains("semanticAggregates"));
    EXPECT_FALSE(result.contains("semanticViolations"));
    EXPECT_FALSE(result.contains("semanticCacheAgeMs"));
}

/*!
 * @brief           Checks that every schema-1 field keeps its name, type and
 *                  value after the semantic additions are merged in.
 */
TEST(MissionHealthTopologyJson, PreservesEverySchema1FieldAndType)
{
    nlohmann::json       schema1  = {{"schema", 1},
                                     {"map_id", 3U},
                                     {"active_maps", 1U},
                                     {"reset_count", 0U},
                                     {"confirmed_rooms", 2},
                                     {"unresolved_rooms", 0},
                                     {"floor_room_links", 1U},
                                     {"rooms", nlohmann::json::array({1, 2})},
                                     {"floors", nlohmann::json::array()},
                                     {"passages", nlohmann::json::array()}};
    const nlohmann::json original = schema1;

    /* An available cache always has an update instant; the default
     * (time_point::min()) would make the age computation overflow. */
    semantic::SemanticReportCacheEntry updatedEntry{};
    updatedEntry.updateInstant = std::chrono::steady_clock::now();

    nlohmann::json result{};
    ASSERT_EQ((augmentMissionHealthTopologyJsonWithSemantics(schema1,
                                                             updatedEntry,
                                                             true,
                                                             result)),
              MissionHealthTopologyJsonStatus::
                  MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS);

    for (nlohmann::json::const_iterator it = original.begin();
         it != original.end();
         ++it)
    {
        if (it.key() == "schema")
        {
            continue;
        }
        ASSERT_TRUE(result.contains(it.key())) << it.key();
        EXPECT_EQ(result[it.key()], it.value()) << it.key();
        EXPECT_EQ(result[it.key()].type(), it.value().type()) << it.key();
    }
    EXPECT_EQ(result["schema"], 2);
}

/*!
 * @brief           Checks that an available cache adds the readable evaluator
 *                  fields, and that map id 0 is kept as a real map.
 */
TEST(MissionHealthTopologyJson, AvailableCacheAddsReadableEvaluatorAdditions)
{
    semantic::SemanticReportCacheEntry entry;
    entry.semanticCycle  = 42U;
    entry.updateSequence = 7U;
    entry.currentMapId   = 0UL; /* map id 0 is a real, valid map -- must not
                                 * be dropped as if it meant "absent". */
    entry.mapRevision                 = 5;
    entry.geometryRevision            = 2U;
    entry.canonicalTopologyDigest     = "topo-digest";
    entry.canonicalFullGeometryDigest = "geo-digest";

    semantic::AggregateAxiomResult aggregate;
    aggregate.axiomCode                = semantic::AxiomCode::AX_WALL_01;
    aggregate.result                   = semantic::AxiomResult::FAIL;
    aggregate.classification           = semantic::AxiomClass::HARD;
    aggregate.contributingFindingCount = 1U;
    entry.evaluationReport.aggregates.push_back(aggregate);

    semantic::Finding failFinding;
    failFinding.id             = "f1";
    failFinding.axiomCode      = semantic::AxiomCode::AX_WALL_01;
    failFinding.result         = semantic::AxiomResult::FAIL;
    failFinding.classification = semantic::AxiomClass::HARD;
    failFinding.reasonCode     = semantic::ReasonCode::WALL_OWNERSHIP_OWNER_BAD;
    entry.evaluationReport.findings.push_back(failFinding);

    semantic::Finding passFinding;
    passFinding.id     = "f2";
    passFinding.result = semantic::AxiomResult::PASS;
    entry.evaluationReport.findings.push_back(passFinding);

    semantic::MapCompletenessResult completeness;
    completeness.mapId              = 0U;
    completeness.isComplete         = false;
    completeness.conservativeResult = semantic::AxiomResult::UNKNOWN;
    completeness.reasons            = {
        semantic::ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS,
        semantic::ReasonCode::WALL_OWNERSHIP_OWNER_BAD};
    completeness.relevantEntityKeys = {
        semantic::EntityKey{semantic::EntityKind::ROOM, 0U, 2},
        semantic::EntityKey{semantic::EntityKind::ROOM, 0U, 1}};
    entry.completenessResults.push_back(completeness);

    /* A synthetic, already-elapsed instant (no real sleep, so this can
     * never be flaky) to prove semanticCacheAgeMs is genuinely positive
     * and monotonic with elapsed wall time, not merely present/zero. */
    entry.updateInstant =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(50);

    nlohmann::json result{};
    ASSERT_EQ((augmentMissionHealthTopologyJsonWithSemantics({{"schema", 1}},
                                                             entry,
                                                             true,
                                                             result)),
              MissionHealthTopologyJsonStatus::
                  MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS);

    EXPECT_EQ(result["schema"], 2);
    EXPECT_EQ(result["semanticCacheAvailable"], true);
    EXPECT_GE(result["semanticCacheAgeMs"].get<std::int64_t>(), 50);
    EXPECT_EQ(result["semanticCycle"], 42U);
    EXPECT_EQ(result["semanticUpdateSequence"], 7U);
    ASSERT_TRUE(result.contains("semanticCurrentMapId"));
    EXPECT_EQ(result["semanticCurrentMapId"], 0U);
    EXPECT_EQ(result["semanticMapRevision"], 5);
    EXPECT_EQ(result["semanticGeometryRevision"], 2U);
    EXPECT_EQ(result["semanticTopologyDigest"], "topo-digest");
    EXPECT_EQ(result["semanticFullGeometryDigest"], "geo-digest");

    ASSERT_EQ(result["semanticAggregates"].size(), 1U);
    EXPECT_EQ(result["semanticAggregates"][0]["axiomCode"], "AX_WALL_01");
    EXPECT_EQ(result["semanticAggregates"][0]["result"], "FAIL");
    EXPECT_EQ(result["semanticAggregates"][0]["classification"], "HARD");

    /* Only the FAIL finding is a "current violation". */
    ASSERT_EQ(result["semanticViolations"].size(), 1U);
    EXPECT_EQ(result["semanticViolations"][0]["findingId"], "f1");
    EXPECT_EQ(result["semanticViolations"][0]["reasonCode"],
              "WALL_OWNERSHIP_OWNER_BAD");

    ASSERT_EQ(result["semanticMapCompleteness"].size(), 1U);
    EXPECT_EQ(result["semanticMapCompleteness"][0]["mapId"], 0U);
    EXPECT_EQ(result["semanticMapCompleteness"][0]["conservativeResult"],
              "UNKNOWN");
    const nlohmann::json &reasonsJson =
        result["semanticMapCompleteness"][0]["reasons"];
    ASSERT_EQ(reasonsJson.size(), 2U);
    /* WALL_OWNERSHIP_MULTIPLE_OWNERS (4) sorts before WALL_OWNERSHIP_OWNER_
     * BAD (5) by underlying semantic::ReasonCode value, regardless of the input
     * vector's own order. */
    EXPECT_EQ(reasonsJson[0], "WALL_OWNERSHIP_MULTIPLE_OWNERS");
    EXPECT_EQ(reasonsJson[1], "WALL_OWNERSHIP_OWNER_BAD");
    const nlohmann::json &entityKeysJson =
        result["semanticMapCompleteness"][0]["relevantEntityKeys"];
    ASSERT_EQ(entityKeysJson.size(), 2U);
    EXPECT_EQ(entityKeysJson[0]["entityId"], 1);
    EXPECT_EQ(entityKeysJson[1]["entityId"], 2);

    /* Sorted axiom capability table with readable CapabilityLevel/
     * semantic::MissingProofOwner names, present regardless of any evaluated
     * snapshot (it is a fixed property of this evaluator's implementation,
     * not of entry_in). */
    ASSERT_TRUE(result.contains("semanticAxiomCapabilities"));
    const nlohmann::json &capabilitiesJson =
        result["semanticAxiomCapabilities"];
    ASSERT_GT(capabilitiesJson.size(), 0U);
    for (const nlohmann::json &row : capabilitiesJson)
    {
        EXPECT_TRUE(row.contains("axiomCode"));
        EXPECT_TRUE(row.contains("capability"));
        EXPECT_TRUE(row.contains("missingProofOwner"));
        EXPECT_NE(row["capability"].get<std::string>().rfind("UNKNOWN_"), 0U);
        EXPECT_NE(row["missingProofOwner"].get<std::string>().rfind("UNKNOWN_"),
                  0U);
    }
    /* Sorted by the underlying semantic::AxiomCode enum value, not by name
     * string (e.g. "AX_WALL_01" == 1 sorts long before "AX_TXN_01" == 13, even
     * though "AX_TXN_01" < "AX_WALL_01" lexicographically) -- matches the
     * fixed declaration order in semantic::AxiomCode.h. */
    static const std::vector<std::string> kExpectedOrder = {"AX_FRAME_01",
                                                            "AX_WALL_01",
                                                            "AX_WALL_02",
                                                            "AX_WALL_03",
                                                            "AX_PASS_01",
                                                            "AX_PASS_02",
                                                            "AX_PASS_03",
                                                            "AX_PASS_04",
                                                            "AX_ROOM_01",
                                                            "AX_ROOM_02",
                                                            "AX_BOUND_01",
                                                            "AX_FLOOR_01",
                                                            "AX_LIFE_01",
                                                            "AX_TXN_01",
                                                            "AX_COMP_01",
                                                            "AX_MERGE_01"};
    ASSERT_EQ(capabilitiesJson.size(), kExpectedOrder.size());
    for (std::size_t i = 0U; i < kExpectedOrder.size(); ++i)
    {
        EXPECT_EQ(capabilitiesJson[i]["axiomCode"], kExpectedOrder[i])
            << "capability table index " << i;
    }
}

/*!
 * @brief           Checks that completeness reasons and entity keys are
 *                  serialized sorted, whatever order the input holds them in.
 */
TEST(MissionHealthTopologyJson,
     CompletenessReasonsAndEntityKeysAreSortedRegardlessOfInputOrder)
{
    semantic::SemanticReportCacheEntry entry;
    semantic::MapCompletenessResult    completeness;
    completeness.mapId              = 1U;
    completeness.reasons            = {semantic::ReasonCode::WALL_TWIN_SELF,
                                       semantic::ReasonCode::WALL_OWNERSHIP_OWNER_BAD};
    completeness.relevantEntityKeys = {
        semantic::EntityKey{semantic::EntityKind::WALL, 1U, 9},
        semantic::EntityKey{semantic::EntityKind::WALL, 1U, 3}};
    entry.completenessResults.push_back(completeness);
    /* An available cache always has an update instant (see above). */
    entry.updateInstant = std::chrono::steady_clock::now();

    nlohmann::json result{};
    ASSERT_EQ((augmentMissionHealthTopologyJsonWithSemantics({{"schema", 1}},
                                                             entry,
                                                             true,
                                                             result)),
              MissionHealthTopologyJsonStatus::
                  MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS);

    const nlohmann::json &reasonsJson =
        result["semanticMapCompleteness"][0]["reasons"];
    ASSERT_EQ(reasonsJson.size(), 2U);
    EXPECT_EQ(reasonsJson[0], "WALL_OWNERSHIP_OWNER_BAD");
    EXPECT_EQ(reasonsJson[1], "WALL_TWIN_SELF");

    const nlohmann::json &entityKeysJson =
        result["semanticMapCompleteness"][0]["relevantEntityKeys"];
    ASSERT_EQ(entityKeysJson.size(), 2U);
    EXPECT_EQ(entityKeysJson[0]["entityId"], 3);
    EXPECT_EQ(entityKeysJson[1]["entityId"], 9);
}

} // namespace core
} // namespace vs_graphs
