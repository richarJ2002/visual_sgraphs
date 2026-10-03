/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            test_LegacyCaptureReplay.cpp
 *
 * @brief           GTests for the truthful deterministic legacy capture
 *                  replay: valid
 *                  parsing, malformed-file reporting, permutation
 *                  invariance, topology/geometry digest independence, never
 *                  fabricating PASS, and the actual 147-file acceptance
 *                  corpus.
 */

#include "test/LegacyCaptureReplay.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#ifndef VS_GRAPHS_WORKSPACE_ROOT
#error "VS_GRAPHS_WORKSPACE_ROOT must be defined by CMakeLists.txt"
#endif

namespace vs_graphs
{
namespace core
{
namespace semantic
{
namespace
{

nlohmann::json makeValidCaptureJson()
{
    return nlohmann::json::parse(R"({
        "scene": "vs_graphs_generated",
        "rooms": [
            {"id": 1, "centroid_xy": [-0.4, -1.1]},
            {"id": 2, "centroid_xy": [-0.2, 4.1]}
        ],
        "walls": [
            {"room_id": 1, "normal": [0.99, 0.03, 0.0], "offset_d": 1.9, "extent_m": 0.8},
            {"room_id": 2, "normal": [0.03, -0.99, 0.06], "offset_d": 6.3, "extent_m": 2.0}
        ],
        "passages": [],
        "floor": {"normal": [0.0, 0.02, -0.99], "offset_d": 1.4}
    })");
}

std::filesystem::path makeTempDir()
{
    std::filesystem::path dir =
        std::filesystem::temp_directory_path() /
        ("legacy_replay_test_" +
         std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
         "_" + std::to_string(std::rand()));
    std::filesystem::create_directories(dir);
    return dir;
}

void writeFile(const std::filesystem::path &path_in,
               const nlohmann::json        &json_in)
{
    std::ofstream out(path_in);
    out << json_in.dump();
}

} // namespace

/*!
 * @brief           Checks that replaying a nonexistent directory returns an
 *                  empty, valid result instead of failing.
 */
TEST(LegacyCaptureReplay, ReturnsValidResultForNonexistentDirectory)
{
    const LegacyReplayResult result = replayLegacyCaptures("/nonexistent/path");

    EXPECT_EQ(result.totalFiles, 0U);
    EXPECT_EQ(result.okFiles, 0U);
    EXPECT_EQ(result.malformedFiles, 0U);
    EXPECT_TRUE(result.fileResults.empty());
}

/*!
 * @brief           Checks that a well-formed capture parses without the
 *                  malformed flag.
 */
TEST(LegacyCaptureReplay, ParsesAValidCaptureWithoutMalformedFlag)
{
    const LegacyParseResult parsed = parseLegacyCapture(makeValidCaptureJson());

    ASSERT_FALSE(parsed.malformed);
    EXPECT_EQ(parsed.capture.rooms.size(), 2U);
    EXPECT_EQ(parsed.capture.walls.size(), 2U);
    EXPECT_EQ(parsed.capture.passageCount, 0U);
    ASSERT_TRUE(parsed.capture.floor.has_value());
}

/*!
 * @brief           Checks that a capture with a missing or wrongly typed
 *                  required field is reported malformed rather than silently
 *                  dropped.
 */
TEST(LegacyCaptureReplay,
     MissingRequiredFieldIsReportedMalformedNotSilentlyDropped)
{
    nlohmann::json missingRooms = makeValidCaptureJson();
    missingRooms.erase("rooms");
    const LegacyParseResult parsed = parseLegacyCapture(missingRooms);
    EXPECT_TRUE(parsed.malformed);
    EXPECT_FALSE(parsed.malformedReason.empty());

    nlohmann::json wrongTypeWall          = makeValidCaptureJson();
    wrongTypeWall["walls"][0]["extent_m"] = "not-a-number";
    EXPECT_TRUE(parseLegacyCapture(wrongTypeWall).malformed);

    nlohmann::json nonEmptyPassages = makeValidCaptureJson();
    nonEmptyPassages["passages"].push_back({{"unknown", "shape"}});
    EXPECT_TRUE(parseLegacyCapture(nonEmptyPassages).malformed);
}

/*!
 * @brief           Checks that replayed axiom results never contain PASS, carry
 *                  a reason, and list each axiom code once.
 */
TEST(LegacyCaptureReplay, NeverEmitsPassOnlyFailOrUnknown)
{
    const LegacyParseResult parsed = parseLegacyCapture(makeValidCaptureJson());
    ASSERT_FALSE(parsed.malformed);

    const std::vector<LegacyAxiomResultRecord> axiomResults =
        evaluateLegacyAxioms(parsed.capture);
    ASSERT_FALSE(axiomResults.empty());
    for (const LegacyAxiomResultRecord &record : axiomResults)
    {
        EXPECT_NE(record.result, AxiomResult::PASS);
        EXPECT_FALSE(record.reason.empty());
    }
    /* Every current axiom code appears exactly once (the capability table
     * is the evaluator's own canonical catalogue). */
    std::vector<AxiomCode> codes;
    for (const LegacyAxiomResultRecord &record : axiomResults)
    {
        codes.push_back(record.axiomCode);
    }
    std::sort(codes.begin(), codes.end());
    EXPECT_EQ(std::adjacent_find(codes.begin(), codes.end()), codes.end())
        << "duplicate axiom code in replay result";
}

/*!
 * @brief           Checks that a wall whose room_id names no room in the file
 *                  is deterministically reported as FAIL for the wall-ownership
 *                  axiom.
 */
TEST(LegacyCaptureReplay,
     UnresolvableWallOwnerIsReportedAsFailDeterministically)
{
    nlohmann::json broken         = makeValidCaptureJson();
    broken["walls"][0]["room_id"] = 99;

    const LegacyParseResult parsed = parseLegacyCapture(broken);
    ASSERT_FALSE(parsed.malformed);

    const std::vector<LegacyAxiomResultRecord> axiomResults =
        evaluateLegacyAxioms(parsed.capture);
    bool foundFail = false;
    for (const LegacyAxiomResultRecord &record : axiomResults)
    {
        if (record.axiomCode == AxiomCode::AX_WALL_01)
        {
            EXPECT_EQ(record.result, AxiomResult::FAIL);
            foundFail = true;
        }
        else
        {
            EXPECT_EQ(record.result, AxiomResult::UNKNOWN);
        }
    }
    EXPECT_TRUE(foundFail);
}

/*!
 * @brief           Checks that reordering the JSON arrays of a capture leaves
 *                  both digests unchanged.
 */
TEST(LegacyCaptureReplay, JsonArrayPermutationProducesIdenticalDigests)
{
    nlohmann::json      captureJson = makeValidCaptureJson();
    const LegacyCapture original    = parseLegacyCapture(captureJson).capture;

    LegacyCapture reversed = original;
    std::reverse(reversed.rooms.begin(), reversed.rooms.end());
    std::reverse(reversed.walls.begin(), reversed.walls.end());

    EXPECT_EQ(legacyTopologyDigest(original), legacyTopologyDigest(reversed));
    EXPECT_EQ(legacyFullGeometryDigest(original),
              legacyFullGeometryDigest(reversed));
}

/*!
 * @brief           Checks that a geometry-only change alters the full geometry
 *                  digest but not the topology digest.
 */
TEST(LegacyCaptureReplay, GeometryOnlyDriftChangesOnlyFullGeometryDigest)
{
    const LegacyCapture near =
        parseLegacyCapture(makeValidCaptureJson()).capture;

    nlohmann::json farJson             = makeValidCaptureJson();
    farJson["rooms"][0]["centroid_xy"] = {500.0, 500.0};
    farJson["walls"][0]["extent_m"]    = 999.0;
    const LegacyCapture far            = parseLegacyCapture(farJson).capture;

    EXPECT_EQ(legacyTopologyDigest(near), legacyTopologyDigest(far));
    EXPECT_NE(legacyFullGeometryDigest(near), legacyFullGeometryDigest(far));
}

/*!
 * @brief           Checks that replaying a directory twice gives the same
 *                  report, with real file counts and results sorted by file
 *                  name.
 */
TEST(LegacyCaptureReplay, DirectoryReplayIsDeterministicAndReportsRealCounts)
{
    const std::filesystem::path dir = makeTempDir();
    writeFile(dir / "capture_attempt_b.json", makeValidCaptureJson());
    nlohmann::json malformedJson = makeValidCaptureJson();
    malformedJson.erase("walls");
    writeFile(dir / "capture_attempt_a_malformed.json", malformedJson);

    const LegacyReplayResult first  = replayLegacyCaptures(dir);
    const LegacyReplayResult second = replayLegacyCaptures(dir);

    ASSERT_EQ(first.totalFiles, 2U);
    EXPECT_EQ(first.okFiles, 1U);
    EXPECT_EQ(first.malformedFiles, 1U);
    /* No fake zero-count SUCCESS: the one OK file has real nonzero counts. */
    const std::vector<LegacyFileReplayResult>::const_iterator okIterator =
        std::find_if(first.fileResults.begin(),
                     first.fileResults.end(),
                     [](const LegacyFileReplayResult &r)
                     { return !r.malformed; });
    ASSERT_NE(okIterator, first.fileResults.end());
    EXPECT_GT(okIterator->roomCount, 0U);
    EXPECT_GT(okIterator->wallCount, 0U);

    /* Deterministic regardless of directory_iterator's own order: two
     * independent replay calls over the same directory produce
     * byte-identical reports, and results are sorted by filename. */
    EXPECT_EQ(first.report.dump(), second.report.dump());
    ASSERT_EQ(first.fileResults.size(), 2U);
    EXPECT_TRUE(std::is_sorted(first.fileResults.begin(),
                               first.fileResults.end(),
                               [](const LegacyFileReplayResult &lhs_in,
                                  const LegacyFileReplayResult &rhs_in)
                               { return lhs_in.fileName < rhs_in.fileName; }));

    std::filesystem::remove_all(dir);
}

/*!
 * @brief           Checks that the real 147-file acceptance corpus replays;
 *                  skipped when that corpus directory is unavailable.
 */
TEST(LegacyCaptureReplay, ReplaysTheReal147FileAcceptanceCorpus)
{
    const std::filesystem::path corpusDir =
        std::string(VS_GRAPHS_WORKSPACE_ROOT) +
        "/test_runs/20260905-160646_DRIFT_EVIDENCE_no_convergence";
    if (!std::filesystem::exists(corpusDir))
    {
        GTEST_SKIP() << "acceptance corpus not present at " << corpusDir;
    }

    const LegacyReplayResult result = replayLegacyCaptures(corpusDir);

    EXPECT_EQ(result.totalFiles, 147U);
    EXPECT_EQ(result.malformedFiles, 0U);
    EXPECT_EQ(result.okFiles, 147U);
    for (const LegacyFileReplayResult &fileResult : result.fileResults)
    {
        ASSERT_FALSE(fileResult.malformed) << fileResult.fileName;
        EXPECT_GT(fileResult.roomCount, 0U) << fileResult.fileName;
        EXPECT_FALSE(fileResult.topologyDigest.empty());
        EXPECT_FALSE(fileResult.fullGeometryDigest.empty());
        for (const LegacyAxiomResultRecord &axiomResult :
             fileResult.axiomResults)
        {
            EXPECT_NE(axiomResult.result, AxiomResult::PASS);
        }
    }
    EXPECT_NO_THROW(nlohmann::json::parse(result.report.dump()));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
