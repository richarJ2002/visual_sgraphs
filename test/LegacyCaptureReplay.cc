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
 * @file            LegacyCaptureReplay.cc
 *
 * @brief           Implements the functions declared in
 *                  LegacyCaptureReplay.h (semantic-axiom-reliability-plan.md
 *                  P1.9).
 */

#include "test/LegacyCaptureReplay.h"

#include <algorithm>
#include <exception>
#include <fstream>

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"
#include "Semantic/Sha256Digest.h"
#include "Semantic/ValueOrder.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

namespace
{

bool isVector2dLess(const Eigen::Vector2d &lhs_in,
                    const Eigen::Vector2d &rhs_in)
{
    if (isDoubleLess(lhs_in.x(), rhs_in.x()))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.x(), lhs_in.x()))
    {
        return false;
    }
    return isDoubleLess(lhs_in.y(), rhs_in.y());
}

bool isLegacyRoomRecordLessFullGeometry(const LegacyRoomRecord &lhs_in,
                                        const LegacyRoomRecord &rhs_in)
{
    if (lhs_in.id != rhs_in.id)
    {
        return lhs_in.id < rhs_in.id;
    }
    return isVector2dLess(lhs_in.centroidXy, rhs_in.centroidXy);
}

bool isLegacyWallRecordLessFullGeometry(const LegacyWallRecord &lhs_in,
                                        const LegacyWallRecord &rhs_in)
{
    if (lhs_in.ownerRoomId != rhs_in.ownerRoomId)
    {
        return lhs_in.ownerRoomId < rhs_in.ownerRoomId;
    }
    if (isVector3dLess(lhs_in.normal, rhs_in.normal))
    {
        return true;
    }
    if (isVector3dLess(rhs_in.normal, lhs_in.normal))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.offsetD, rhs_in.offsetD))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.offsetD, lhs_in.offsetD))
    {
        return false;
    }
    return isDoubleLess(lhs_in.extentM, rhs_in.extentM);
}

/*! @brief Reads a fixed-length numeric JSON array into an Eigen vector;
 *  returns false (leaving \p out unmodified) when \p value_in is not an
 *  array of exactly \p Size numbers. */
template <int Size>
bool readFixedNumericArray(const nlohmann::json           &value_in,
                           Eigen::Matrix<double, Size, 1> &out_inout)
{
    if (!value_in.is_array() ||
        value_in.size() != static_cast<std::size_t>(Size))
    {
        return false;
    }
    for (int i = 0; i < Size; ++i)
    {
        if (!value_in[static_cast<std::size_t>(i)].is_number())
        {
            return false;
        }
        out_inout[i] = value_in[static_cast<std::size_t>(i)].get<double>();
    }
    return true;
}

/*! @brief Canonical JSON of the discrete (topology) facts common to both
 *  digests: sorted room-id multiset, sorted wall-owner-room-id multiset,
 *  passageCount, hasFloor. */
nlohmann::json topologyJson(const LegacyCapture &capture_in)
{
    std::vector<int> roomIds;
    roomIds.reserve(capture_in.rooms.size());
    for (const LegacyRoomRecord &room : capture_in.rooms)
    {
        roomIds.push_back(room.id);
    }
    std::sort(roomIds.begin(), roomIds.end());

    std::vector<int> wallOwnerRoomIds;
    wallOwnerRoomIds.reserve(capture_in.walls.size());
    for (const LegacyWallRecord &wall : capture_in.walls)
    {
        wallOwnerRoomIds.push_back(wall.ownerRoomId);
    }
    std::sort(wallOwnerRoomIds.begin(), wallOwnerRoomIds.end());

    nlohmann::json json;
    json["roomIds"]          = roomIds;
    json["wallOwnerRoomIds"] = wallOwnerRoomIds;
    json["passageCount"]     = capture_in.passageCount;
    json["hasFloor"]         = capture_in.floor.has_value();
    return json;
}

} // namespace

LegacyParseResult parseLegacyCapture(const nlohmann::json &json_in)
{
    LegacyParseResult result;

    if (!json_in.is_object())
    {
        result.malformed       = true;
        result.malformedReason = "top-level value is not a JSON object";
        return result;
    }
    if (json_in.contains("scene") && json_in["scene"].is_string())
    {
        result.capture.scene = json_in["scene"].get<std::string>();
    }

    if (!json_in.contains("rooms") || !json_in["rooms"].is_array())
    {
        result.malformed       = true;
        result.malformedReason = "missing or non-array \"rooms\"";
        return result;
    }
    for (const nlohmann::json &roomJson : json_in["rooms"])
    {
        if (!roomJson.is_object() || !roomJson.contains("id") ||
            !roomJson["id"].is_number_integer() ||
            !roomJson.contains("centroid_xy"))
        {
            result.malformed       = true;
            result.malformedReason = "malformed room entry (missing/wrong-"
                                     "typed \"id\" or \"centroid_xy\")";
            return result;
        }
        LegacyRoomRecord room;
        room.id = roomJson["id"].get<int>();
        if (!readFixedNumericArray<2>(roomJson["centroid_xy"], room.centroidXy))
        {
            result.malformed       = true;
            result.malformedReason = "room \"centroid_xy\" is not a 2-"
                                     "element numeric array";
            return result;
        }
        result.capture.rooms.push_back(room);
    }

    if (!json_in.contains("walls") || !json_in["walls"].is_array())
    {
        result.malformed       = true;
        result.malformedReason = "missing or non-array \"walls\"";
        return result;
    }
    for (const nlohmann::json &wallJson : json_in["walls"])
    {
        if (!wallJson.is_object() || !wallJson.contains("room_id") ||
            !wallJson["room_id"].is_number_integer() ||
            !wallJson.contains("normal") || !wallJson.contains("offset_d") ||
            !wallJson["offset_d"].is_number() ||
            !wallJson.contains("extent_m") || !wallJson["extent_m"].is_number())
        {
            result.malformed       = true;
            result.malformedReason = "malformed wall entry (missing/wrong-"
                                     "typed \"room_id\"/\"normal\"/"
                                     "\"offset_d\"/\"extent_m\")";
            return result;
        }
        LegacyWallRecord wall;
        wall.ownerRoomId = wallJson["room_id"].get<int>();
        if (!readFixedNumericArray<3>(wallJson["normal"], wall.normal))
        {
            result.malformed       = true;
            result.malformedReason = "wall \"normal\" is not a 3-element "
                                     "numeric array";
            return result;
        }
        wall.offsetD = wallJson["offset_d"].get<double>();
        wall.extentM = wallJson["extent_m"].get<double>();
        result.capture.walls.push_back(wall);
    }

    if (!json_in.contains("passages") || !json_in["passages"].is_array())
    {
        result.malformed       = true;
        result.malformedReason = "missing or non-array \"passages\"";
        return result;
    }
    if (!json_in["passages"].empty())
    {
        /* Every observed Phase-0 capture has an empty "passages" array;
         * the legacy schema documents no per-passage field shape, so a
         * non-empty array cannot be parsed without fabricating a
         * structure -- report it explicitly rather than guessing. */
        result.malformed       = true;
        result.malformedReason = "non-empty \"passages\" array has no "
                                 "documented legacy element schema";
        return result;
    }
    result.capture.passageCount = 0U;

    if (json_in.contains("floor"))
    {
        const nlohmann::json &floorJson = json_in["floor"];
        if (!floorJson.is_object() || !floorJson.contains("normal") ||
            !floorJson.contains("offset_d") ||
            !floorJson["offset_d"].is_number())
        {
            result.malformed       = true;
            result.malformedReason = "malformed \"floor\" entry (missing/"
                                     "wrong-typed \"normal\"/\"offset_d\")";
            return result;
        }
        LegacyFloorRecord floor;
        if (!readFixedNumericArray<3>(floorJson["normal"], floor.normal))
        {
            result.malformed       = true;
            result.malformedReason = "floor \"normal\" is not a 3-element "
                                     "numeric array";
            return result;
        }
        floor.offsetD        = floorJson["offset_d"].get<double>();
        result.capture.floor = floor;
    }

    return result;
}

std::string legacyTopologyDigest(const LegacyCapture &capture_in)
{
    std::string hexDigest{};
    if (sha256HexDigest(topologyJson(capture_in).dump(), hexDigest) !=
        Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
    {
        // sha256HexDigest cannot fail; continue as before.
    }
    return hexDigest;
}

std::string legacyFullGeometryDigest(const LegacyCapture &capture_in)
{
    std::vector<LegacyRoomRecord> rooms = capture_in.rooms;
    std::sort(rooms.begin(), rooms.end(), &isLegacyRoomRecordLessFullGeometry);
    nlohmann::json roomsJson = nlohmann::json::array();
    for (const LegacyRoomRecord &room : rooms)
    {
        roomsJson.push_back(
            {{"id", room.id},
             {"centroidXy", {room.centroidXy.x(), room.centroidXy.y()}}});
    }

    std::vector<LegacyWallRecord> walls = capture_in.walls;
    std::sort(walls.begin(), walls.end(), &isLegacyWallRecordLessFullGeometry);
    nlohmann::json wallsJson = nlohmann::json::array();
    for (const LegacyWallRecord &wall : walls)
    {
        wallsJson.push_back(
            {{"ownerRoomId", wall.ownerRoomId},
             {"normal", {wall.normal.x(), wall.normal.y(), wall.normal.z()}},
             {"offsetD", wall.offsetD},
             {"extentM", wall.extentM}});
    }

    nlohmann::json json = topologyJson(capture_in);
    json["rooms"]       = std::move(roomsJson);
    json["walls"]       = std::move(wallsJson);
    if (capture_in.floor.has_value())
    {
        json["floor"] = {{"normal",
                          {capture_in.floor->normal.x(),
                           capture_in.floor->normal.y(),
                           capture_in.floor->normal.z()}},
                         {"offsetD", capture_in.floor->offsetD}};
    }

    std::string hexDigest{};
    if (sha256HexDigest(json.dump(), hexDigest) !=
        Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
    {
        // sha256HexDigest cannot fail; continue as before.
    }
    return hexDigest;
}

std::vector<LegacyAxiomResultRecord>
    evaluateLegacyAxioms(const LegacyCapture &capture_in)
{
    std::vector<int> roomIds;
    roomIds.reserve(capture_in.rooms.size());
    for (const LegacyRoomRecord &room : capture_in.rooms)
    {
        roomIds.push_back(room.id);
    }

    bool hasUnresolvableOwner = false;
    for (const LegacyWallRecord &wall : capture_in.walls)
    {
        if (std::find(roomIds.begin(), roomIds.end(), wall.ownerRoomId) ==
            roomIds.end())
        {
            hasUnresolvableOwner = true;
            break;
        }
    }

    static constexpr const char *kLegacySchemaInsufficient =
        "LEGACY_SCHEMA_INSUFFICIENT_NO_IDENTITY_LIVENESS_MAP_OR_PROVENANCE";

    std::vector<AxiomCapabilityEntry> capabilityTable{};
    if (computeAxiomCapabilityTable(capabilityTable) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // computeAxiomCapabilityTable cannot fail; continue as before.
    }
    std::vector<LegacyAxiomResultRecord> results;
    results.reserve(capabilityTable.size());
    for (const AxiomCapabilityEntry &entry : capabilityTable)
    {
        LegacyAxiomResultRecord record;
        record.axiomCode = entry.axiomCode;
        if (entry.axiomCode == AxiomCode::AX_WALL_01 && hasUnresolvableOwner)
        {
            record.result = AxiomResult::FAIL;
            std::string reasonCodeName2{};
            if (reasonCodeName(ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE,
                               reasonCodeName2) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // reasonCodeName cannot fail; continue as before.
            }
            record.reason = reasonCodeName2;
        }
        else
        {
            record.result = AxiomResult::UNKNOWN;
            record.reason = kLegacySchemaInsufficient;
        }
        results.push_back(record);
    }
    std::sort(results.begin(),
              results.end(),
              [](const LegacyAxiomResultRecord &lhs_in,
                 const LegacyAxiomResultRecord &rhs_in)
              { return lhs_in.axiomCode < rhs_in.axiomCode; });

    return results;
}

LegacyReplayResult
    replayLegacyCaptures(const std::filesystem::path &corpusDir_in)
{
    LegacyReplayResult result;

    if (!std::filesystem::exists(corpusDir_in) ||
        !std::filesystem::is_directory(corpusDir_in))
    {
        return result;
    }

    std::vector<std::filesystem::path> captureFiles;
    for (const std::filesystem::directory_entry &entry :
         std::filesystem::directory_iterator(corpusDir_in))
    {
        if (entry.is_regular_file() &&
            entry.path().filename().string().rfind("capture_attempt_", 0) == 0)
        {
            captureFiles.push_back(entry.path());
        }
    }
    /* Sorted by filename so the report is independent of
     * directory_iterator's unspecified enumeration order; filenames are
     * otherwise pure diagnostic metadata (never topology/geometry input --
     * see legacyTopologyDigest()/legacyFullGeometryDigest()). */
    std::sort(captureFiles.begin(), captureFiles.end());
    result.totalFiles = captureFiles.size();

    for (const std::filesystem::path &filePath : captureFiles)
    {
        LegacyFileReplayResult fileResult;
        fileResult.fileName = filePath.filename().string();

        std::ifstream  file(filePath);
        nlohmann::json parsedJson;
        bool           parseOk = file.is_open();
        if (parseOk)
        {
            try
            {
                parsedJson = nlohmann::json::parse(file);
            }
            catch (const std::exception &)
            {
                parseOk = false;
            }
        }

        if (!parseOk)
        {
            fileResult.malformed       = true;
            fileResult.malformedReason = "could not open or JSON-parse file";
            ++result.malformedFiles;
            result.fileResults.push_back(std::move(fileResult));
            continue;
        }

        const LegacyParseResult parseResult = parseLegacyCapture(parsedJson);
        if (parseResult.malformed)
        {
            fileResult.malformed       = true;
            fileResult.malformedReason = parseResult.malformedReason;
            ++result.malformedFiles;
            result.fileResults.push_back(std::move(fileResult));
            continue;
        }

        const LegacyCapture &capture  = parseResult.capture;
        fileResult.roomCount          = capture.rooms.size();
        fileResult.wallCount          = capture.walls.size();
        fileResult.passageCount       = capture.passageCount;
        fileResult.hasFloor           = capture.floor.has_value();
        fileResult.topologyDigest     = legacyTopologyDigest(capture);
        fileResult.fullGeometryDigest = legacyFullGeometryDigest(capture);
        fileResult.axiomResults       = evaluateLegacyAxioms(capture);
        ++result.okFiles;
        result.fileResults.push_back(std::move(fileResult));
    }

    /* Build the bounded JSON report. */
    nlohmann::json topologyClassCounts       = nlohmann::json::object();
    nlohmann::json geometryDigestCounts      = nlohmann::json::object();
    nlohmann::json perAxiomTotals            = nlohmann::json::object();
    nlohmann::json unavailableEvidenceTotals = nlohmann::json::object();
    nlohmann::json filesJson                 = nlohmann::json::array();

    for (const LegacyFileReplayResult &fileResult : result.fileResults)
    {
        nlohmann::json fileJson;
        fileJson["file"]      = fileResult.fileName;
        fileJson["malformed"] = fileResult.malformed;
        if (fileResult.malformed)
        {
            fileJson["malformedReason"] = fileResult.malformedReason;
            filesJson.push_back(std::move(fileJson));
            continue;
        }

        fileJson["roomCount"]          = fileResult.roomCount;
        fileJson["wallCount"]          = fileResult.wallCount;
        fileJson["passageCount"]       = fileResult.passageCount;
        fileJson["hasFloor"]           = fileResult.hasFloor;
        fileJson["topologyDigest"]     = fileResult.topologyDigest;
        fileJson["fullGeometryDigest"] = fileResult.fullGeometryDigest;

        nlohmann::json axiomResultsJson = nlohmann::json::array();
        for (const LegacyAxiomResultRecord &axiomResult :
             fileResult.axiomResults)
        {
            std::string codeName{};
            if (axiomCodeName(axiomResult.axiomCode, codeName) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // axiomCodeName cannot fail; continue as before.
            }
            std::string axiomResultName2{};
            if (axiomResultName(axiomResult.result, axiomResultName2) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // axiomResultName cannot fail; continue as before.
            }
            axiomResultsJson.push_back({{"axiomCode", codeName},
                                        {"result", axiomResultName2},
                                        {"reason", axiomResult.reason}});

            std::string resultKey{};
            if (axiomResultName(axiomResult.result, resultKey) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // axiomResultName cannot fail; continue as before.
            }
            if (!perAxiomTotals.contains(codeName))
            {
                perAxiomTotals[codeName] = nlohmann::json::object();
            }
            const std::uint64_t current =
                perAxiomTotals[codeName].value(resultKey, std::uint64_t{0U});
            perAxiomTotals[codeName][resultKey] = current + 1U;
            if (axiomResult.result == AxiomResult::UNKNOWN)
            {
                std::uint64_t unavailable =
                    unavailableEvidenceTotals.value(codeName,
                                                    std::uint64_t{0U});
                unavailableEvidenceTotals[codeName] = unavailable + 1U;
            }
        }
        fileJson["axiomResults"] = std::move(axiomResultsJson);
        filesJson.push_back(std::move(fileJson));

        std::uint64_t topoCount =
            topologyClassCounts.value(fileResult.topologyDigest,
                                      std::uint64_t{0U});
        topologyClassCounts[fileResult.topologyDigest] = topoCount + 1U;
        std::uint64_t geomCount =
            geometryDigestCounts.value(fileResult.fullGeometryDigest,
                                       std::uint64_t{0U});
        geometryDigestCounts[fileResult.fullGeometryDigest] = geomCount + 1U;
    }

    nlohmann::json report;
    report["schema"]    = 1;
    report["corpusDir"] = corpusDir_in.string();
    report["orderingNote"] =
        "unordered: capture_attempt_*.json filenames carry no encoded "
        "chronological order; fileResults is sorted by filename purely for "
        "deterministic report bytes, not to imply a time sequence";
    report["totalFiles"]                  = result.totalFiles;
    report["malformedFiles"]              = result.malformedFiles;
    report["okFiles"]                     = result.okFiles;
    report["distinctTopologyClasses"]     = topologyClassCounts.size();
    report["distinctFullGeometryDigests"] = geometryDigestCounts.size();
    report["topologyClassCounts"]         = std::move(topologyClassCounts);
    report["geometryDigestCounts"]        = std::move(geometryDigestCounts);
    report["perAxiomResultTotals"]        = std::move(perAxiomTotals);
    report["unavailableEvidenceTotals"] = std::move(unavailableEvidenceTotals);
    report["files"]                     = std::move(filesJson);

    result.report = std::move(report);
    return result;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
