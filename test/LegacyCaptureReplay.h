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
 * @file            LegacyCaptureReplay.h
 *
 * @brief           Declares truthful deterministic replay of legacy
 *                  capture_attempt_*.json files (Phase-0 snapshots,
 *                  semantic-axiom-reliability-plan.md P1.9) against the
 *                  current axiom catalogue.
 *
 *                  The legacy schema is: {"scene": string, "rooms":
 *                  [{"id": int, "centroid_xy": [x, y]}], "walls": [{
 *                  "room_id": int, "normal": [x, y, z], "offset_d": number,
 *                  "extent_m": number}], "passages": [], "floor":
 *                  {"normal": [x, y, z], "offset_d": number}}. It carries no
 *                  wall identity, map identity, liveness, reciprocal
 *                  references, authoritative passage slots, provenance, or
 *                  timing -- a wall's only non-geometric fact is which
 *                  room's "room_id" it declares, and every axiom needing
 *                  identity/liveness/provenance is therefore reported
 *                  UNKNOWN, never fabricated. The one axiom code this
 *                  schema can directly falsify is AX_WALL_01 (wall
 *                  ownership): a wall whose room_id does not match any
 *                  room declared in the same file is a directly provable
 *                  contradiction (an unresolvable owner reference), so
 *                  that one case alone reports FAIL.
 */

#ifndef LEGACY_CAPTURE_REPLAY_H
#define LEGACY_CAPTURE_REPLAY_H

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomCode.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomResult.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! @brief One legacy room entry: {"id", "centroid_xy"}. */
struct LegacyRoomRecord
{
    int             id{0};
    Eigen::Vector2d centroidXy{Eigen::Vector2d::Zero()};
};

/*! @brief One legacy wall entry: {"room_id", "normal", "offset_d",
 *  "extent_m"}. \c ownerRoomId is this wall's only non-geometric fact --
 *  the schema has no independent wall identity. */
struct LegacyWallRecord
{
    int             ownerRoomId{0};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    double          offsetD{0.0};
    double          extentM{0.0};
};

/*! @brief The legacy "floor" entry: {"normal", "offset_d"}. */
struct LegacyFloorRecord
{
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    double          offsetD{0.0};
};

/*! @brief One fully parsed, pointer-free legacy capture file. Preserves
 *  every present field exactly and every present multiplicity (duplicate
 *  room ids, if any, are kept, never deduplicated). */
struct LegacyCapture
{
    std::string                   scene;
    std::vector<LegacyRoomRecord> rooms;
    std::vector<LegacyWallRecord> walls;

    /*! @brief Size of the legacy "passages" array. Every observed capture
     *  in the Phase-0 corpus has an empty "passages" array and the legacy
     *  schema documents no per-passage field shape, so a non-empty
     *  "passages" array is reported as a parse failure (see parseLegacy
     *  Capture()) rather than a guessed structure. */
    std::size_t passageCount{0U};

    std::optional<LegacyFloorRecord> floor;
};

/*! @brief Result of parsing one file: either a valid LegacyCapture, or an
 *  explicit, never-silently-dropped malformed reason. */
struct LegacyParseResult
{
    bool          malformed{false};
    std::string   malformedReason;
    LegacyCapture capture;
};

/*! @brief One current AxiomCode's replayed result for one legacy capture.
 *  \c result is always FAIL or UNKNOWN, never PASS -- see
 *  evaluateLegacyAxioms(). \c reason is a readable string: either a stable
 *  ReasonCode name (for the one directly provable FAIL) or a fixed
 *  "LEGACY_SCHEMA_INSUFFICIENT..." sentinel (this schema has no generic
 *  "insufficient evidence" ReasonCode of its own, and adding one to the
 *  closed evaluator enum is out of this replay's scope). */
struct LegacyAxiomResultRecord
{
    AxiomCode   axiomCode{AxiomCode::AX_FRAME_01};
    AxiomResult result{AxiomResult::UNKNOWN};
    std::string reason;
};

/*! @brief Complete replay result for one file. \c fileName is diagnostic
 *  metadata only -- it never participates in \c topologyDigest or \c
 *  fullGeometryDigest. */
struct LegacyFileReplayResult
{
    std::string fileName;
    bool        malformed{false};
    std::string malformedReason;

    std::size_t roomCount{0U};
    std::size_t wallCount{0U};
    std::size_t passageCount{0U};
    bool        hasFloor{false};

    /*! @brief sha256HexDigest() of a canonical projection containing only
     *  discrete facts (room id multiset, wall-owner-room-id multiset,
     *  passageCount, hasFloor) -- never a geometric field. Empty for a
     *  malformed file. */
    std::string topologyDigest;

    /*! @brief sha256HexDigest() of a canonical projection containing every
     *  field \c topologyDigest covers plus every geometric field (room
     *  centroids, wall normal/offset/extent, floor normal/offset), each
     *  compared with total-order-safe double comparison, never input
     *  order. Empty for a malformed file. */
    std::string fullGeometryDigest;

    /*! @brief Exactly one entry per current AxiomCode (from
     *  computeAxiomCapabilityTable(), so this list tracks the evaluator's
     *  own catalogue rather than a separately hardcoded count). Empty for
     *  a malformed file. */
    std::vector<LegacyAxiomResultRecord> axiomResults;
};

/*! @brief Complete, deterministic replay result for a corpus directory. */
struct LegacyReplayResult
{
    /*! @brief Total files in the corpus directory matching
     *  capture_attempt_*.json. */
    std::size_t totalFiles{0U};

    /*! @brief Files that failed to parse against the documented legacy
     *  schema (never silently dropped -- see \c fileResults). */
    std::size_t malformedFiles{0U};

    /*! @brief Files successfully parsed and evaluated. */
    std::size_t okFiles{0U};

    /*! @brief Per-file results, sorted by \c fileName for a deterministic
     *  report regardless of directory iteration order. */
    std::vector<LegacyFileReplayResult> fileResults;

    /*! @brief The complete bounded JSON report built by
     *  replayLegacyCaptures(), ready to write to a file: schema, corpusDir,
     *  ordering note, totals, topology/geometry digest class counts,
     *  per-axiom result totals, unavailable-evidence totals, and every
     *  file's record. */
    nlohmann::json report;
};

/*! @brief Parses one legacy capture JSON value against the documented
 *  schema. Never throws; a structurally invalid value (missing/wrong-typed
 *  "rooms"/"walls"/"passages"/room or wall field, or a non-empty
 *  "passages" array) is reported via \c LegacyParseResult::malformed, not
 *  silently coerced or dropped. */
LegacyParseResult parseLegacyCapture(const nlohmann::json &json_in);

/*! @brief Canonical topology-only digest of \p capture_in: see
 *  LegacyFileReplayResult::topologyDigest. */
std::string legacyTopologyDigest(const LegacyCapture &capture_in);

/*! @brief Canonical full-geometry digest of \p capture_in: see
 *  LegacyFileReplayResult::fullGeometryDigest. */
std::string legacyFullGeometryDigest(const LegacyCapture &capture_in);

/*! @brief Evaluates every current AxiomCode against \p capture_in. Never
 *  returns AxiomResult::PASS -- this legacy schema never carries the full
 *  evidence any current axiom requires for a positive proof. Returns FAIL
 *  only for AX_WALL_01 when some wall's ownerRoomId matches no declared
 *  room id (a directly provable, legacy-observable contradiction); every
 *  other code, and AX_WALL_01 itself when no such contradiction exists, is
 *  UNKNOWN. */
std::vector<LegacyAxiomResultRecord>
    evaluateLegacyAxioms(const LegacyCapture &capture_in);

/*!
 * @brief       Replays every capture_attempt_*.json file in \p corpusDir_in
 *              through parseLegacyCapture()/legacyTopologyDigest()/
 *              legacyFullGeometryDigest()/evaluateLegacyAxioms() and builds
 *              one deterministic, bounded report.
 *
 * @param[in]   corpusDir_in    Directory to scan (non-recursive) for
 *                              capture_attempt_*.json files.
 *
 * @return      A result whose \c fileResults order and \c report bytes
 *              depend only on each file's content and name, never on
 *              std::filesystem::directory_iterator's own (unspecified)
 *              enumeration order. \c totalFiles/malformedFiles/okFiles
 *              always sum consistently; a nonexistent or non-directory \p
 *              corpusDir_in yields totalFiles == 0, not a fabricated
 *              failure count.
 */
LegacyReplayResult
    replayLegacyCaptures(const std::filesystem::path &corpusDir_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // LEGACY_CAPTURE_REPLAY_H
