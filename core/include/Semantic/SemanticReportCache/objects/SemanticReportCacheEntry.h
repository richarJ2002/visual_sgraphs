/**
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
 * @file            SemanticReportCacheEntry.h
 *
 * @brief           Declares the copied, immutable value returned by
 *                   SemanticReportCache::getLatest() -- one coherent
 *                   semantic cycle's complete evaluation output.
 */

#ifndef SEMANTIC_REPORT_CACHE_ENTRY_H
#define SEMANTIC_REPORT_CACHE_ENTRY_H

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomEvaluationReport.h"
#include "Semantic/SemanticAxiomEvaluator/objects/MapCompletenessResult.h"
#include "Semantic/SemanticGraphSnapshot/objects/SemanticGraphSnapshot.h"

namespace ORB_SLAM3
{
namespace semantic
{

/*!
 * @brief       Copied, pointer-free, immutable result of one complete
 *              semantic transaction cycle.
 *
 *              Every field is a value copied while SemanticReportCache's
 *              own mutex was held; none aliases Atlas/Map/Room/Wall/
 *              Passage/Floor state, so an entry safely outlives the
 *              production objects it was captured from and may be read on
 *              any thread (a ROS service callback, in particular) without
 *              touching the semantic-update lock.
 */
struct SemanticReportCacheEntry
{
  public:
    /*! @brief The complete, pointer-free graph snapshot this cycle
     *  evaluated. */
    SemanticGraphSnapshot snapshot;

    /*! @brief The complete axiom evaluation output for \c snapshot. */
    AxiomEvaluationReport evaluationReport;

    /*! @brief The complete map-completeness shadow results for \c
     *  snapshot. */
    std::vector<MapCompletenessResult> completenessResults;

    /*! @brief SemanticsManager::Run()'s own semantic transaction cycle
     *  counter at capture time. */
    std::uint64_t semanticCycle{0U};

    /*! @brief Monotonically increasing count of successful
     *  SemanticReportCache::update() calls; distinct from \c
     *  semanticCycle, which SemanticsManager owns and may skip values for
     *  (e.g. a cycle with no transaction to evaluate). */
    std::uint64_t updateSequence{0U};

    /*! @brief \c snapshot.currentMapId, duplicated here for convenient
     *  access without inspecting the full snapshot; absent under the same
     *  conditions documented on SemanticGraphSnapshot::currentMapId. */
    std::optional<long unsigned int> currentMapId;

    /*! @brief Map::GetMapChangeIndex() for \c currentMapId at capture
     *  time, copied under the semantic-update lock; absent when \c
     *  currentMapId itself is absent. This is the map's own real,
     *  monotonically incremented change counter (Map::mnMapChange, bumped
     *  by every Map::IncreaseChangeIndex() call), not a value invented for
     *  this cache. Map::GetLastMapChange() (Map::mnMapChangeNotified) is
     *  deliberately not used here: it is a separate "last change this
     *  consumer already reacted to" cursor written by an unrelated
     *  tracking/loop-closing consumer via Map::SetLastMapChange(), so
     *  reading it here would silently piggyback on that consumer's own
     *  notification state instead of observing the map's actual revision. */
    std::optional<int> mapRevision;

    /*! @brief Documented stable digest (see Sha256Digest.h) of \c
     *  snapshot's canonical topology-only serialization
     *  (serializeSnapshotTopologyOnly()'s "maps" content, without the
     *  schema field). Independent of every geometric field. */
    std::string canonicalTopologyDigest;

    /*! @brief Same as \c canonicalTopologyDigest, over the canonical
     *  full-geometry serialization (serializeSnapshotFullGeometry()'s
     *  "maps" content). */
    std::string canonicalFullGeometryDigest;

    /*! @brief Counter that increments by exactly one, relative to the
     *  previously cached entry, only when \c canonicalFullGeometryDigest
     *  differs from the previous entry's -- i.e. it counts genuine
     *  geometry changes, not every cycle. Starts at 0 for the first
     *  update() call (nothing to compare against yet) and at 1 the first
     *  time a digest is actually observed to differ from a prior one. */
    std::uint64_t geometryRevision{0U};

    /*! @brief Wall-clock-free duration spent inside evaluateState()/
     *  evaluateTransition()/evaluateMapCompleteness() for this cycle. */
    std::chrono::milliseconds evaluationDuration{0};

    /*! @brief std::chrono::steady_clock instant this entry was cached,
     *  used only to compute an elapsed "age" for a caller (e.g. mission
     *  health's staleness field); never itself serialized as a canonical
     *  timestamp, and never compared across process restarts. */
    std::chrono::steady_clock::time_point updateInstant{
        std::chrono::steady_clock::time_point::min()};
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_REPORT_CACHE_ENTRY_H
