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
 * @file            SemanticReportCache.h
 *
 * @brief           Declares SemanticReportCache, the mutex-protected,
 *                   copied-value cache SemanticsManager owns to publish the
 *                   latest complete semantic evaluation cycle to any reader
 *                   thread (e.g. a ROS service callback) without touching
 *                   the semantic-update lock or exposing any live
 *                   Atlas/Map/Room/Wall/Passage pointer.
 */

#ifndef SEMANTIC_REPORT_CACHE_H
#define SEMANTIC_REPORT_CACHE_H

#include <mutex>

#include "Semantic/SemanticReportCache/objects/SemanticReportCacheEntry.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief       Thread-safe, copied-value cache of the latest complete
 *              semantic evaluation cycle.
 *
 *              Every accessor holds \c mMutex only long enough to copy
 *              state in or out, never across a caller's own processing;
 *              every returned SemanticReportCacheEntry is an independent
 *              value with no aliasing into the cache's own storage or into
 *              any production model object.
 */
class SemanticReportCache
{
  public:
    SemanticReportCache() = default;

    /*!
     * @brief       Records one complete semantic transaction cycle's
     *              result.
     *
     *              \c geometryRevision on the stored entry increments by
     *              exactly one relative to the entry this call replaces
     *              only when \p canonicalFullGeometryDigest_in differs
     *              from that prior entry's digest (or is the very first
     *              update, in which case it starts at 0); a cycle with an
     *              unchanged full-geometry digest leaves the counter
     *              unchanged, so callers can detect genuine geometry drift
     *              without re-parsing the digest strings themselves.
     *
     * @param[in]   snapshot_in                        Complete captured
     *              snapshot this cycle evaluated.
     * @param[in]   evaluationReport_in                 Complete evaluation
     *              output for \p snapshot_in.
     * @param[in]   completenessResults_in              Complete
     *              map-completeness shadow results for \p snapshot_in.
     * @param[in]   semanticCycle_in                    SemanticsManager's
     *              own transaction cycle counter.
     * @param[in]   currentMapId_in                     \p snapshot_in's
     *              current map id, or absent.
     * @param[in]   mapRevision_in                      Map::GetMapChangeIndex()
     *              for that map, copied under the semantic lock, or
     *              absent when \p currentMapId_in is absent.
     * @param[in]   canonicalTopologyDigest_in           Digest of \p
     *              snapshot_in's canonical topology-only serialization.
     * @param[in]   canonicalFullGeometryDigest_in       Digest of \p
     *              snapshot_in's canonical full-geometry serialization.
     * @param[in]   evaluationDuration_in                Time spent
     *              evaluating this cycle.
     */
    void
        update(const SemanticGraphSnapshot              &snapshot_in,
               const AxiomEvaluationReport              &evaluationReport_in,
               const std::vector<MapCompletenessResult> &completenessResults_in,
               std::uint64_t                             semanticCycle_in,
               std::optional<long unsigned int>          currentMapId_in,
               std::optional<int>                        mapRevision_in,
               const std::string        &canonicalTopologyDigest_in,
               const std::string        &canonicalFullGeometryDigest_in,
               std::chrono::milliseconds evaluationDuration_in);

    /*! @brief Returns a copy of the latest cached entry. When
     *  isAvailable() would return false, returns a default-constructed
     *  SemanticReportCacheEntry (updateInstant ==
     *  steady_clock::time_point::min(), every collection empty) rather
     *  than a null/optional value -- callers must check isAvailable()
     *  first to distinguish "no update yet" from a genuinely empty-graph
     *  cycle. */
    SemanticReportCacheEntry getLatest() const;

    /*! @brief True once at least one update() call has completed. */
    bool isAvailable() const;

  private:
    mutable std::mutex       mMutex;
    SemanticReportCacheEntry mLatest;
    bool                     mIsAvailable{false};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_REPORT_CACHE_H
