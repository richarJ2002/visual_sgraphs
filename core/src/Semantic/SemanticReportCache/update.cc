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
 * @file            update.cc
 *
 * @brief           Implements SemanticReportCache::update(), declared in
 *                  Semantic/SemanticReportCache.h.
 */

#include "Semantic/SemanticReportCache.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void SemanticReportCache::update(
    const SemanticGraphSnapshot              &snapshot_in,
    const AxiomEvaluationReport              &evaluationReport_in,
    const std::vector<MapCompletenessResult> &completenessResults_in,
    std::uint64_t                             semanticCycle_in,
    std::optional<long unsigned int>          currentMapId_in,
    std::optional<int>                        mapRevision_in,
    const std::string                        &canonicalTopologyDigest_in,
    const std::string                        &canonicalFullGeometryDigest_in,
    std::chrono::milliseconds                 evaluationDuration_in)
{
    std::lock_guard<std::mutex> lock(mMutex);

    const bool geometryChanged =
        mIsAvailable &&
        (canonicalFullGeometryDigest_in != mLatest.canonicalFullGeometryDigest);
    const std::uint64_t nextGeometryRevision =
        mIsAvailable ? (mLatest.geometryRevision + (geometryChanged ? 1U : 0U))
                     : 0U;

    SemanticReportCacheEntry entry;
    entry.snapshot                    = snapshot_in;
    entry.evaluationReport            = evaluationReport_in;
    entry.completenessResults         = completenessResults_in;
    entry.semanticCycle               = semanticCycle_in;
    entry.updateSequence              = mLatest.updateSequence + 1U;
    entry.currentMapId                = currentMapId_in;
    entry.mapRevision                 = mapRevision_in;
    entry.canonicalTopologyDigest     = canonicalTopologyDigest_in;
    entry.canonicalFullGeometryDigest = canonicalFullGeometryDigest_in;
    entry.geometryRevision            = nextGeometryRevision;
    entry.evaluationDuration          = evaluationDuration_in;
    entry.updateInstant               = std::chrono::steady_clock::now();

    mLatest      = std::move(entry);
    mIsAvailable = true;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
