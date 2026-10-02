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
 * @file            captureOpenPassageHypotheses.cc
 *
 * @brief           Implements SemanticsManager::captureOpenPassageHypotheses(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "Semantic/SemanticGraphSnapshot.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::captureOpenPassageHypotheses(
    std::vector<semantic::OpenPassageHypothesisRecord>
        &captureOpenPassageHypotheses_out) const
{
    std::vector<semantic::OpenPassageHypothesisRecord> records;
    records.reserve(openPassageEvidence.size());
    for (const OpenPassageEvidence &evidence : openPassageEvidence)
    {
        semantic::OpenPassageHypothesisRecord record;
        semantic::RawPlaneRef                 rawPlaneRef2{};
        if (semantic::rawPlaneRef(evidence.p_supportingWall, rawPlaneRef2) !=
            semantic::SemanticGraphSnapshotStatus::
                SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: rawPlaneRef returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        record.supportingWallRef = rawPlaneRef2;
        record.centroid_world_m  = evidence.centroid_world_m;
        record.confirmationCount = evidence.confirmationCount;
        record.missedUpdateCount = evidence.missedUpdateCount;
        record.lastConfirmedSkeletonFingerprint =
            evidence.lastConfirmedSkeletonFingerprint;
        record.openingRadius_m = evidence.openingRadius_m;
        record.heightSpan_m    = evidence.heightSpan_m;
        records.push_back(record);
    }
    std::sort(records.begin(),
              records.end(),
              [](const semantic::OpenPassageHypothesisRecord &lhs_in,
                 const semantic::OpenPassageHypothesisRecord &rhs_in)
              {
                  if (semantic::isRawPlaneRefLess(lhs_in.supportingWallRef,
                                                  rhs_in.supportingWallRef))
                      return true;
                  if (semantic::isRawPlaneRefLess(rhs_in.supportingWallRef,
                                                  lhs_in.supportingWallRef))
                      return false;
                  return semantic::isVector3dLess(lhs_in.centroid_world_m,
                                                  rhs_in.centroid_world_m);
              });
    captureOpenPassageHypotheses_out = records;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
