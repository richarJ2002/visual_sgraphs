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
 * @file            OpenPassageHypothesisRecord.h
 *
 * @brief           Declares a value-only copy of one
 *                   SemanticsManager::OpenPassageEvidence entry.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_OPEN_PASSAGE_HYPOTHESIS_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_OPEN_PASSAGE_HYPOTHESIS_RECORD_H

#include <cstdint>

#include <Eigen/Core>

#include "Semantic/SemanticGraphSnapshot/objects/RawPlaneRef.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Value-only copy of one open-passage hypothesis
 *              (SemanticsManager::OpenPassageEvidence, SemanticsManager.h),
 *              a temporally tracked candidate awaiting repeated Voxblox
 *              confirmation before it is promoted to a Passage.
 *
 *              Never holds SemanticsManager::OpenPassageEvidence's own
 *              Plane* -- \c supportingWallRef is the pointer-free
 *              RawPlaneRef built from it at capture time instead (see
 *              rawPlaneRef()).
 */
struct OpenPassageHypothesisRecord
{
  public:
    /*! @brief Pointer-free reference to the supporting wall Plane, or
     *  reason == UnavailableReason::NULL_REFERENCE when the source
     *  pointer was null. */
    RawPlaneRef supportingWallRef;

    /*! @brief OpenPassageEvidence::centroid_world_m at capture time. */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*! @brief OpenPassageEvidence::confirmationCount at capture time. */
    std::size_t confirmationCount{0U};

    /*! @brief OpenPassageEvidence::missedUpdateCount at capture time. */
    std::size_t missedUpdateCount{0U};

    /*! @brief OpenPassageEvidence::lastConfirmedSkeletonFingerprint at
     *  capture time. */
    std::uint64_t lastConfirmedSkeletonFingerprint{0U};

    /*! @brief OpenPassageEvidence::openingRadius_m at capture time. */
    double openingRadius_m{0.0};

    /*! @brief OpenPassageEvidence::heightSpan_m at capture time. */
    double heightSpan_m{0.0};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_OPEN_PASSAGE_HYPOTHESIS_RECORD_H
