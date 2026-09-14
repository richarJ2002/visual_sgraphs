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
 * @file            UnresolvedWallHypothesisRecord.h
 *
 * @brief           Declares a value-only copy of one
 *                   SemanticsManager::UndefendedWallState entry.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_UNRESOLVED_WALL_HYPOTHESIS_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_UNRESOLVED_WALL_HYPOTHESIS_RECORD_H

#include <cstddef>

#include "Semantic/SemanticGraphSnapshot/objects/RawPlaneRef.h"

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Value-only copy of one weak, unused wall hypothesis
 *              awaiting bounded retirement
 *              (SemanticsManager::UndefendedWallState, SemanticsManager.h).
 *
 *              Never holds SemanticsManager::UndefendedWallState's own
 *              Plane* -- \c wallRef is the pointer-free RawPlaneRef built
 *              from it at capture time instead (see rawPlaneRef()).
 */
struct UnresolvedWallHypothesisRecord
{
  public:
    /*! @brief Pointer-free reference to the unresolved wall Plane, or
     *  reason == UnavailableReason::NULL_REFERENCE when the source
     *  pointer was null. */
    RawPlaneRef wallRef;

    /*! @brief UndefendedWallState::unresolvedCycles at capture time. */
    unsigned int unresolvedCycles{0U};

    /*! @brief UndefendedWallState::cloudPointCount at capture time. */
    std::size_t cloudPointCount{0U};

    /*! @brief UndefendedWallState::observationCount at capture time. */
    std::size_t observationCount{0U};
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_GRAPH_SNAPSHOT_UNRESOLVED_WALL_HYPOTHESIS_RECORD_H
