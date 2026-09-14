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
 * @file            RoomBoundaryWallEvidenceStatus.h
 *
 * @brief           Declares the typed outcome of
 *                  isValidBoundaryWallEvidence()'s validation of one
 *                  RoomRecord::wallRefs entry, replacing a lossy boolean so a
 *                  known contradiction is never indistinguishable from merely
 *                  unavailable evidence.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_WALL_EVIDENCE_STATUS_H
#define SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_WALL_EVIDENCE_STATUS_H

#include <cstdint>

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Typed validity of one RoomRecord::wallRefs entry as boundary
 *              support evidence for its owning room (2026-09-07 residual
 *              proof-closure repair).
 */
enum class RoomBoundaryWallEvidenceStatus : std::uint8_t
{
    /*! @brief Present, WALL-typed, live, same-map, uniquely resolved, and
     *  reciprocally owned by the room -- trustworthy boundary evidence. */
    VALID = 0U,

    /*! @brief No reference was attempted, or the referenced plane has no
     *  map, or its WallRecord is not locatable among the captured maps: an
     *  ordinary gap in positive evidence, not a proven contradiction. */
    UNAVAILABLE = 1U,

    /*! @brief The reference is provably wrong: wrong plane type, retired,
     *  cross-map, an ambiguous duplicate-identity wall key, or a resolved
     *  WallRecord that does not reciprocally own the room. A known
     *  contradiction that must dominate any other VALID reference. */
    INVALID = 2U
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_WALL_EVIDENCE_STATUS_H
