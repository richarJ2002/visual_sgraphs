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
 * @file            PassageFloorAgreement.h
 *
 * @brief           Declares the outcome of evaluatePassageFloorAgreement(),
 *                  shared by the AX-PASS-04 and AX-FLOOR-01 evaluators.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_PASSAGE_FLOOR_AGREEMENT_H
#define SEMANTIC_AXIOM_EVALUATOR_PASSAGE_FLOOR_AGREEMENT_H

#include <cstdint>

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Whether one live passage's real (live, confirmed) endpoint
 *              rooms agree on floor.
 */
enum class PassageFloorAgreement : std::uint8_t
{
    /*! @brief Fewer than two real endpoints have a resolvable floor link,
     *  so agreement cannot be checked; also used for zero/one real
     *  endpoints, which vacuously cannot disagree. */
    NOT_APPLICABLE = 0U,

    /*! @brief At least one real endpoint has no floor link yet. */
    EVIDENCE_UNAVAILABLE = 1U,

    /*! @brief Two real endpoints resolve to different floors. */
    DISAGREE = 2U,

    /*! @brief Two real endpoints resolve to the same floor. */
    AGREE = 3U,

    /*! @brief Two real endpoints share an equal floor key, but that key
     *  names more than one FloorRecord in the same map, or the resolved
     *  floor does not reciprocally list one or both endpoint rooms in its
     *  own roomRefs: identity/reciprocity ambiguity, distinct from a plain
     *  missing floor link (EVIDENCE_UNAVAILABLE) or a genuine cross-floor
     *  disagreement (DISAGREE). 2026-09-07 residual proof-closure repair. */
    AMBIGUOUS = 4U,

    /*! @brief At least one real endpoint room's own canonical
     *  evaluateOneRoomFloorReciprocity() result is FAIL: a known room/floor
     *  contradiction (wrong kind, cross-map, duplicate identity, duplicate
     *  or missing reverse membership, or a second claiming floor) that must
     *  dominate any floorKey-equality comparison rather than let equal
     *  dangling keys or one malformed reverse member become AGREE.
     *  2026-09-07 second proof-closure repair. */
    ENDPOINT_ROOM_FLOOR_INVALID = 5U,

    /*! @brief At least one real endpoint room's own canonical
     *  evaluateOneRoomFloorReciprocity() result is UNKNOWN (no FAIL among
     *  its findings, but at least one UNKNOWN): that room's own room-floor
     *  proof is itself unavailable, so this passage's floor agreement
     *  cannot be positively proved either, even though it is also not a
     *  proven contradiction. Distinct from ENDPOINT_ROOM_FLOOR_INVALID
     *  (FAIL dominates) and EVIDENCE_UNAVAILABLE (no floor link at all).
     *  Checkpoint-A residual repair. */
    ENDPOINT_ROOM_FLOOR_UNVERIFIED = 6U
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_PASSAGE_FLOOR_AGREEMENT_H
