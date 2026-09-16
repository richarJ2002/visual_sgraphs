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
 * @file            roomListsPassageBack.cc
 *
 * @brief           Implements roomListsPassageBack(), declared in
 *                  private_functions.h.
 *
 *                  Consolidates what was previously two independent
 *                  copies of this function in evaluateAxPass02.cc and
 *                  computeConservativeMapCompleteness.cc.
 *
 *                  A matching
 *                  passageRefs entry no longer proves reciprocity merely by
 *                  key equality. It must also carry reason == NONE (an
 *                  EntityRef invariant violation is a known contradiction,
 *                  not proof), and be live (available and true); a
 *                  liveness-unavailable match is excluded from positive
 *                  proof without being treated as a hard contradiction.
 *                  More than one otherwise-clean match is itself ambiguous.
 *                  The passage this claim is about must also resolve to
 *                  exactly one live passage record in the caller's own
 *                  snapshot -- see the countPassageRecordsWithKey() check
 *                  below -- since a duplicate-identity or unenumerable
 *                  passage cannot be reciprocated with either. Known
 *                  limitation: a liveness-unavailable-only match returns
 *                  false (no proof) rather than a dedicated UNKNOWN, since
 *                  this function's boolean return cannot itself distinguish
 *                  the two; callers treat false uniformly as
 *                  non-reciprocal.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool roomListsPassageBack(const SemanticGraphSnapshot &snapshot_in,
                          const EntityKey             &roomKey_in,
                          const EntityKey             &passageKey_in)
{
    if (countMapSnapshotsWithId(snapshot_in, roomKey_in.mapId) > 1U)
    {
        return false;
    }
    if (countPassageRecordsWithKey(snapshot_in, passageKey_in) != 1U)
    {
        return false;
    }
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != roomKey_in.mapId)
        {
            continue;
        }
        if (countRoomRecordsWithKey(snapshot_in, roomKey_in) > 1U)
        {
            /* Ambiguous identity can never supply positive reciprocity
             * proof. */
            return false;
        }
        const RoomRecord *p_room =
            findRecordByKey(mapSnapshot.rooms, roomKey_in);
        if (p_room == nullptr)
        {
            return false;
        }
        std::size_t cleanMatchCount = 0U;
        for (const EntityRef &passageRef : p_room->passageRefs)
        {
            if (!passageRef.key.has_value() || *passageRef.key != passageKey_in)
            {
                continue;
            }
            if (passageRef.reason != UnavailableReason::NONE)
            {
                /* A keyed match whose own reason is not NONE is an
                 * invariant violation: a known contradiction, dominating
                 * even an otherwise-clean match elsewhere in this room. */
                return false;
            }
            if (passageRef.isLive.has_value() && !(*passageRef.isLive))
            {
                return false;
            }
            if (!passageRef.isLive.has_value())
            {
                /* Missing liveness is unavailable, not live: excluded from
                 * positive proof without being a hard contradiction. */
                continue;
            }
            ++cleanMatchCount;
        }
        return cleanMatchCount == 1U;
    }
    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
