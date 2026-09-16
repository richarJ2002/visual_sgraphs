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
 * @file         scanReversePassageEndpoints.cc
 *
 * @brief        Implements scanReversePassageEndpoints(), declared in
 *               private_functions.h.
 *
 *               Checking only the two rooms named by
 *               PassageRecord::knownSideRoomRef/prospectiveRoomRef leaves
 *               a third live confirmed room whose own RoomRecord::
 *               passageRefs also names this passage structurally invisible.
 *               This function inverts the direction: it scans every
 *               RoomRecord (live or retired) in every captured map for a
 *               passageRefs entry naming this passage, independent of the
 *               passage's own forward fields, and classifies each match
 *               -- a retired room that still lists the passage is itself
 *               a bad reverse reference, not silently excluded from
 *               consideration.
 *
 *               A retired RoomRecord (isLive == false) is skipped in
 *               full -- current-state axioms use live committed records,
 *               and retired reverse-room history alone must not poison a
 *               live passage. A live prospective (non-ROOM-variant)
 *               room's passageRefs entries are examined for the same
 *               bad/cross-map/duplicate-identity/wrong-kind anomalies as
 *               a confirmed room's, not silently discarded, even though
 *               they still never contribute to confirmedReverseRoomKeys.
 *               A same-map, live room's passageRefs entry sharing this
 *               passage's map/entity id but a different EntityKind is a
 *               typed wrongKindReverseRoomKeys anomaly rather than a
 *               silent non-match: EntityKey equality compares kind too,
 *               so the general `*passageRef.key != passage_in.key`
 *               comparison alone would otherwise treat a wrong-kind key
 *               exactly like an unrelated reference. A live, same-map,
 *               ROOM-variant room whose own passageRefs names this
 *               passage more than once is a typed
 *               duplicateReferenceRoomKeys anomaly instead of being
 *               silently collapsed to one clean match by the final
 *               key-deduplication pass.
 *
 *               An unkeyed passageRefs entry (a local id with no map at
 *               capture time) is not attributed to this specific passage
 *               by bare local-id equality -- local ids are unique only
 *               within one map and are not themselves a map-qualified
 *               identity (see EntityKey.h); that evidence is reported
 *               room/map-scoped by evaluateRoomMalformedPassageReferences()
 *               instead. A clean live prospective reverse relationship is
 *               represented in prospectiveReverseRoomKeys rather than
 *               silently discarded, and a same-map reverse reference
 *               whose own EntityRef::isLive carries no value is
 *               represented in livenessUnavailableReverseRoomKeys rather
 *               than silently falling through to be counted as a
 *               confirmed match ("missing liveness is unavailable, not
 *               live"). Duplicate-multiplicity detection counts every
 *               otherwise-clean match (confirmed, prospective, or
 *               liveness-unavailable alike), not only ROOM-variant
 *               confirmed matches.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

ReversePassageEndpointScan
    scanReversePassageEndpoints(const PassageRecord         &passage_in,
                                long unsigned int            expectedMapId_in,
                                const SemanticGraphSnapshot &snapshot_in)
{
    ReversePassageEndpointScan scan;

    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        for (const RoomRecord &room : mapSnapshot.rooms)
        {
            /* Current-state axioms use live committed records; retired
             * reverse-room history alone must not poison a live passage. */
            if (!room.isLive)
            {
                continue;
            }

            /* Counts every otherwise-clean match this room's own
             * passageRefs makes to this passage -- confirmed, prospective,
             * or liveness-unavailable alike -- so duplicate multiplicity is
             * detected regardless of which of those three clean categories
             * the duplicated entries fall into ("duplicate prospective or
             * unavailable-liveness references must not escape the
             * multiplicity check"). Bad/cross-map/duplicate-identity/
             * wrong-kind entries are excluded: each is already
             * individually flagged via its own dedicated anomaly bucket
             * regardless of count. */
            std::size_t cleanMatchCountThisRoom = 0U;
            for (const EntityRef &passageRef : room.passageRefs)
            {
                if (!passageRef.key.has_value())
                {
                    /* An unkeyed local-id-only reference can never be
                     * safely attributed to this specific passage merely
                     * because a bare integer happens to match -- local ids
                     * are unique only within one map and are not
                     * themselves a map-qualified identity (see
                     * EntityKey.h). evaluateRoomMalformedPassageReferences()
                     * reports this room/map-scoped evidence independently
                     * of any specific passage. */
                    continue;
                }
                if (passageRef.key->mapId == passage_in.key.mapId &&
                    passageRef.key->entityId == passage_in.key.entityId &&
                    passageRef.key->kind != EntityKind::PASSAGE)
                {
                    scan.wrongKindReverseRoomKeys.push_back(room.key);
                    continue;
                }
                if (*passageRef.key != passage_in.key)
                {
                    continue;
                }
                if (passageRef.reason != UnavailableReason::NONE)
                {
                    /* EntityRef documents
                     * key.has_value() <=> reason == NONE as an invariant; a
                     * keyed reverse reference whose own reason is not NONE
                     * is a known contradiction, not proof of reciprocity. */
                    scan.badReverseRoomKeys.push_back(room.key);
                    continue;
                }
                if (room.key.mapId != expectedMapId_in)
                {
                    scan.crossMapReverseRoomKeys.push_back(room.key);
                    continue;
                }
                if (countRoomRecordsWithKey(snapshot_in, room.key) > 1U)
                {
                    scan.duplicateIdentityRoomKeys.push_back(room.key);
                    continue;
                }
                if (passageRef.isLive.has_value() && !(*passageRef.isLive))
                {
                    scan.badReverseRoomKeys.push_back(room.key);
                    continue;
                }
                ++cleanMatchCountThisRoom;
                if (!passageRef.isLive.has_value())
                {
                    /* "Missing liveness is unavailable, not live": never
                     * silently counted as a confirmed (or prospective)
                     * reciprocal endpoint. */
                    scan.livenessUnavailableReverseRoomKeys.push_back(room.key);
                }
                else if (room.variant != Room::RoomVariant::ROOM)
                {
                    /* A clean live prospective reverse relationship is
                     * represented, not silently discarded, even though a
                     * prospective handle is never a "real"/confirmed
                     * cardinality endpoint (see isRealPassageEndpoint()). */
                    scan.prospectiveReverseRoomKeys.push_back(room.key);
                }
                else
                {
                    scan.confirmedReverseRoomKeys.push_back(room.key);
                }
            }
            if (cleanMatchCountThisRoom > 1U)
            {
                scan.duplicateReferenceRoomKeys.push_back(room.key);
            }
        }
    }

    for (std::vector<EntityKey> *p_keys :
         {&scan.confirmedReverseRoomKeys,
          &scan.badReverseRoomKeys,
          &scan.crossMapReverseRoomKeys,
          &scan.duplicateIdentityRoomKeys,
          &scan.wrongKindReverseRoomKeys,
          &scan.duplicateReferenceRoomKeys,
          &scan.prospectiveReverseRoomKeys,
          &scan.livenessUnavailableReverseRoomKeys})
    {
        std::sort(p_keys->begin(), p_keys->end());
        p_keys->erase(std::unique(p_keys->begin(), p_keys->end()),
                      p_keys->end());
    }

    return scan;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
