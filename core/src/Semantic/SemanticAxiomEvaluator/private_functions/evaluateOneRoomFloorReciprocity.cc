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
 * @file            evaluateOneRoomFloorReciprocity.cc
 *
 * @brief           Implements evaluateOneRoomFloorReciprocity(), declared in
 *                  private_functions.h.
 *
 *                  Scans every FloorRecord in the room's own map --
 *                  not only the one RoomRecord::floorRef names -- to detect
 *                  duplicate same-key floor identity, duplicate reverse
 *                  membership within the named floor, and a second,
 *                  distinct floor also listing this room. "Wrong kind" is
 *                  not a separate case: FloorRecord::roomRefs entries are
 *                  always built via entityRefForRoom() (ROOM-kind by
 *                  construction), so a wrong-kind key structurally cannot
 *                  equal room_in.key -- the same argument
 *                  scanReversePassageEndpoints.cc makes for passageRefs.
 *
 *                  No early UNKNOWN "no floor yet" verdict is returned
 *                  before checking whether some other floor in the map
 *                  reverse-claims this room anyway -- a floor's own
 *                  roomRefs naming a room with no reciprocal forward
 *                  floorRef at all is a known FAIL, not the ordinary
 *                  missing-evidence case. Also validates the forward key's
 *                  own EntityKind (defensive; RoomRecord::floorRef is
 *                  always FLOOR-kind by construction via
 *                  entityRefForFloor(), reachable only through adversarial
 *                  snapshot mutation, mirroring this file's own wrong-kind
 *                  argument above) and the named floor's own declaredMapId
 *                  consistency with its containing map.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOneRoomFloorReciprocity(const RoomRecord            &room_in,
                                     const SemanticGraphSnapshot &snapshot_in,
                                     const MapSnapshot    &mapSnapshot_in,
                                     std::vector<Finding> &findings_inout)
{
    const std::vector<EntityKey> involvedKeys{room_in.key};

    if (room_in.key.kind != EntityKind::ROOM)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_ROOM_WRONG_KIND,
                        involvedKeys));
        return;
    }

    if (room_in.declaredMapId.has_value() &&
        *room_in.declaredMapId != room_in.key.mapId)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH,
                        involvedKeys));
        return;
    }

    if (countMapSnapshotsWithId(snapshot_in, room_in.key.mapId) > 1U)
    {
        /* Which MapSnapshot actually holds
         * this room's floor is itself ambiguous when its own containing map
         * id is duplicated -- no first-match FloorRecord lookup below may
         * supply positive proof, and this is also the root cause of a real
         * null-pointer crash (a duplicate mapId can place the uniquely-
         * counted FloorRecord in a sibling MapSnapshot never searched by
         * findRecordByKey(mapSnapshot_in.floors, ...) below). */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_CONTAINING_MAP_AMBIGUOUS,
                        involvedKeys));
        return;
    }

    if (room_in.floorRef.key.has_value() &&
        room_in.floorRef.reason != UnavailableReason::NONE)
    {
        /* EntityRef documents key.has_value() <=> reason == NONE as an
         * invariant; a keyed forward floor reference whose own reason is
         * not NONE is a known contradiction, not an ordinary valid
         * reference. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_ROOM_REASON_INCONSISTENT,
                        involvedKeys));
        return;
    }

    if (!room_in.floorRef.key.has_value())
    {
        /* A missing forward link is not automatically the ordinary case:
         * some other floor may still reverse-claim this room, a known
         * contradiction that must dominate the plain "no floor yet"
         * UNKNOWN. */
        for (const FloorRecord &floor : mapSnapshot_in.floors)
        {
            for (const EntityRef &reverseRoomReference : floor.roomRefs)
            {
                if (reverseRoomReference.key.has_value() &&
                    *reverseRoomReference.key == room_in.key)
                {
                    findings_inout.push_back(makeFinding(
                        AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::
                            ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK,
                        {room_in.key, floor.key}));
                    return;
                }
            }
        }
        findings_inout.push_back(makeFinding(AxiomCode::AX_FLOOR_01,
                                             AxiomResult::UNKNOWN,
                                             ReasonCode::ROOM_FLOOR_UNLINKED,
                                             involvedKeys));
        return;
    }

    if (room_in.floorRef.key->kind != EntityKind::FLOOR)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_FLOOR_01,
                                             AxiomResult::FAIL,
                                             ReasonCode::ROOM_FLOOR_WRONG_KIND,
                                             involvedKeys));
        return;
    }

    if (room_in.floorRef.key->mapId != room_in.key.mapId)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_FLOOR_01,
                                             AxiomResult::FAIL,
                                             ReasonCode::ROOM_FLOOR_CROSS_MAP,
                                             involvedKeys));
        return;
    }

    const std::size_t namedFloorMatchCount =
        countFloorRecordsWithKey(snapshot_in, *room_in.floorRef.key);
    if (namedFloorMatchCount > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_DUPLICATE_IDENTITY,
                        involvedKeys));
        return;
    }
    if (namedFloorMatchCount == 0U)
    {
        /* Same map, key present, but not locatable among captured floor
         * records -- an anomaly this schema cannot further diagnose (see
         * WALL_TWIN_MAP_UNAVAILABLE's analogous case). */
        findings_inout.push_back(makeFinding(AxiomCode::AX_FLOOR_01,
                                             AxiomResult::UNKNOWN,
                                             ReasonCode::ROOM_FLOOR_UNLINKED,
                                             involvedKeys));
        return;
    }

    const FloorRecord *p_floor =
        findRecordByKey(mapSnapshot_in.floors, *room_in.floorRef.key);
    if (p_floor == nullptr)
    {
        /* countFloorRecordsWithKey() is snapshot-wide, so
         * namedFloorMatchCount == 1U above only proves a match exists
         * somewhere in the snapshot, not that it is enumerated in this
         * room's own mapSnapshot_in -- defense in depth alongside the
         * duplicate-containing-map preflight above. */
        findings_inout.push_back(makeFinding(AxiomCode::AX_FLOOR_01,
                                             AxiomResult::UNKNOWN,
                                             ReasonCode::ROOM_FLOOR_UNLINKED,
                                             involvedKeys));
        return;
    }

    if (p_floor->declaredMapId.has_value() &&
        *p_floor->declaredMapId != p_floor->key.mapId)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_DECLARED_MAP_MISMATCH,
                        involvedKeys));
        return;
    }

    std::size_t reverseMembershipCount         = 0U;
    std::size_t livenessUnavailableMemberCount = 0U;
    bool        anyReverseMemberMalformed      = false;
    for (const EntityRef &roomReference : p_floor->roomRefs)
    {
        if (!roomReference.key.has_value() || *roomReference.key != room_in.key)
        {
            continue;
        }
        if (roomReference.reason != UnavailableReason::NONE ||
            (roomReference.isLive.has_value() && !(*roomReference.isLive)))
        {
            /* A keyed reverse member naming this room whose own reason is
             * inconsistent (EntityRef invariant violation), or whose own
             * liveness is known false, is a known contradiction -- not
             * silently counted as reciprocal proof. */
            anyReverseMemberMalformed = true;
            continue;
        }
        if (!roomReference.isLive.has_value())
        {
            /* "Missing liveness is unavailable, not live": never silently
             * counted as a confirmed reciprocal member. */
            ++livenessUnavailableMemberCount;
            continue;
        }
        ++reverseMembershipCount;
    }
    if (anyReverseMemberMalformed)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_REVERSE_MEMBER_INVALID,
                        involvedKeys));
        return;
    }
    /* Multiplicity must count every otherwise-clean match
     * (confirmed-live or liveness-unavailable alike), not only confirmed-live
     * ones -- a clean-live member plus a liveness-unavailable duplicate is
     * still ambiguous multiplicity, not a clean single reciprocal member
     * (mirrors scanReversePassageEndpoints.cc's own cleanMatchCountThisRoom,
     * which already counts liveness-unavailable matches toward duplicate
     * detection). */
    const std::size_t totalOtherwiseCleanCount =
        reverseMembershipCount + livenessUnavailableMemberCount;
    if (totalOtherwiseCleanCount == 0U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_NON_RECIPROCAL,
                        involvedKeys));
        return;
    }
    if (totalOtherwiseCleanCount > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP,
                        involvedKeys));
        return;
    }
    if (reverseMembershipCount == 0U)
    {
        /* Exactly one otherwise-clean match, and it is the
         * liveness-unavailable one. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_FLOOR_01,
            AxiomResult::UNKNOWN,
            ReasonCode::ROOM_FLOOR_REVERSE_MEMBER_LIVENESS_UNAVAILABLE,
            involvedKeys));
        return;
    }

    for (const FloorRecord &otherFloor : mapSnapshot_in.floors)
    {
        if (otherFloor.key == p_floor->key)
        {
            continue;
        }
        for (const EntityRef &roomReference : otherFloor.roomRefs)
        {
            if (roomReference.key.has_value() &&
                *roomReference.key == room_in.key)
            {
                findings_inout.push_back(makeFinding(
                    AxiomCode::AX_FLOOR_01,
                    AxiomResult::FAIL,
                    ReasonCode::ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS,
                    {room_in.key, otherFloor.key, p_floor->key}));
                return;
            }
        }
    }

    findings_inout.push_back(
        makeFinding(AxiomCode::AX_FLOOR_01,
                    AxiomResult::PASS,
                    ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID,
                    involvedKeys));
    if (!room_in.declaredMapId.has_value())
    {
        /* Every other clause is affirmatively satisfied, but the room's
         * own declared map could not actually be verified either way --
         * cap this room's positive proof at UNKNOWN rather than PASS,
         * mirroring the analogous wall/passage declared-map-absent
         * patterns in this module. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_UNAVAILABLE,
                        involvedKeys));
    }
    if (!p_floor->declaredMapId.has_value())
    {
        /* The uniquely resolved floor's own
         * declaredMapId could not be verified either way either -- cap
         * this room's positive proof at UNKNOWN rather than PASS,
         * mirroring the room's own missing-declared-map cap immediately
         * above. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_FLOOR_FLOOR_DECLARED_MAP_UNAVAILABLE,
                        involvedKeys));
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
