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
 * @file         computeConservativeMapCompleteness.cc
 *
 * @brief        Implements computeConservativeMapCompleteness(),
 *               declared in private_functions.h.
 *
 *               Replaces the
 *               pre-repair hand-written subset (room/prospective
 *               counts, a passage-endpoint-only re-derivation, and
 *               a wall-ownership duplicate of AX-WALL-01's own
 *               logic) with direct reuse of the same map-scoped,
 *               per-entity leaf evaluators
 *               evaluateAxWall01.cc/evaluateAxWall03.cc/
 *               evaluateAxBound01.cc/evaluateAxFloor01.cc/
 *               evaluateAxPass02.cc already call --
 *               evaluateOneWall()/evaluateOneWallTwin()/
 *               evaluateOneRoomBoundary()/
 *               evaluateOneRoomFloorReciprocity()/
 *               evaluateOnePassageCardinality() -- so the two paths
 *               cannot drift apart. This function still never calls
 *               evaluateState() or evaluateMapCompleteness() itself,
 *               avoiding the circular dependency
 *               evaluateAxComp01.cc's own Doxygen documents
 *               (evaluateState() calls evaluateMapCompleteness() to
 *               build its AX-COMP-01 Finding); calling the leaf
 *               per-entity functions directly is not a call back
 *               into that orchestrator.
 *
 *               Because AX-BOUND-01 and AX-PASS-02 can now only
 *               report FAIL or UNKNOWN (their capability level is
 *               PARTIAL/DEFERRED -- see
 *               computeAxiomCapabilityTable()), any map with at
 *               least one live passage or one confirmed
 *               COMPLETE-status room is capped at UNKNOWN here too,
 *               never PASS. This is the honest, schema-limited
 *               consequence, not a defect: "If the accepted schema
 *               makes full positive proof impossible, that is an
 *               honest result, not a reason to fabricate a PASS."
 *
 *                  Rules, in the order checked (any FAIL-class condition
 *                  found anywhere makes the whole map FAIL; otherwise any
 *                  UNKNOWN-class condition makes it UNKNOWN; otherwise
 *                  PASS):
 *                  - zero confirmed (live, ROOM-variant) rooms -> FAIL;
 *                  - any live prospective (UNDEFINED-variant) room -> FAIL;
 *                  - any confirmed room whose model-reported boundaryStatus
 *                    is not COMPLETE, or whose COMPLETE geometry
 *                    independently fails evaluateOneRoomBoundary() -> FAIL;
 *                  - any non-passable live passage, or any live passage
 *                    whose evaluateOnePassageCardinality() reports FAIL ->
 *                    FAIL (COMPLETENESS_PASSAGE_ENDPOINTS_INVALID);
 *                  - any live wall whose evaluateOneWall() reports FAIL, or
 *                    any live wall whose evaluateOneWallTwin() reports FAIL
 *                    -> FAIL (COMPLETENESS_HARD_CONTRADICTION);
 *                  - any confirmed room whose evaluateOneRoomFloorReciprocity()
 *                    reports FAIL -> FAIL (COMPLETENESS_HARD_CONTRADICTION);
 *                  - otherwise, any live passage (cardinality proof is
 *                    permanently UNKNOWN pending authoritative endpoint
 *                    slots), any UNKNOWN wall/twin/floor/boundary finding
 *                    above -> UNKNOWN; and
 *                  - otherwise PASS.
 *
 *                  A map-local duplicate same-key room, wall, passage,
 *                  or floor record is an unconditional FAIL
 *                  (COMPLETENESS_DUPLICATE_IDENTITY); every live
 *                  passage's AX-PASS-03 (evaluateOnePassageSlotState())
 *                  and AX-PASS-04 (evaluateOnePassageMapAndFloor()) FAIL
 *                  findings also drive COMPLETENESS_PASSAGE_ENDPOINTS_
 *                  INVALID, not only AX-PASS-02 cardinality; every live
 *                  passage's AX-FLOOR-01 endpoint-floor-identity
 *                  (evaluateOnePassageFloorIdentity()) finding
 *                  contributes to the same floor FAIL/UNKNOWN aggregation
 *                  as the room-floor branch; and any confirmed room also
 *                  contributes COMPLETENESS_ROOM_CREATION_PROVENANCE_
 *                  UNAVAILABLE (RoomRecord::creationProvenanceReason is
 *                  always NOT_TRACKED_BY_CURRENT_SCHEMA).
 *
 *                  A duplicate MapSnapshot::mapId sharing this map's own
 *                  id is an unconditional FAIL (COMPLETENESS_DUPLICATE_
 *                  MAP_IDENTITY); every live passage's AX-PASS-01
 *                  provenance reuses evaluateOnePassageProvenance() (the
 *                  same leaf evaluateAxPass01() calls) instead of a
 *                  hand-written passable() check, and its FAIL also drives
 *                  COMPLETENESS_PASSAGE_ENDPOINTS_INVALID; and every live
 *                  room's malformed (unkeyed) passageRefs entries, found
 *                  via evaluateRoomMalformedPassageReferences() (the same
 *                  leaf evaluateAxPass02() calls), drive a dedicated
 *                  COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE
 *                  FAIL.
 *
 *                  Every confirmed room's
 *                  room-creation-provenance UNKNOWN now reuses the identical
 *                  AX-ROOM-01 leaf evaluateOneRoomCreationProvenance() (the
 *                  same leaf evaluateAxRoom01() calls), replacing a
 *                  hand-written per-room loop that duplicated its logic.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>
#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MapCompletenessResult
    computeConservativeMapCompleteness(const SemanticGraphSnapshot &snapshot_in,
                                       const MapSnapshot &mapSnapshot_in)
{
    MapCompletenessResult result;
    result.mapId = mapSnapshot_in.mapId;

    std::vector<ReasonCode> failReasons;
    std::vector<ReasonCode> unknownReasons;
    std::vector<EntityKey>  relevantKeys;

    bool anyLiveProspective = false;
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (!room.isLive)
        {
            continue;
        }
        if (room.variant == Room::RoomVariant::ROOM)
        {
            result.confirmedRoomCount++;
            if (room.boundaryStatus == Room::BoundaryStatus::COMPLETE)
            {
                result.completeRoomCount++;
            }
            else
            {
                relevantKeys.push_back(room.key);
            }
        }
        else
        {
            result.prospectiveRoomCount++;
            anyLiveProspective = true;
            relevantKeys.push_back(room.key);
        }
    }

    if (result.confirmedRoomCount == 0U)
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS);
    }
    if (anyLiveProspective)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_LIVE_PROSPECTIVE_ROOM_PRESENT);
    }

    if (countMapSnapshotsWithId(snapshot_in, mapSnapshot_in.mapId) > 1U)
    {
        /* Which MapSnapshot is
         * authoritative for this map is itself ambiguous, so no
         * first-matching-map lookup performed anywhere below (or by any
         * shared leaf this function calls) can be trusted for this map. */
        failReasons.push_back(ReasonCode::COMPLETENESS_DUPLICATE_MAP_IDENTITY);
    }

    /* Map-local duplicate identity ambiguity: a duplicate same-key room,
     * wall, passage, or floor record must never let any first-match proof
     * elsewhere in this map be trusted. */
    std::vector<EntityKey> duplicateIdentityKeys;
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (countRoomRecordsWithKey(snapshot_in, room.key) > 1U)
        {
            duplicateIdentityKeys.push_back(room.key);
        }
    }
    for (const WallRecord &wall : mapSnapshot_in.walls)
    {
        if (countWallRecordsWithKey(snapshot_in, wall.key) > 1U)
        {
            duplicateIdentityKeys.push_back(wall.key);
        }
    }
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (countPassageRecordsWithKey(snapshot_in, passage.key) > 1U)
        {
            duplicateIdentityKeys.push_back(passage.key);
        }
    }
    for (const FloorRecord &floor : mapSnapshot_in.floors)
    {
        if (countFloorRecordsWithKey(snapshot_in, floor.key) > 1U)
        {
            duplicateIdentityKeys.push_back(floor.key);
        }
    }
    if (!duplicateIdentityKeys.empty())
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_DUPLICATE_IDENTITY);
        relevantKeys.insert(relevantKeys.end(),
                            duplicateIdentityKeys.begin(),
                            duplicateIdentityKeys.end());
    }

    std::vector<Finding> boundaryFindings;
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (!room.isLive || room.variant != Room::RoomVariant::ROOM)
        {
            continue;
        }
        evaluateOneRoomBoundary(room, snapshot_in, boundaryFindings);
    }
    const bool anyBoundaryFail =
        anyFindingIs(boundaryFindings, AxiomResult::FAIL);
    const bool anyBoundaryUnknown =
        anyFindingIs(boundaryFindings, AxiomResult::UNKNOWN);
    if (result.completeRoomCount != result.confirmedRoomCount ||
        anyBoundaryFail)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_ROOM_BOUNDARY_NOT_COMPLETE);
        appendKeysFromFindings(boundaryFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
    }

    std::vector<Finding> passageCardinalityFindings;
    std::vector<Finding> passageProvenanceFindings;
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (!passage.isLive)
        {
            continue;
        }
        result.livePassageCount++;
        /* Reuses the identical
         * AX-PASS-01 leaf evaluateAxPass01() calls, rather than a
         * hand-written passable() check, so both paths flow through the
         * same shared logic. */
        evaluateOnePassageProvenance(passage, passageProvenanceFindings);
        evaluateOnePassageCardinality(passage,
                                      snapshot_in,
                                      passageCardinalityFindings);
    }
    const bool anyPassageCardinalityFail =
        anyFindingIs(passageCardinalityFindings, AxiomResult::FAIL);
    const bool anyPassageProvenanceFail =
        anyFindingIs(passageProvenanceFindings, AxiomResult::FAIL);

    /* AX-PASS-03 (slot state) and AX-PASS-04 (map/floor agreement):
     * conservative completeness must consume the same leaf checks as state
     * evaluation for these axioms too, not only AX-PASS-02 cardinality. */
    std::vector<Finding> passageSlotFindings;
    std::vector<Finding> passageMapFloorFindings;
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (!passage.isLive)
        {
            continue;
        }
        evaluateOnePassageSlotState(passage, snapshot_in, passageSlotFindings);
        evaluateOnePassageMapAndFloor(passage,
                                      snapshot_in,
                                      passageMapFloorFindings);
    }
    const bool anyPassageSlotFail =
        anyFindingIs(passageSlotFindings, AxiomResult::FAIL);
    const bool anyPassageMapFloorFail =
        anyFindingIs(passageMapFloorFindings, AxiomResult::FAIL);

    if (anyPassageProvenanceFail || anyPassageCardinalityFail ||
        anyPassageSlotFail || anyPassageMapFloorFail)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID);
        appendKeysFromFindings(passageProvenanceFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
        appendKeysFromFindings(passageCardinalityFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
        appendKeysFromFindings(passageSlotFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
        appendKeysFromFindings(passageMapFloorFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
    }

    std::vector<Finding> malformedPassageReferenceFindings;
    evaluateRoomMalformedPassageReferences(mapSnapshot_in,
                                           malformedPassageReferenceFindings);
    if (anyFindingIs(malformedPassageReferenceFindings, AxiomResult::FAIL))
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE);
        appendKeysFromFindings(malformedPassageReferenceFindings,
                               AxiomResult::FAIL,
                               relevantKeys);
    }

    std::vector<Finding> wallFindings;
    std::vector<Finding> twinFindings;
    for (const WallRecord &wall : mapSnapshot_in.walls)
    {
        if (!wall.isLive)
        {
            continue;
        }
        evaluateOneWall(wall, snapshot_in, wallFindings);
        evaluateOneWallTwin(wall, snapshot_in, twinFindings);
    }
    const bool anyWallFail = anyFindingIs(wallFindings, AxiomResult::FAIL);
    const bool anyWallUnknown =
        anyFindingIs(wallFindings, AxiomResult::UNKNOWN);
    const bool anyTwinFail = anyFindingIs(twinFindings, AxiomResult::FAIL);
    const bool anyTwinUnknown =
        anyFindingIs(twinFindings, AxiomResult::UNKNOWN);
    if (anyWallFail || anyTwinFail)
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_HARD_CONTRADICTION);
        appendKeysFromFindings(wallFindings, AxiomResult::FAIL, relevantKeys);
        appendKeysFromFindings(twinFindings, AxiomResult::FAIL, relevantKeys);
    }

    std::vector<Finding> floorFindings;
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (!room.isLive || room.variant != Room::RoomVariant::ROOM)
        {
            continue;
        }
        evaluateOneRoomFloorReciprocity(room,
                                        snapshot_in,
                                        mapSnapshot_in,
                                        floorFindings);
    }
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (!passage.isLive)
        {
            continue;
        }
        evaluateOnePassageFloorIdentity(passage, snapshot_in, floorFindings);
    }
    const bool anyFloorFail = anyFindingIs(floorFindings, AxiomResult::FAIL);
    const bool anyFloorUnknown =
        anyFindingIs(floorFindings, AxiomResult::UNKNOWN);
    if (anyFloorFail)
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_HARD_CONTRADICTION);
        appendKeysFromFindings(floorFindings, AxiomResult::FAIL, relevantKeys);
    }

    if (failReasons.empty())
    {
        if (result.livePassageCount > 0U)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE);
            appendKeysFromFindings(passageCardinalityFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
            appendKeysFromFindings(passageProvenanceFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
        if (anyBoundaryUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            appendKeysFromFindings(boundaryFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
        if (anyWallUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            appendKeysFromFindings(wallFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
        if (anyTwinUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            appendKeysFromFindings(twinFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
        if (anyFloorUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            appendKeysFromFindings(floorFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
        std::vector<Finding> roomProvenanceFindings;
        for (const RoomRecord &room : mapSnapshot_in.rooms)
        {
            if (!room.isLive || room.variant != Room::RoomVariant::ROOM)
            {
                continue;
            }
            /* Reuses the identical AX-ROOM-01
             * leaf evaluateAxRoom01() calls, rather than a hand-written
             * per-room loop, so both paths flow through the same shared
             * logic (mirroring the AX-PASS-01 provenance reuse above). */
            evaluateOneRoomCreationProvenance(room, roomProvenanceFindings);
        }
        if (anyFindingIs(roomProvenanceFindings, AxiomResult::UNKNOWN))
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE);
            appendKeysFromFindings(roomProvenanceFindings,
                                   AxiomResult::UNKNOWN,
                                   relevantKeys);
        }
    }

    /* fullyValidPassageCount stays 0 in this slice:
     * evaluateOnePassageCardinality() never returns AxiomResult::PASS
     * (PassageRecord::endpointSlotReason is always
     * NOT_TRACKED_BY_CURRENT_SCHEMA -- see its own Doxygen), and full validity
     * also requires same-floor and COMPLETE-endpoint proof this slice cannot
     * establish either. Left at its zero-initialized default rather than a loop
     * whose body could never execute. */

    std::sort(relevantKeys.begin(), relevantKeys.end());
    relevantKeys.erase(std::unique(relevantKeys.begin(), relevantKeys.end()),
                       relevantKeys.end());
    result.relevantEntityKeys = relevantKeys;

    std::sort(failReasons.begin(), failReasons.end());
    failReasons.erase(std::unique(failReasons.begin(), failReasons.end()),
                      failReasons.end());
    std::sort(unknownReasons.begin(), unknownReasons.end());
    unknownReasons.erase(
        std::unique(unknownReasons.begin(), unknownReasons.end()),
        unknownReasons.end());

    if (!failReasons.empty())
    {
        result.conservativeResult = AxiomResult::FAIL;
        result.reasons            = failReasons;
    }
    else if (!unknownReasons.empty())
    {
        result.conservativeResult = AxiomResult::UNKNOWN;
        result.reasons            = unknownReasons;
    }
    else
    {
        result.conservativeResult = AxiomResult::PASS;
    }
    result.isComplete = (result.conservativeResult == AxiomResult::PASS);

    return result;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
