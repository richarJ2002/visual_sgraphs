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
 * @file            computeConservativeMapCompleteness.cc
 *
 * @brief           Implements computeConservativeMapCompleteness(),
 *                  declared in private_functions.h.
 *
 *                  Replaces the
 *                  pre-repair hand-written subset (room/prospective
 *                  counts, a passage-endpoint-only re-derivation, and
 *                  a wall-ownership duplicate of AX-WALL-01's own
 *                  logic) with direct reuse of the same map-scoped,
 *                  per-entity leaf evaluators
 *                  evaluateAxWall01.cc/evaluateAxWall03.cc/
 *                  evaluateAxBound01.cc/evaluateAxFloor01.cc/
 *                  evaluateAxPass02.cc already call --
 *                  evaluateOneWall()/evaluateOneWallTwin()/
 *                  evaluateOneRoomBoundary()/
 *                  evaluateOneRoomFloorReciprocity()/
 *                  evaluateOnePassageCardinality() -- so the two paths
 *                  cannot drift apart. This function still never calls
 *                  evaluateState() or evaluateMapCompleteness() itself,
 *                  avoiding the circular dependency
 *                  evaluateAxComp01.cc's own Doxygen documents
 *                  (evaluateState() calls evaluateMapCompleteness() to
 *                  build its AX-COMP-01 Finding); calling the leaf
 *                  per-entity functions directly is not a call back
 *                  into that orchestrator.
 *
 *                  Because AX-BOUND-01 and AX-PASS-02 can now only
 *                  report FAIL or UNKNOWN (their capability level is
 *                  PARTIAL/DEFERRED -- see
 *                  computeAxiomCapabilityTable()), any map with at
 *                  least one live passage or one confirmed
 *                  COMPLETE-status room is capped at UNKNOWN here too,
 *                  never PASS. This is the honest, schema-limited
 *                  consequence, not a defect: "If the accepted schema
 *                  makes full positive proof impossible, that is an
 *                  honest result, not a reason to fabricate a PASS."
 *
 *                     Rules, in the order checked (any FAIL-class condition
 *                     found anywhere makes the whole map FAIL; otherwise any
 *                     UNKNOWN-class condition makes it UNKNOWN; otherwise
 *                     PASS):
 *                     - zero confirmed (live, ROOM-variant) rooms -> FAIL;
 *                     - any live prospective (UNDEFINED-variant) room -> FAIL;
 *                     - any confirmed room whose model-reported boundaryStatus
 *                       is not COMPLETE, or whose COMPLETE geometry
 *                       independently fails evaluateOneRoomBoundary() -> FAIL;
 *                     - any non-passable live passage, or any live passage
 *                       whose evaluateOnePassageCardinality() reports FAIL ->
 *                       FAIL (COMPLETENESS_PASSAGE_ENDPOINTS_INVALID);
 *                     - any live wall whose evaluateOneWall() reports FAIL, or
 *                       any live wall whose evaluateOneWallTwin() reports FAIL
 *                       -> FAIL (COMPLETENESS_HARD_CONTRADICTION);
 *                     - any confirmed room whose
 *                       evaluateOneRoomFloorReciprocity() reports FAIL -> FAIL
 *                       (COMPLETENESS_HARD_CONTRADICTION);
 *                     - otherwise, any live passage (cardinality proof is
 *                       permanently UNKNOWN pending authoritative endpoint
 *                       slots), any UNKNOWN wall/twin/floor/boundary finding
 *                       above -> UNKNOWN; and
 *                     - otherwise PASS.
 *
 *                     A map-local duplicate same-key room, wall, passage,
 *                     or floor record is an unconditional FAIL
 *                     (COMPLETENESS_DUPLICATE_IDENTITY); every live
 *                     passage's AX-PASS-03 (evaluateOnePassageSlotState())
 *                     and AX-PASS-04 (evaluateOnePassageMapAndFloor()) FAIL
 *                     findings also drive COMPLETENESS_PASSAGE_ENDPOINTS_
 *                     INVALID, not only AX-PASS-02 cardinality; every live
 *                     passage's AX-FLOOR-01 endpoint-floor-identity
 *                     (evaluateOnePassageFloorIdentity()) finding
 *                     contributes to the same floor FAIL/UNKNOWN aggregation
 *                     as the room-floor branch; and any confirmed room also
 *                     contributes COMPLETENESS_ROOM_CREATION_PROVENANCE_
 *                     UNAVAILABLE (RoomRecord::creationProvenanceReason is
 *                     always NOT_TRACKED_BY_CURRENT_SCHEMA).
 *
 *                     A duplicate MapSnapshot::mapId sharing this map's own
 *                     id is an unconditional FAIL (COMPLETENESS_DUPLICATE_
 *                     MAP_IDENTITY); every live passage's AX-PASS-01
 *                     provenance reuses evaluateOnePassageProvenance() (the
 *                     same leaf evaluateAxPass01() calls) instead of a
 *                     hand-written passable() check, and its FAIL also drives
 *                     COMPLETENESS_PASSAGE_ENDPOINTS_INVALID; and every live
 *                     room's malformed (unkeyed) passageRefs entries, found
 *                     via evaluateRoomMalformedPassageReferences() (the same
 *                     leaf evaluateAxPass02() calls), drive a dedicated
 *                     COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE
 *                     FAIL.
 *
 *                     Every confirmed room's
 *                     room-creation-provenance UNKNOWN now reuses the identical
 *                     AX-ROOM-01 leaf evaluateOneRoomCreationProvenance() (the
 *                     same leaf evaluateAxRoom01() calls), replacing a
 *                     hand-written per-room loop that duplicated its logic.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>
#include <cstddef>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus computeConservativeMapCompleteness(
    const SemanticGraphSnapshot &snapshot_in,
    const MapSnapshot           &mapSnapshot_in,
    MapCompletenessResult       &conservativeMapCompleteness_out)
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

    std::size_t mapSnapshots{};
    if (countMapSnapshotsWithId(snapshot_in,
                                mapSnapshot_in.mapId,
                                mapSnapshots) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMapSnapshotsWithId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (mapSnapshots > 1U)
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
        std::size_t roomRecords{};
        if (countRoomRecordsWithKey(snapshot_in, room.key, roomRecords) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: countRoomRecordsWithKey returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (roomRecords > 1U)
        {
            duplicateIdentityKeys.push_back(room.key);
        }
    }
    for (const WallRecord &wall : mapSnapshot_in.walls)
    {
        std::size_t wallRecords{};
        if (countWallRecordsWithKey(snapshot_in, wall.key, wallRecords) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: countWallRecordsWithKey returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (wallRecords > 1U)
        {
            duplicateIdentityKeys.push_back(wall.key);
        }
    }
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        std::size_t passageRecords{};
        if (countPassageRecordsWithKey(snapshot_in,
                                       passage.key,
                                       passageRecords) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: countPassageRecordsWithKey returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (passageRecords > 1U)
        {
            duplicateIdentityKeys.push_back(passage.key);
        }
    }
    for (const FloorRecord &floor : mapSnapshot_in.floors)
    {
        std::size_t floorRecords{};
        if (countFloorRecordsWithKey(snapshot_in, floor.key, floorRecords) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: countFloorRecordsWithKey returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (floorRecords > 1U)
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
        if (evaluateOneRoomBoundary(room, snapshot_in, boundaryFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOneRoomBoundary returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    bool anyBoundaryFail{};
    if (anyFindingIs(boundaryFindings, AxiomResult::FAIL, anyBoundaryFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyBoundaryUnknown{};
    if (anyFindingIs(boundaryFindings,
                     AxiomResult::UNKNOWN,
                     anyBoundaryUnknown) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (result.completeRoomCount != result.confirmedRoomCount ||
        anyBoundaryFail)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_ROOM_BOUNDARY_NOT_COMPLETE);
        if (appendKeysFromFindings(boundaryFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
        if (evaluateOnePassageProvenance(passage, passageProvenanceFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOnePassageProvenance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (evaluateOnePassageCardinality(passage,
                                          snapshot_in,
                                          passageCardinalityFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOnePassageCardinality returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    bool anyPassageCardinalityFail{};
    if (anyFindingIs(passageCardinalityFindings,
                     AxiomResult::FAIL,
                     anyPassageCardinalityFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyPassageProvenanceFail{};
    if (anyFindingIs(passageProvenanceFindings,
                     AxiomResult::FAIL,
                     anyPassageProvenanceFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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
        if (evaluateOnePassageSlotState(passage,
                                        snapshot_in,
                                        passageSlotFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOnePassageSlotState returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (evaluateOnePassageMapAndFloor(passage,
                                          snapshot_in,
                                          passageMapFloorFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOnePassageMapAndFloor returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    bool anyPassageSlotFail{};
    if (anyFindingIs(passageSlotFindings,
                     AxiomResult::FAIL,
                     anyPassageSlotFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyPassageMapFloorFail{};
    if (anyFindingIs(passageMapFloorFindings,
                     AxiomResult::FAIL,
                     anyPassageMapFloorFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (anyPassageProvenanceFail || anyPassageCardinalityFail ||
        anyPassageSlotFail || anyPassageMapFloorFail)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID);
        if (appendKeysFromFindings(passageProvenanceFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (appendKeysFromFindings(passageCardinalityFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (appendKeysFromFindings(passageSlotFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (appendKeysFromFindings(passageMapFloorFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::vector<Finding> malformedPassageReferenceFindings;
    if (evaluateRoomMalformedPassageReferences(
            mapSnapshot_in,
            malformedPassageReferenceFindings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateRoomMalformedPassageReferences cannot fail; continue as
        // before.
    }
    bool hasFinding{};
    if (anyFindingIs(malformedPassageReferenceFindings,
                     AxiomResult::FAIL,
                     hasFinding) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (hasFinding)
    {
        failReasons.push_back(
            ReasonCode::COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE);
        if (appendKeysFromFindings(malformedPassageReferenceFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::vector<Finding> wallFindings;
    std::vector<Finding> twinFindings;
    for (const WallRecord &wall : mapSnapshot_in.walls)
    {
        if (!wall.isLive)
        {
            continue;
        }
        if (evaluateOneWall(wall, snapshot_in, wallFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: evaluateOneWall returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (evaluateOneWallTwin(wall, snapshot_in, twinFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: evaluateOneWallTwin returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    bool anyWallFail{};
    if (anyFindingIs(wallFindings, AxiomResult::FAIL, anyWallFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyWallUnknown{};
    if (anyFindingIs(wallFindings, AxiomResult::UNKNOWN, anyWallUnknown) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyTwinFail{};
    if (anyFindingIs(twinFindings, AxiomResult::FAIL, anyTwinFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyTwinUnknown{};
    if (anyFindingIs(twinFindings, AxiomResult::UNKNOWN, anyTwinUnknown) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (anyWallFail || anyTwinFail)
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_HARD_CONTRADICTION);
        if (appendKeysFromFindings(wallFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (appendKeysFromFindings(twinFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::vector<Finding> floorFindings;
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (!room.isLive || room.variant != Room::RoomVariant::ROOM)
        {
            continue;
        }
        if (evaluateOneRoomFloorReciprocity(room,
                                            snapshot_in,
                                            mapSnapshot_in,
                                            floorFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOneRoomFloorReciprocity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (!passage.isLive)
        {
            continue;
        }
        if (evaluateOnePassageFloorIdentity(passage,
                                            snapshot_in,
                                            floorFindings) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOnePassageFloorIdentity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    bool anyFloorFail{};
    if (anyFindingIs(floorFindings, AxiomResult::FAIL, anyFloorFail) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool anyFloorUnknown{};
    if (anyFindingIs(floorFindings, AxiomResult::UNKNOWN, anyFloorUnknown) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: anyFindingIs returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (anyFloorFail)
    {
        failReasons.push_back(ReasonCode::COMPLETENESS_HARD_CONTRADICTION);
        if (appendKeysFromFindings(floorFindings,
                                   AxiomResult::FAIL,
                                   relevantKeys) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendKeysFromFindings returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (failReasons.empty())
    {
        if (result.livePassageCount > 0U)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE);
            if (appendKeysFromFindings(passageCardinalityFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (appendKeysFromFindings(passageProvenanceFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (anyBoundaryUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            if (appendKeysFromFindings(boundaryFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (anyWallUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            if (appendKeysFromFindings(wallFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (anyTwinUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            if (appendKeysFromFindings(twinFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        if (anyFloorUnknown)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE);
            if (appendKeysFromFindings(floorFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            if (evaluateOneRoomCreationProvenance(room,
                                                  roomProvenanceFindings) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // evaluateOneRoomCreationProvenance cannot fail; continue as
                // before.
            }
        }
        bool hasFinding2{};
        if (anyFindingIs(roomProvenanceFindings,
                         AxiomResult::UNKNOWN,
                         hasFinding2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: anyFindingIs returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (hasFinding2)
        {
            unknownReasons.push_back(
                ReasonCode::COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE);
            if (appendKeysFromFindings(roomProvenanceFindings,
                                       AxiomResult::UNKNOWN,
                                       relevantKeys) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: appendKeysFromFindings returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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

    conservativeMapCompleteness_out = result;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
