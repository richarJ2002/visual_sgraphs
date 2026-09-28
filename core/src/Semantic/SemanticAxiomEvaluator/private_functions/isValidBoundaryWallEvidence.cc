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
 * @file            isValidBoundaryWallEvidence.cc
 *
 * @brief           Implements isValidBoundaryWallEvidence(), declared in
 *                  private_functions.h.
 *
 *                  AX-BOUND-01 does not treat any nonempty
 *                  RoomRecord::wallRefs as boundary support. Each entry
 *                  must be present, WALL-typed, live, in the room's own
 *                  map, resolve to exactly one WallRecord (no
 *                  duplicate-identity ambiguity), and that WallRecord's
 *                  own ownerRoomRefs must resolve back to this room
 *                  (reciprocal ownership) -- otherwise it is not
 *                  trustworthy boundary evidence.
 *
 *                  A typed RoomBoundaryWallEvidenceStatus return lets
 *                  evaluateOneRoomBoundary() distinguish a proven
 *                  contradiction (INVALID -- wrong type, retired,
 *                  cross-map, ambiguous identity, or non-reciprocal) from
 *                  an ordinary evidence gap (UNAVAILABLE -- no reference
 *                  attempted, no map, or the target WallRecord is not
 *                  locatable). A known INVALID reference must never be
 *                  silently indistinguishable from merely unavailable
 *                  evidence.
 *
 *                  The raw reference's own wallKey/mapId/planeId field
 *                  consistency and wallKey.kind are additionally
 *                  validated, as are the resolved WallRecord's own
 *                  planeType and declared-map consistency; and
 *                  evaluateOneWall() itself is reused (rather than a
 *                  second, weaker reciprocal-ownership check) for the
 *                  final ownership-chain proof, so this function and
 *                  AX-WALL-01 can never define "a valid wall"
 *                  differently.
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

SemanticAxiomEvaluatorStatus isValidBoundaryWallEvidence(
    const RawPlaneRef              &wallReference_in,
    const RoomRecord               &room_in,
    const SemanticGraphSnapshot    &snapshot_in,
    RoomBoundaryWallEvidenceStatus &evidenceStatus_out)
{
    if (wallReference_in.reason != UnavailableReason::NONE)
    {
        /* RawPlaneRef documents
         * reason == NONE exactly when the underlying plane pointer was
         * non-null; a "reason claims absent" value that nonetheless carries
         * populated data (mapId/wallKey/a real planeType) is an invariant
         * violation and a known contradiction, not the ordinary "nothing
         * there" case. */
        if (wallReference_in.mapId.has_value() ||
            wallReference_in.wallKey.has_value() ||
            wallReference_in.planeType !=
                geometric::Plane::PlaneVariant::UNDEFINED)
        {
            evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::UNAVAILABLE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (wallReference_in.planeType != geometric::Plane::PlaneVariant::WALL)
    {
        /* A real, mapped, live plane pointer exists but is the wrong type:
         * a known contradiction, not merely missing evidence. */
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!wallReference_in.isLive)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!wallReference_in.mapId.has_value())
    {
        /* The plane exists and is live/WALL-typed but has no map of its
         * own: same-map/reciprocity cannot be verified either way. */
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::UNAVAILABLE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (*wallReference_in.mapId != room_in.key.mapId)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!wallReference_in.wallKey.has_value())
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::UNAVAILABLE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (wallReference_in.wallKey->kind != EntityKind::WALL ||
        wallReference_in.wallKey->mapId != *wallReference_in.mapId ||
        wallReference_in.wallKey->entityId != wallReference_in.planeId)
    {
        /* The wallKey field itself is internally inconsistent with this
         * same reference's own mapId/planeId/kind: a known contradiction in
         * the raw data, not an ordinary gap. */
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::size_t wallRecords{};
    if (countWallRecordsWithKey(snapshot_in,
                                *wallReference_in.wallKey,
                                wallRecords) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // countWallRecordsWithKey cannot fail; continue as before.
    }
    if (wallRecords > 1U)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    std::size_t mapSnapshots{};
    if (countMapSnapshotsWithId(snapshot_in,
                                wallReference_in.wallKey->mapId,
                                mapSnapshots) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // countMapSnapshotsWithId cannot fail; continue as before.
    }
    if (mapSnapshots > 1U)
    {
        /* Which MapSnapshot actually holds
         * this wall is itself ambiguous when its own containing map id is
         * duplicated -- no first-match lookup below may supply positive
         * proof. */
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    const WallRecord *p_wall = nullptr;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != wallReference_in.wallKey->mapId)
        {
            continue;
        }
        const WallRecord *p_record = nullptr;
        if (findRecordByKey(mapSnapshot.walls,
                            *wallReference_in.wallKey,
                            p_record) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // findRecordByKey cannot fail; continue as before.
        }
        p_wall = p_record;
        break;
    }
    if (p_wall == nullptr)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::UNAVAILABLE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!p_wall->isLive ||
        p_wall->planeType != geometric::Plane::PlaneVariant::WALL)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (p_wall->declaredMapId.has_value() &&
        *p_wall->declaredMapId != p_wall->key.mapId)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    /* Reuse the complete wall-record -> owner-ref -> live room-record ->
     * reciprocal wall-ref chain evaluateOneWall() already proves for
     * AX-WALL-01, rather than a second, independently maintained
     * reciprocal-ownership check that could drift out of agreement with
     * it. */
    std::vector<Finding> wallOwnershipScratch;
    if (evaluateOneWall(*p_wall, snapshot_in, wallOwnershipScratch) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateOneWall cannot fail; continue as before.
    }
    /* The aggregate AX-WALL-01
     * result for this wall must be a clean PASS -- a PASS finding
     * accompanied by an UNKNOWN (e.g. the wall's or owner's own declared map
     * being genuinely absent) is not full positive proof and must not be
     * reused as valid boundary evidence. */
    bool hasFinding{};
    if (anyFindingIs(wallOwnershipScratch, AxiomResult::FAIL, hasFinding) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // anyFindingIs cannot fail; continue as before.
    }
    if (hasFinding)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    bool hasFinding2{};
    if (anyFindingIs(wallOwnershipScratch, AxiomResult::UNKNOWN, hasFinding2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // anyFindingIs cannot fail; continue as before.
    }
    if (hasFinding2)
    {
        evidenceStatus_out = RoomBoundaryWallEvidenceStatus::UNAVAILABLE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    for (const Finding &finding : wallOwnershipScratch)
    {
        if (finding.result == AxiomResult::PASS &&
            finding.reasonCode ==
                ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER &&
            std::find(finding.involvedKeys.begin(),
                      finding.involvedKeys.end(),
                      room_in.key) != finding.involvedKeys.end())
        {
            evidenceStatus_out = RoomBoundaryWallEvidenceStatus::VALID;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
    }
    evidenceStatus_out = RoomBoundaryWallEvidenceStatus::INVALID;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
