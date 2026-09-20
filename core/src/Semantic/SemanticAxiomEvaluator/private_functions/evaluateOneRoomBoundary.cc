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
 * @file         evaluateOneRoomBoundary.cc
 *
 * @brief        Implements evaluateOneRoomBoundary(), declared in
 *               private_functions.h.
 *
 *               A non-finite corner is an explicit FAIL
 *               (checkRoomBoundaryGeometry()'s NON_FINITE_CORNER status);
 *               wall evidence is validated per-reference by
 *               isValidBoundaryWallEvidence() rather than accepted merely
 *               for being a nonempty vector; and this evaluator never emits
 *               ROOM_BOUNDARY_STRUCTURALLY_VALID/PASS -- edge-to-wall and
 *               gap-to-aperture geometric correspondence are not
 *               implemented here, so a COMPLETE room with otherwise-valid
 *               geometry and at least one verified, live, reciprocal,
 *               same-map WALL reference is UNKNOWN, never PASS.
 *
 *               Each RoomRecord::wallRefs entry's typed
 *               RoomBoundaryWallEvidenceStatus is inspected individually
 *               rather than merely counted as a boolean: a known-INVALID
 *               reference is a FAIL even when another reference is VALID,
 *               so a provable contradiction cannot be hidden behind one
 *               otherwise-valid reference or silently reach the
 *               edge-support-coverage UNKNOWN.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOneRoomBoundary(const RoomRecord            &room_in,
                             const SemanticGraphSnapshot &snapshot_in,
                             std::vector<Finding>        &findings_inout)
{
    const std::vector<EntityKey> involvedKeys{room_in.key};

    if (room_in.boundaryStatus == Room::BoundaryStatus::CONFLICTING)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_CONFLICTING_STATE,
                        involvedKeys));
        return;
    }

    if (room_in.boundaryStatus != Room::BoundaryStatus::COMPLETE)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_NOT_YET_COMPLETE,
                        involvedKeys));
        return;
    }

    switch (checkRoomBoundaryGeometry(room_in.boundaryCorners_World_m))
    {
    case RoomBoundaryGeometryStatus::TOO_FEW_CORNERS:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_TOO_FEW_CORNERS,
                        involvedKeys));
        return;
    case RoomBoundaryGeometryStatus::NON_FINITE_CORNER:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER,
                        involvedKeys));
        return;
    case RoomBoundaryGeometryStatus::DEGENERATE_EDGE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_DEGENERATE_EDGE,
                        involvedKeys));
        return;
    case RoomBoundaryGeometryStatus::SELF_INTERSECTING:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_SELF_INTERSECTING,
                        involvedKeys));
        return;
    case RoomBoundaryGeometryStatus::VALID:
        break;
    }

    std::size_t validWallEvidenceCount = 0U;
    bool        anyInvalidWallEvidence = false;
    for (const RawPlaneRef &wallRef : room_in.wallRefs)
    {
        switch (isValidBoundaryWallEvidence(wallRef, room_in, snapshot_in))
        {
        case RoomBoundaryWallEvidenceStatus::VALID:
            ++validWallEvidenceCount;
            break;
        case RoomBoundaryWallEvidenceStatus::INVALID:
            anyInvalidWallEvidence = true;
            break;
        case RoomBoundaryWallEvidenceStatus::UNAVAILABLE:
            break;
        }
    }
    if (anyInvalidWallEvidence)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE,
                        involvedKeys));
        return;
    }
    if (validWallEvidenceCount == 0U)
    {
        if (room_in.wallRefs.empty())
        {
            findings_inout.push_back(
                makeFinding(AxiomCode::AX_BOUND_01,
                            AxiomResult::FAIL,
                            ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE,
                            involvedKeys));
            return;
        }
        /* A nonempty wallRefs collection whose every entry is genuinely
         * UNAVAILABLE (no entry independently proven INVALID, already
         * excluded above) is an evidence gap, not the same contradiction as
         * a room with no wall evidence at all. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE,
                        involvedKeys));
        return;
    }

    if (!room_in.observationGaps.empty())
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED,
                        involvedKeys));
        return;
    }

    findings_inout.push_back(
        makeFinding(AxiomCode::AX_BOUND_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED,
                    involvedKeys));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
