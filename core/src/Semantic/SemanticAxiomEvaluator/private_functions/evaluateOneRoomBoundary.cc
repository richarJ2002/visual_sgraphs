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
 * @file            evaluateOneRoomBoundary.cc
 *
 * @brief           Implements evaluateOneRoomBoundary(), declared in
 *                  private_functions.h.
 *
 *                  A non-finite corner is an explicit FAIL
 *                  (checkRoomBoundaryGeometry()'s NON_FINITE_CORNER status);
 *                  wall evidence is validated per-reference by
 *                  isValidBoundaryWallEvidence() rather than accepted merely
 *                  for being a nonempty vector; and this evaluator never emits
 *                  ROOM_BOUNDARY_STRUCTURALLY_VALID/PASS -- edge-to-wall and
 *                  gap-to-aperture geometric correspondence are not
 *                  implemented here, so a COMPLETE room with otherwise-valid
 *                  geometry and at least one verified, live, reciprocal,
 *                  same-map WALL reference is UNKNOWN, never PASS.
 *
 *                  Each RoomRecord::wallRefs entry's typed
 *                  RoomBoundaryWallEvidenceStatus is inspected individually
 *                  rather than merely counted as a boolean: a known-INVALID
 *                  reference is a FAIL even when another reference is VALID,
 *                  so a provable contradiction cannot be hidden behind one
 *                  otherwise-valid reference or silently reach the
 *                  edge-support-coverage UNKNOWN.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOneRoomBoundary(const RoomRecord            &room_in,
                            const SemanticGraphSnapshot &snapshot_in,
                            std::vector<Finding>        &findings_inout)
{
    const std::vector<EntityKey> involvedKeys{room_in.key};

    if (room_in.boundaryStatus == Room::BoundaryStatus::CONFLICTING)
    {
        Finding finding{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_CONFLICTING_STATE,
                        involvedKeys,
                        finding) != SemanticAxiomEvaluatorStatus::
                                        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (room_in.boundaryStatus != Room::BoundaryStatus::COMPLETE)
    {
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_NOT_YET_COMPLETE,
                        involvedKeys,
                        finding2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding2);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    RoomBoundaryGeometryStatus geometryStatus{};
    if (checkRoomBoundaryGeometry(room_in.boundaryCorners_world_m,
                                  geometryStatus) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkRoomBoundaryGeometry returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    switch (geometryStatus)
    {
    case RoomBoundaryGeometryStatus::TOO_FEW_CORNERS:
    {
        Finding finding3{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_TOO_FEW_CORNERS,
                        involvedKeys,
                        finding3) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding3);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case RoomBoundaryGeometryStatus::NON_FINITE_CORNER:
    {
        Finding finding4{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER,
                        involvedKeys,
                        finding4) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding4);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case RoomBoundaryGeometryStatus::DEGENERATE_EDGE:
    {
        Finding finding5{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_DEGENERATE_EDGE,
                        involvedKeys,
                        finding5) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding5);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case RoomBoundaryGeometryStatus::SELF_INTERSECTING:
    {
        Finding finding6{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_SELF_INTERSECTING,
                        involvedKeys,
                        finding6) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding6);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case RoomBoundaryGeometryStatus::VALID:
        break;
    }

    std::size_t validWallEvidenceCount = 0U;
    bool        anyInvalidWallEvidence = false;
    for (const RawPlaneRef &wallReference : room_in.wallRefs)
    {
        RoomBoundaryWallEvidenceStatus evidenceStatus{};
        if (isValidBoundaryWallEvidence(wallReference,
                                        room_in,
                                        snapshot_in,
                                        evidenceStatus) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isValidBoundaryWallEvidence returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        switch (evidenceStatus)
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
        Finding finding7{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::FAIL,
                        ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE,
                        involvedKeys,
                        finding7) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding7);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (validWallEvidenceCount == 0U)
    {
        if (room_in.wallRefs.empty())
        {
            Finding finding8{};
            if (makeFinding(AxiomCode::AX_BOUND_01,
                            AxiomResult::FAIL,
                            ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE,
                            involvedKeys,
                            finding8) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: makeFinding returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            findings_inout.push_back(finding8);
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        /* A nonempty wallRefs collection whose every entry is genuinely
         * UNAVAILABLE (no entry independently proven INVALID, already
         * excluded above) is an evidence gap, not the same contradiction as
         * a room with no wall evidence at all. */
        Finding finding9{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE,
                        involvedKeys,
                        finding9) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding9);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (!room_in.observationGaps.empty())
    {
        Finding finding10{};
        if (makeFinding(AxiomCode::AX_BOUND_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED,
                        involvedKeys,
                        finding10) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding10);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    Finding finding11{};
    if (makeFinding(AxiomCode::AX_BOUND_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED,
                    involvedKeys,
                    finding11) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding11);

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
