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
 * @file            evaluateOneWallTwin.cc
 *
 * @brief           Implements evaluateOneWallTwin(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOneWallTwin(const WallRecord            &wall_in,
                        const SemanticGraphSnapshot &snapshot_in,
                        std::vector<Finding>        &findings_inout)
{
    const RawPlaneRef &twin = wall_in.twinRef;
    if (twin.reason != UnavailableReason::NONE)
    {
        /* RawPlaneRef documents
         * reason == NONE exactly when the underlying plane pointer was
         * non-null; a "reason claims absent" value that nonetheless carries
         * populated data (mapId/wallKey/a real planeType) is an invariant
         * violation and a known contradiction, not an ordinary absent
         * twin. */
        if (twin.mapId.has_value() || twin.wallKey.has_value() ||
            twin.planeType != geometric::Plane::PlaneVariant::UNDEFINED)
        {
            Finding finding{};
            if (makeFinding(AxiomCode::AX_WALL_03,
                            AxiomResult::FAIL,
                            ReasonCode::WALL_TWIN_REASON_INCONSISTENT,
                            {wall_in.key},
                            finding) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: makeFinding returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            findings_inout.push_back(finding);
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::PASS,
                        ReasonCode::WALL_TWIN_ABSENT,
                        {wall_in.key},
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

    if (twin.planeType != geometric::Plane::PlaneVariant::WALL)
    {
        Finding finding3{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_WRONG_TYPE,
                        {wall_in.key},
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (!twin.wallKey.has_value())
    {
        /* planeType == WALL but wallKey absent means the twin has no map
         * (rawPlaneRef()'s documented invariant); same-map/asymmetric/
         * shared-owner cannot be checked without one. */
        Finding finding4{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_TWIN_MAP_UNAVAILABLE,
                        {wall_in.key},
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (*twin.wallKey == wall_in.key)
    {
        Finding finding5{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_SELF,
                        {wall_in.key},
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (twin.wallKey->mapId != wall_in.key.mapId)
    {
        Finding finding6{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_CROSS_MAP,
                        {wall_in.key, *twin.wallKey},
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::size_t wallRecords{};
    if (countWallRecordsWithKey(snapshot_in, *twin.wallKey, wallRecords) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countWallRecordsWithKey returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (wallRecords > 1U)
    {
        Finding finding7{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_DUPLICATE_IDENTITY,
                        {wall_in.key, *twin.wallKey},
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
    std::size_t mapSnapshots{};
    if (countMapSnapshotsWithId(snapshot_in,
                                twin.wallKey->mapId,
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
        /* Which MapSnapshot actually holds
         * the twin is itself ambiguous when its own containing map id is
         * duplicated -- findWallByKeyInSnapshot()'s first-match lookup may
         * not supply positive proof in that case. */
        Finding finding8{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_CONTAINING_MAP_AMBIGUOUS,
                        {wall_in.key, *twin.wallKey},
                        finding8) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding8);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    const WallRecord *p_twinRecord = nullptr;
    if (findWallByKeyInSnapshot(snapshot_in, *twin.wallKey, p_twinRecord) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: findWallByKeyInSnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_twinRecord == nullptr)
    {
        /* Same map id as this wall, WALL-typed, but not locatable among the
         * captured active maps' wall records -- an anomaly this schema
         * cannot further diagnose (see badRetiredMapVisibility). */
        Finding finding9{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_TWIN_MAP_UNAVAILABLE,
                        {wall_in.key, *twin.wallKey},
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

    if (!p_twinRecord->isLive)
    {
        Finding finding10{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_BAD,
                        {wall_in.key, *twin.wallKey},
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

    const bool isSymmetric = p_twinRecord->twinRef.wallKey.has_value() &&
                             (*p_twinRecord->twinRef.wallKey == wall_in.key);
    if (!isSymmetric)
    {
        Finding finding11{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_ASYMMETRIC,
                        {wall_in.key, *twin.wallKey},
                        finding11) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding11);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::vector<EntityKey> sharedLiveOwnerKeys;
    for (const EntityRef &ownerA : wall_in.ownerRoomRefs)
    {
        if (!ownerA.key.has_value() ||
            !(ownerA.isLive.has_value() && *ownerA.isLive))
        {
            continue;
        }
        for (const EntityRef &ownerB : p_twinRecord->ownerRoomRefs)
        {
            if (ownerB.key.has_value() && *ownerB.key == *ownerA.key)
            {
                sharedLiveOwnerKeys.push_back(*ownerA.key);
            }
        }
    }
    if (!sharedLiveOwnerKeys.empty())
    {
        std::vector<EntityKey> involvedKeys{wall_in.key, *twin.wallKey};
        involvedKeys.insert(involvedKeys.end(),
                            sharedLiveOwnerKeys.begin(),
                            sharedLiveOwnerKeys.end());
        Finding finding12{};
        if (makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_SHARED_OWNER_FORBIDDEN,
                        std::move(involvedKeys),
                        finding12) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding12);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    Finding finding13{};
    if (makeFinding(
            AxiomCode::AX_WALL_03,
            AxiomResult::UNKNOWN,
            ReasonCode::WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED,
            {wall_in.key, *twin.wallKey},
            finding13) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding13);

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
