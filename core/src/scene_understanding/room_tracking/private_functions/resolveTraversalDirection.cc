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
 * @file            resolveTraversalDirection.cc
 *
 * @brief           Implements resolveTraversalDirection(), declared in
 *                  room_tracking/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"

#include "scene_understanding/room_tracking/private_functions/private_functions.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus resolveTraversalDirection(
    semantic::Passage                      *p_passage_inout,
    const Eigen::Vector3d                  &crossingStart_world_m_in,
    double                                  minimumSideDistance_m_in,
    semantic::Passage::KnownSideProvenance &knownSide_out,
    semantic::Passage::TraversalDirection  &traversalDirection_out)
{
    traversalDirection_out = semantic::Passage::TraversalDirection::UNKNOWN;
    if (p_passage_inout->getKnownSideProvenance(knownSide_out) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool knownSideHasDirection{};
    if (knownSide_out.hasDirection(knownSideHasDirection) !=
        semantic::KnownSideProvenanceStatus::
            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!knownSideHasDirection)
    {
        g2o::Plane3D passageGlobalEquation{};
        if (p_passage_inout->getGlobalEquation(passageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d passageEquation = passageGlobalEquation.coeffs();
        const double    normalNorm      = passageEquation.head<3>().norm();
        if (passageEquation.allFinite() && normalNorm > 1e-8)
        {
            passageEquation /= normalNorm;
            const double observationSide =
                passageEquation.head<3>().dot(crossingStart_world_m_in) +
                passageEquation(3);
            if (std::abs(observationSide) > minimumSideDistance_m_in)
            {
                if (p_passage_inout->setKnownSideDirection(
                        (observationSide > 0.0 ? 1.0 : -1.0) *
                        passageEquation.head<3>()) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: setKnownSideDirection rejected its "
                                "input; continuing as before.",
                                __func__);
                }
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance{};
                if (p_passage_inout->getKnownSideProvenance(
                        passageKnownSideProvenance) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    /*!
                     * getKnownSideProvenance cannot fail; continue
                     * as before.
                     */
                }
                knownSide_out = passageKnownSideProvenance;
            }
        }
    }
    bool knownSideHasDirection2{};
    if (knownSide_out.hasDirection(knownSideHasDirection2) !=
        semantic::KnownSideProvenanceStatus::
            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (knownSideHasDirection2)
    {
        Eigen::Vector3d passageCentroid{};
        if (p_passage_inout->getCentroid(passageCentroid) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d startFromPassage_world_m =
            crossingStart_world_m_in - passageCentroid;
        traversalDirection_out =
            startFromPassage_world_m.dot(
                knownSide_out.knownSideDirection_world) >= 0.0
                ? semantic::Passage::TraversalDirection::KNOWN_TO_FAR
                : semantic::Passage::TraversalDirection::FAR_TO_KNOWN;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
