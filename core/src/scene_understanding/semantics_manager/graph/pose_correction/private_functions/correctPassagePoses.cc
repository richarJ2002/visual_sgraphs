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
 * @file            correctPassagePoses.cc
 *
 * @brief           Implements correctPassagePoses(), declared in
 *                  semantics_manager/graph/pose_correction/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/pose_correction/private_functions/private_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void correctPassagePoses(const PoseCorrectionField        &field_in,
                         std::vector<semantic::Passage *> &passages_inout)
{
    for (semantic::Passage *p_passage : passages_inout)
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        Eigen::Vector3d passageCentroid_oldWorld_m{};
        if (p_passage->getCentroid(passageCentroid_oldWorld_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const g2o::Sim3 *p_passageCorrection = nullptr;

        geometric::Plane *p_passageAssociateDoor = nullptr;
        if (p_passage->getAssociateDoor(p_passageAssociateDoor) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passageAssociateDoor != nullptr)
        {
            const std::map<geometric::Plane *, g2o::Sim3>::const_iterator
                correctionIterator =
                    field_in.planeCorrections_oldWorldToNewWorld.find(
                        p_passageAssociateDoor);

            if (correctionIterator !=
                field_in.planeCorrections_oldWorldToNewWorld.end())
            {
                p_passageCorrection = &correctionIterator->second;
            }
        }

        if (p_passageCorrection == nullptr)
        {
            double closestWallSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            std::vector<geometric::Plane *> passageAssociateWalls{};
            if (p_passage->getAssociateWalls(passageAssociateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAssociateWalls returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : passageAssociateWalls)
            {
                const std::map<geometric::Plane *,
                               g2o::Sim3>::const_iterator correctionIterator =
                    field_in.planeCorrections_oldWorldToNewWorld.find(p_wall);

                if (p_wall == nullptr ||
                    correctionIterator ==
                        field_in.planeCorrections_oldWorldToNewWorld.end())
                {
                    continue;
                }

                const std::map<geometric::Plane *,
                               Eigen::Vector3d>::const_iterator
                    centroidIterator =
                        field_in.planeCentroids_oldWorld_m.find(p_wall);

                if (centroidIterator ==
                    field_in.planeCentroids_oldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - passageCentroid_oldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_passageCorrection           = &correctionIterator->second;
                }
            }
        }

        if (p_passage->applyTransform(p_passageCorrection != nullptr
                                          ? *p_passageCorrection
                                          : field_in.selectCorrectionForPoint(
                                                passageCentroid_oldWorld_m)) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
