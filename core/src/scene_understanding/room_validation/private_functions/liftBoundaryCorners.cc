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
 * @file            liftBoundaryCorners.cc
 *
 * @brief           Implements liftBoundaryCorners(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus liftBoundaryCorners(
    const std::vector<Eigen::Vector2d>     &boundaryCorners_world_m_in,
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector3d                  &groundNormal_world_in,
    const Eigen::Vector3d                  &groundAxisU_world_in,
    const Eigen::Vector3d                  &groundAxisV_world_in,
    std::vector<Eigen::Vector3d>           &boundaryCorners3d_world_m_out)
{
    boundaryCorners3d_world_m_out.clear();
    boundaryCorners3d_world_m_out.reserve(boundaryCorners_world_m_in.size());
    for (std::size_t cornerIndex = 0U;
         cornerIndex < boundaryCorners_world_m_in.size();
         ++cornerIndex)
    {
        const geometric::Plane *p_currentCornerWall =
            wallSegments_in[cornerIndex].p_wall;
        const geometric::Plane *p_nextCornerWall =
            wallSegments_in[(cornerIndex + 1U) % wallSegments_in.size()].p_wall;
        double height_m = 0.0;
        if (p_currentCornerWall != nullptr && p_nextCornerWall != nullptr)
        {
            Eigen::Vector3d currentCornerWallGetCentroid{};
            if (p_currentCornerWall->getCentroid(
                    currentCornerWallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3d nextCornerWallGetCentroid{};
            if (p_nextCornerWall->getCentroid(nextCornerWallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            height_m = 0.5 * (currentCornerWallGetCentroid.cast<double>().dot(
                                  groundNormal_world_in) +
                              nextCornerWallGetCentroid.cast<double>().dot(
                                  groundNormal_world_in));
        }
        boundaryCorners3d_world_m_out.push_back(
            boundaryCorners_world_m_in[cornerIndex].x() * groundAxisU_world_in +
            boundaryCorners_world_m_in[cornerIndex].y() * groundAxisV_world_in +
            height_m * groundNormal_world_in);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
