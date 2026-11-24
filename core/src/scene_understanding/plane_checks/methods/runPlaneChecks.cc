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
 * @file            runPlaneChecks.cc
 *
 * @brief           Implements SemanticsManager::runPlaneChecks(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runPlaneChecks(
    const std::vector<geometric::Plane *> &intakePlanes_in)
{
    /* The one up direction of this stage (the map-start camera's -Y unless
     * the active map is inertial). */
    Eigen::Vector3d upDirection_world{};
    bool            isFromImuGravity{};
    if (p_atlas->getUpDirection(upDirection_world, isFromImuGravity) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Geometrically test planes to make sure normals are expected */
    if (rejectMisorientedPlanes(intakePlanes_in, upDirection_world) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rejectMisorientedPlanes returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* The main ground plane is chosen only after the tilt check, so a
     * rejected plane cannot become the height reference. */
    geometric::Plane *p_mainGroundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_mainGroundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mainGroundPlane == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Find the transform from the main ground plane to horizontal */
    Eigen::Matrix4f groundPlaneToHorizontal{};
    if (computePlaneToHorizontal(p_mainGroundPlane,
                                 upDirection_world,
                                 groundPlaneToHorizontal) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computePlaneToHorizontal returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    groundPlanToHorizontalTransformation = groundPlaneToHorizontal;

    /* Drop ground planes past the step limit from the main floor */
    if (filterGroundPlanes(intakePlanes_in,
                           p_mainGroundPlane,
                           upDirection_world) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: filterGroundPlanes returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
