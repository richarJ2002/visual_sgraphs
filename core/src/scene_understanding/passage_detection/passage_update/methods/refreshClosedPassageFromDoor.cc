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
 * @file            refreshClosedPassageFromDoor.cc
 *
 * @brief           Implements SemanticsManager::refreshClosedPassageFromDoor(),
 * declared in SemanticsManagerClass.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::refreshClosedPassageFromDoor(
    semantic::Passage                 *p_passage_inout,
    vs_graphs::core::geometric::Plane *p_doorPlane_in)
{
    if (p_doorPlane_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract width height supple of door */
    std::pair<double, double>          widthHeight{};
    geometric::Plane::GeometrySnapshot doorPlaneGetGeometrySnapshot{};
    if (p_doorPlane_in->getGeometrySnapshot(doorPlaneGetGeometrySnapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (utils::utils::Utils::computePlaneWidthHeight(
            doorPlaneGetGeometrySnapshot.supportCloud,
            widthHeight) != utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computePlaneWidthHeight returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Extract the measured height and width */
    const double measuredWidth  = widthHeight.first;
    const double measuredHeight = widthHeight.second;

    /* Extract the max width */
    const double maximumWidth =
        static_cast<double>(p_sysParams->semSeg.maxDoorWidth);

    /* Extract the max height */
    const double maximumHeight =
        static_cast<double>(p_sysParams->semSeg.maxDoorHeight);

    /* Determine if dimensions are valid */
    const bool validDimensions =
        std::isfinite(measuredWidth) && std::isfinite(measuredHeight) &&
        measuredWidth > 0.0 && measuredHeight > 0.0 &&
        measuredWidth <= maximumWidth && measuredHeight <= maximumHeight;

    if (!validDimensions)
    {
        int passageId{};
        if (p_passage_inout->getId(passageId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        int doorPlaneGetId{};
        if (p_doorPlane_in->getId(doorPlaneGetId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemanticsManager] Rejecting door plane "
                  << doorPlaneGetId << " for passage " << passageId
                  << ": measured dimensions " << measuredWidth << "x"
                  << measuredHeight << " m exceed limits " << maximumWidth
                  << "x" << maximumHeight << " m." << std::endl;

        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Set centroid of the door plane */
    Eigen::Vector3d doorPlaneGetCentroid{};
    if (p_doorPlane_in->getCentroid(doorPlaneGetCentroid) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_inout->setCentroid(doorPlaneGetCentroid) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Get the plane global equation */
    g2o::Plane3D doorPlaneGetGlobalEquation{};
    if (p_doorPlane_in->getGlobalEquation(doorPlaneGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_inout->setGlobalEquation(doorPlaneGetGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Set width & height of the passage */
    if (p_passage_inout->setWidth(measuredWidth) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_inout->setHeight(measuredHeight) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setHeight returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
