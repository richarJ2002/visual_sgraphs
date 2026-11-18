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
 * @file            isWallSeenThroughPassage.cc
 *
 * @brief           Implements SemanticsManager::isWallSeenThroughPassage(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::isWallSeenThroughPassage(
    semantic::Passage                        *p_passage_in,
    const Eigen::Vector3d                    &wallCentroid_world_m_in,
    const geometric::Plane::GeometrySnapshot &wallGeometry_in,
    const Eigen::Vector3d                    &viewpoint_world_m_in,
    const Eigen::Vector3d                    &groundNormal_world_in,
    double                                    minimumSideDistance_m_in,
    bool                                     &isSeenThrough_out)
{
    isSeenThrough_out = false;

    bool crossesToCentroid{};
    if (segmentCrossesPassageOpening(
            viewpoint_world_m_in,
            wallCentroid_world_m_in,
            p_passage_in,
            groundNormal_world_in,
            static_cast<double>(
                p_sysParams->roomSeg.passagePartition.openingMargin_m),
            minimumSideDistance_m_in,
            crossesToCentroid) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: segmentCrossesPassageOpening returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    if (crossesToCentroid)
    {
        isSeenThrough_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallSupportCloud =
        wallGeometry_in.supportCloud;
    if (p_wallSupportCloud == nullptr || p_wallSupportCloud->empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    /* ponytail: fixed stride to about 200 tested points per wall
     * and passage; replace with a per-cycle cached decision if the
     * every-cycle sweep ever shows in the semantic cycle time. */
    const std::size_t supportPointStride =
        std::max<std::size_t>(1U, p_wallSupportCloud->size() / 200U);
    std::size_t testedPointCount   = 0U;
    std::size_t crossingPointCount = 0U;
    for (std::size_t pointIndex = 0U; pointIndex < p_wallSupportCloud->size();
         pointIndex += supportPointStride)
    {
        const pcl::PointXYZRGBA &supportPoint =
            p_wallSupportCloud->points[pointIndex];
        if (!pcl::isFinite(supportPoint))
        {
            continue;
        }
        ++testedPointCount;
        bool crossesToSupportPoint{};
        if (segmentCrossesPassageOpening(
                viewpoint_world_m_in,
                Eigen::Vector3d(static_cast<double>(supportPoint.x),
                                static_cast<double>(supportPoint.y),
                                static_cast<double>(supportPoint.z)),
                p_passage_in,
                groundNormal_world_in,
                static_cast<double>(
                    p_sysParams->roomSeg.passagePartition.openingMargin_m),
                minimumSideDistance_m_in,
                crossesToSupportPoint) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: segmentCrossesPassageOpening returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
        if (crossesToSupportPoint)
        {
            ++crossingPointCount;
        }
    }
    isSeenThrough_out = 2U * crossingPointCount > testedPointCount;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
