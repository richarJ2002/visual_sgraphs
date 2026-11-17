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
 * @file            isWallAtPassageOpening.cc
 *
 * @brief           Implements isWallAtPassageOpening(), declared in
 *                  semantics_manager/private_functions/private_functions.h.
 */

#include "private_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/common/point_tests.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus isWallAtPassageOpening(
    geometric::Plane                      *p_roomWall_in,
    const std::vector<geometric::Plane *> &supportingWalls_in,
    const Eigen::Vector4d                 &passageEquation_world_in,
    const Eigen::Vector3d                 &passageCentroid_world_m_in,
    double                                 passageWidth_m_in,
    double                                 openingFrameReach_m_in,
    bool                                  &isAtPassageOpening_out)
{
    constexpr double maximumSupportingPlaneDistance_m = 1.00;
    constexpr double minimumNormalAlignment           = 0.80;

    isAtPassageOpening_out = false;
    bool roomWallIsBad{};
    if (!(p_roomWall_in == nullptr) &&
        p_roomWall_in->isBad(roomWallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_roomWall_in == nullptr || roomWallIsBad)
    {
        isAtPassageOpening_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (std::find(supportingWalls_in.begin(),
                  supportingWalls_in.end(),
                  p_roomWall_in) != supportingWalls_in.end())
    {
        isAtPassageOpening_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    geometric::Plane::GeometrySnapshot roomWallGeometry{};
    if (p_roomWall_in->getGeometrySnapshot(roomWallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d roomWallEquation = roomWallGeometry.planeEquation_world;

    const double roomWallNormalNorm = roomWallEquation.head<3>().norm();

    if (!roomWallEquation.allFinite() || roomWallNormalNorm < 1e-8)
    {
        isAtPassageOpening_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    roomWallEquation /= roomWallNormalNorm;

    const double normalAlignment = std::abs(
        roomWallEquation.head<3>().dot(passageEquation_world_in.head<3>()));

    const double passagePlaneDistance_m =
        std::abs(roomWallEquation.head<3>().dot(passageCentroid_world_m_in) +
                 roomWallEquation(3));

    if (normalAlignment < minimumNormalAlignment ||
        passagePlaneDistance_m > maximumSupportingPlaneDistance_m)
    {
        isAtPassageOpening_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_roomWallCloud =
        roomWallGeometry.supportCloud;

    if (p_roomWallCloud == nullptr || p_roomWallCloud->empty())
    {
        isAtPassageOpening_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    double nearestOpeningEdgeDistance_m =
        std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &wallPoint : p_roomWallCloud->points)
    {
        if (!pcl::isFinite(wallPoint))
        {
            continue;
        }

        const Eigen::Vector3d wallPoint_world_m(
            static_cast<double>(wallPoint.x),
            static_cast<double>(wallPoint.y),
            static_cast<double>(wallPoint.z));

        Eigen::Vector3d openingOffset_world_m =
            wallPoint_world_m - passageCentroid_world_m_in;

        openingOffset_world_m -=
            openingOffset_world_m.dot(roomWallEquation.head<3>()) *
            roomWallEquation.head<3>();

        nearestOpeningEdgeDistance_m = std::min(nearestOpeningEdgeDistance_m,
                                                openingOffset_world_m.norm());
    }

    /* A non-finite or non-positive width gives no reach. */
    isAtPassageOpening_out =
        passageWidth_m_in > 0.0 &&
        nearestOpeningEdgeDistance_m <= openingFrameReach_m_in;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
