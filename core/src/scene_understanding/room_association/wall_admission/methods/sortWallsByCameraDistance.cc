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
 * @file            sortWallsByCameraDistance.cc
 *
 * @brief           Implements SemanticsManager::sortWallsByCameraDistance(),
 *                  declared in SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <tuple>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::sortWallsByCameraDistance(
    std::vector<geometric::Plane *> &walls_inout)
{
    /* Ground axes exactly as admitWallToRoom() builds them, so the distance
     * is measured on the same finite segments admission reasons about. */
    Eigen::Vector3d   groundNormal_world = Eigen::Vector3d::Zero();
    geometric::Plane *p_groundPlane      = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation_world =
            groundPlaneGetGlobalEquation.coeffs();
        const double groundNormalNorm = groundEquation_world.head<3>().norm();
        if (groundEquation_world.allFinite() && groundNormalNorm > 1e-8)
        {
            groundNormal_world =
                groundEquation_world.head<3>() / groundNormalNorm;
        }
    }
    const bool canMeasureDistance =
        hasCameraCenter && !groundNormal_world.isZero();
    const Eigen::Vector3d groundAxisU_world =
        groundNormal_world.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_world =
        groundNormal_world.cross(groundAxisU_world).normalized();
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_sysParams->roomSeg.boundaryTopology;

    /* Sort key: (camera distance m, first observation time s, plane id). */
    std::vector<std::tuple<double, double, int, geometric::Plane *>> wallKeys{};
    wallKeys.reserve(walls_inout.size());
    for (geometric::Plane *p_wall : walls_inout)
    {
        double cameraDistance_m       = std::numeric_limits<double>::infinity();
        double firstObservationTime_s = std::numeric_limits<double>::infinity();
        int    wallId                 = std::numeric_limits<int>::max();
        if (p_wall != nullptr)
        {
            if (p_wall->getId(wallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            std::map<keyframe::KeyFrame *, geometric::Plane::Observation>
                wallObservations{};
            if (p_wall->getObservations(wallObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (const auto &[p_keyFrame, observation] : wallObservations)
            {
                if (p_keyFrame != nullptr)
                {
                    firstObservationTime_s =
                        std::min(firstObservationTime_s, p_keyFrame->timeStamp);
                }
            }

            FiniteWallSegment2d wallSegment{};
            bool                isBuilt{};
            if (canMeasureDistance &&
                buildFiniteWallSegment2d(p_wall,
                                         groundNormal_world,
                                         groundAxisU_world,
                                         groundAxisV_world,
                                         topologyParameters.endpointTrimRatio,
                                         topologyParameters.minimumWallLength_m,
                                         wallSegment,
                                         isBuilt) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: buildFiniteWallSegment2d returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (canMeasureDistance && isBuilt)
            {
                const Eigen::Vector2d cameraGround_world_m(
                    currentCameraCenter_world_m.dot(groundAxisU_world),
                    currentCameraCenter_world_m.dot(groundAxisV_world));
                if (pointToSegmentDistance_m(cameraGround_world_m,
                                             wallSegment,
                                             cameraDistance_m) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: pointToSegmentDistance_m returned a failure "
                        "status although it cannot fail; continuing as "
                        "before.",
                        __func__);
                }
            }
        }
        wallKeys.emplace_back(cameraDistance_m,
                              firstObservationTime_s,
                              wallId,
                              p_wall);
    }

    /* Plane ids are unique, so the first three fields decide every pair;
     * stable_sort keeps any remaining tie (null planes) in input order. */
    std::stable_sort(
        wallKeys.begin(),
        wallKeys.end(),
        [](const std::tuple<double, double, int, geometric::Plane *> &first_in,
           const std::tuple<double, double, int, geometric::Plane *> &second_in)
        {
            return std::make_tuple(std::get<0>(first_in),
                                   std::get<1>(first_in),
                                   std::get<2>(first_in)) <
                   std::make_tuple(std::get<0>(second_in),
                                   std::get<1>(second_in),
                                   std::get<2>(second_in));
        });

    for (std::size_t wallIndex = 0U; wallIndex < wallKeys.size(); wallIndex++)
    {
        walls_inout[wallIndex] = std::get<3>(wallKeys[wallIndex]);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
