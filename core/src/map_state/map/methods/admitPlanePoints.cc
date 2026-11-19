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
 * @file            admitPlanePoints.cc
 *
 * @brief           Implements Map::admitPlanePoints(), declared in
 *                  map/objects/MapClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/map/objects/MapClass.h"

#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::admitPlanePoints(
    const geometric::Plane             *p_plane_in,
    const geometric::Plane             *p_releasedPlane_in,
    const std::vector<Eigen::Vector3f> &pointPositions_world_m_in,
    std::vector<bool>                  &isPointAdmitted_out)
{
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const double voxelSize_m =
        static_cast<double>(p_params->semantics.pointDensity.voxelSize_m);
    const std::uint32_t maxPointsPerVoxel =
        p_params->semantics.pointDensity.maxPointsPerVoxel;

    isPointAdmitted_out.assign(pointPositions_world_m_in.size(), false);

    /* Leaf lock: nothing else is locked while it is held. */
    std::lock_guard<std::mutex> lock(pointDensityMutex);

    /* Both planes give back every voxel they hold before any point is
     * counted, so a plane never competes with its own earlier points. */
    for (const geometric::Plane *p_releasingPlane :
         {p_plane_in, p_releasedPlane_in})
    {
        const std::unordered_map<const geometric::Plane *,
                                 std::vector<std::int64_t>>::iterator
            planeKeysIt = voxelKeysByPlane.find(p_releasingPlane);
        if (planeKeysIt == voxelKeysByPlane.end())
        {
            continue;
        }
        for (const std::int64_t voxelKey : planeKeysIt->second)
        {
            const std::unordered_map<std::int64_t, std::uint32_t>::iterator
                voxelCountIt = pointCountByVoxel.find(voxelKey);
            if (voxelCountIt != pointCountByVoxel.end() &&
                --voxelCountIt->second == 0U)
            {
                pointCountByVoxel.erase(voxelCountIt);
            }
        }
        voxelKeysByPlane.erase(planeKeysIt);
    }

    std::vector<std::int64_t> admittedVoxelKeys;
    admittedVoxelKeys.reserve(pointPositions_world_m_in.size());
    for (std::size_t pointIndex = 0U;
         pointIndex < pointPositions_world_m_in.size();
         ++pointIndex)
    {
        const Eigen::Vector3f &pointPosition_world_m =
            pointPositions_world_m_in[pointIndex];
        if (!pointPosition_world_m.allFinite())
        {
            continue;
        }
        std::int64_t voxelKey{0};
        if (computePointVoxelKey(pointPosition_world_m,
                                 voxelSize_m,
                                 voxelKey) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computePointVoxelKey returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::uint32_t &voxelPointCount = pointCountByVoxel[voxelKey];
        if (voxelPointCount >= maxPointsPerVoxel)
        {
            continue;
        }
        ++voxelPointCount;
        admittedVoxelKeys.push_back(voxelKey);
        isPointAdmitted_out[pointIndex] = true;
    }

    if (p_plane_in != nullptr && !admittedVoxelKeys.empty())
    {
        voxelKeysByPlane[p_plane_in] = std::move(admittedVoxelKeys);
    }

    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
