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
 * @file            getMaxPointsPerVoxel.cc
 *
 * @brief           Implements Map::getMaxPointsPerVoxel(), declared in
 *                  map/objects/MapClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/map/objects/MapClass.h"
#include "mapping/geometric/Plane/Plane.h"

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::getMaxPointsPerVoxel(std::uint32_t &maxPointsPerVoxel_out,
                                    bool          &isCountConsistent_out)
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

    /*!
     * The clouds are copied one plane at a time while other threads move and
     * re-admit planes, so a count is kept only when no admission happened
     * while it was taken (densityRevision unchanged). Every cloud change is
     * admitted inside the plane's own lock, so such a count sees one
     * consistent set of clouds.
     */
    constexpr int MAX_COUNT_ATTEMPTS = 10;
    isCountConsistent_out            = false;
    for (int countAttempt = 0;
         countAttempt < MAX_COUNT_ATTEMPTS && !isCountConsistent_out;
         ++countAttempt)
    {
        std::uint64_t revisionBefore{0U};
        {
            std::lock_guard<std::mutex> lock(pointDensityMutex);
            revisionBefore = densityRevision;
        }

        std::vector<geometric::Plane *> mapPlanes{};
        if (getAllPlanes(mapPlanes) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllPlanes returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }

        /* Counted from the clouds themselves, not from pointCountByVoxel, so
         * the result checks the table instead of repeating it. */
        std::unordered_map<std::int64_t, std::uint32_t> pointCountByVoxelKey{};
        maxPointsPerVoxel_out = 0U;
        for (geometric::Plane *p_mapPlane : mapPlanes)
        {
            if (p_mapPlane == nullptr)
            {
                continue;
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud{};
            if (p_mapPlane->getMapClouds(p_planeCloud) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getMapClouds returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_planeCloud == nullptr)
            {
                continue;
            }
            for (const pcl::PointXYZRGBA &planePoint : p_planeCloud->points)
            {
                const Eigen::Vector3f pointPosition_world_m(planePoint.x,
                                                            planePoint.y,
                                                            planePoint.z);
                if (!pointPosition_world_m.allFinite())
                {
                    continue;
                }
                std::int64_t voxelKey{0};
                if (computePointVoxelKey(pointPosition_world_m,
                                         voxelSize_m,
                                         voxelKey) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: computePointVoxelKey returned a failure "
                        "status although it cannot fail; continuing as "
                        "before.",
                        __func__);
                }
                maxPointsPerVoxel_out =
                    std::max(maxPointsPerVoxel_out,
                             ++pointCountByVoxelKey[voxelKey]);
            }
        }

        std::lock_guard<std::mutex> lock(pointDensityMutex);
        isCountConsistent_out = densityRevision == revisionBefore;
    }

    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
