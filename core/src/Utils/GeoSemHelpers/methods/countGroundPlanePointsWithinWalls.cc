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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::countGroundPlanePointsWithinWalls(
    std::vector<vs_graphs::core::geometric::Plane *> &roomWalls_in,
    vs_graphs::core::geometric::Plane                *p_groundPlane_in,
    size_t                                           &groundPlanePoints_out)
{
    // [TODO] - verify the correctness of this function
    // the point cloud of the ground plane
    geometric::Plane::GeometrySnapshot groundGeometry{};
    if (p_groundPlane_in->getGeometrySnapshot(groundGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_groundCloud =
        groundGeometry.supportCloud;

    // the number of points within the walls
    size_t count = 0;

    // store the wall equations
    std::vector<Eigen::Vector4d> wallEquations;
    for (geometric::Plane *const &wall : roomWalls_in)
    {
        g2o::Plane3D wallGetGlobalEquation{};
        if (wall->getGlobalEquation(wallGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        wallEquations.push_back(wallGetGlobalEquation.coeffs());
    }

    // for each point in the ground plane, check if it is within the walls
    for (const pcl::PointXYZRGBA &point : p_groundCloud->points)
    {
        bool isWithinWalls = true;
        for (const Eigen::Vector4d &wallEquation : wallEquations)
        {
            // convert the point to Eigen vector
            Eigen::Vector3d pointVector =
                Eigen::Vector3d(point.x, point.y, point.z);

            // substitute the point into the wall equation to get the signed
            // distance
            float signedDistance = static_cast<float>(
                wallEquation.head<3>().dot(pointVector) + wallEquation(3));

            // if the point is outside the wall, break the loop
            if (signedDistance < 0)
            {
                isWithinWalls = false;
                break;
            }
        }

        // if the point is within the walls, increment the count
        if (isWithinWalls)
        {
            count++;
        }
    }
    groundPlanePoints_out = count;
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
