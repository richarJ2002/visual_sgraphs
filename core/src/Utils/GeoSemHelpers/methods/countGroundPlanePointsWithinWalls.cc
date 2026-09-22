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

namespace vs_graphs
{
namespace core
{

size_t GeoSemHelpers::countGroundPlanePointsWithinWalls(
    std::vector<vs_graphs::core::geometric::Plane *> &roomWalls,
    vs_graphs::core::geometric::Plane                *groundPlane)
{
    // [TODO] - verify the correctness of this function
    // the point cloud of the ground plane
    const geometric::Plane::GeometrySnapshot groundGeometry =
        groundPlane->getGeometrySnapshot();
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr groundCloud =
        groundGeometry.supportCloud;

    // the number of points within the walls
    size_t count = 0;

    // store the wall equations
    std::vector<Eigen::Vector4d> wallEquations;
    for (const auto &wall : roomWalls)
        wallEquations.push_back(wall->getGlobalEquation().coeffs());

    // for each point in the ground plane, check if it is within the walls
    for (const auto &point : groundCloud->points)
    {
        bool isWithinWalls = true;
        for (const auto &wallEquation : wallEquations)
        {
            // convert the point to Eigen vector
            Eigen::Vector3d pointVec =
                Eigen::Vector3d(point.x, point.y, point.z);

            // substitute the point into the wall equation to get the signed
            // distance
            float signedDistance =
                wallEquation.head<3>().dot(pointVec) + wallEquation(3);

            // if the point is outside the wall, break the loop
            if (signedDistance < 0)
            {
                isWithinWalls = false;
                break;
            }
        }

        // if the point is within the walls, increment the count
        if (isWithinWalls)
            count++;
    }
    return count;
}

} // namespace core
} // namespace vs_graphs
