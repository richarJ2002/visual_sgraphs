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

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

void Plane::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    for (auto &point : planeCloud->points)
    {
        Eigen::Vector3d pointVector(point.x, point.y, point.z);

        pointVector = transform_oldWorldToNewWorld_in.map(pointVector);

        point.x = pointVector.x();
        point.y = pointVector.y();
        point.z = pointVector.z();
    }

    globalEquation =
        transformPlaneEquation(globalEquation, transform_oldWorldToNewWorld_in);

    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();

    /* Recompute the finite bounds in the transformed frame. */
    updatePlaneBoundsWithoutLock();
    ++cloudGeneration;
    successfulRefitGeneration = cloudGeneration;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
