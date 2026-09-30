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
 * @file            replaceMapClouds.cc
 *
 * @brief           Implements Plane::replaceMapClouds(), declared in
 *                  Geometric/Plane.h.
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

PlaneStatus Plane::replaceMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_replacementCloud_in)
{
    std::scoped_lock lock(positionMutex, typeMutex, featuresMutex);
    planeCloud->clear();
    lastSuccessfulRefitFinitePointCount = 0;
    ++cloudGeneration;

    if (!p_replacementCloud_in || p_replacementCloud_in->empty())
    {
        centroid.setZero();
        isFlaggedBad = true;
        p_octree->deleteTree();
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    planeCloud->assign(p_replacementCloud_in->begin(),
                       p_replacementCloud_in->end());
    planeCloud->header         = p_replacementCloud_in->header;
    planeCloud->is_dense       = p_replacementCloud_in->is_dense;
    planeCloud->sensor_origin_ = p_replacementCloud_in->sensor_origin_;
    planeCloud->sensor_orientation_ =
        p_replacementCloud_in->sensor_orientation_;

    /* Update the octree */
    p_octree->deleteTree();
    p_octree->setInputCloud(planeCloud);
    p_octree->addPointsFromInputCloud();

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
