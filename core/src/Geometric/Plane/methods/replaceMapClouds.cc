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

void Plane::replaceMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);
    planeCloud->clear();
    lastSuccessfulRefitFinitePointCount = 0;
    ++cloudGeneration;

    if (!p_planeCloud_in || p_planeCloud_in->empty())
    {
        centroid.setZero();
        mbBad = true;
        octree->deleteTree();
        return;
    }

    planeCloud->assign(p_planeCloud_in->begin(), p_planeCloud_in->end());
    planeCloud->header              = p_planeCloud_in->header;
    planeCloud->is_dense            = p_planeCloud_in->is_dense;
    planeCloud->sensor_origin_      = p_planeCloud_in->sensor_origin_;
    planeCloud->sensor_orientation_ = p_planeCloud_in->sensor_orientation_;

    /* Update the octree */
    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
