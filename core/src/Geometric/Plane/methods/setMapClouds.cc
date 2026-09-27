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

void Plane::setMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in)
{
    if (!p_planeCloud_in || p_planeCloud_in->empty())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexFeatures);

    /* Add the new points to the plane cloud */
    for (const auto &point : p_planeCloud_in->points)
    {
        planeCloud->points.push_back(point);
    }

    /* Direct writes to the points container do not update PCL's organization
     * metadata. Mapped planes accumulate observations over time, so retaining
     * the first observation's width eventually makes every later copy or
     * transform report an invalid width/size combination. */
    planeCloud->width  = planeCloud->size();
    planeCloud->height = 1;

    /* Update the octree */
    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();
    ++cloudGeneration;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
