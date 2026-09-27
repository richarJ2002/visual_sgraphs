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

bool Plane::isPointinPlaneCloud(const Eigen::Vector3d &point)
{
    /*!
     * A NaN/Inf point (e.g. from a near-degenerate plane/line intersection
     * upstream, more likely when pose estimates are noisy) reaching
     * octree->radiusSearch() below trips PCL's own internal assertion
     * ("Invalid (NaN, Inf) point coordinates given to nearestKSearch!") and
     * aborts the process. Mirrors the same guard already used at the other
     * PCL nearest-neighbour call site (Utils.cc's finite-cloud-overlap
     * check) -- a point that isn't finite cannot be "in" the cloud, so
     * false is the correct answer, not a crash.
     */
    if (!point.allFinite())
    {
        return false;
    }

    unique_lock<mutex> lock(mMutexFeatures);
    pcl::PointXYZRGBA  pointPCL;
    pointPCL.x = point(0);
    pointPCL.y = point(1);
    pointPCL.z = point(2);

    types::SystemParams *p_sysParams = types::SystemParams::getParams();
    std::vector<int>     pointIdxRadiusSearch;
    std::vector<float>   pointRadiusSquaredDistance;

    if (octree->radiusSearch(
            pointPCL,
            p_sysParams->refineMapPoints.octree.searchRadius,
            pointIdxRadiusSearch,
            pointRadiusSquaredDistance,
            p_sysParams->refineMapPoints.octree.minNeighbors) ==
        p_sysParams->refineMapPoints.octree.minNeighbors)
    {
        return true;
    }

    return false;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
