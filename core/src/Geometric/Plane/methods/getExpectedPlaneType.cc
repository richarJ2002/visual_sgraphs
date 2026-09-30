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
 * @file            getExpectedPlaneType.cc
 *
 * @brief           Implements Plane::getExpectedPlaneType(), declared in
 *                  Geometric/Plane.h.
 */

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <mutex>
#include <pcl/octree/octree_search.h>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus
    Plane::getExpectedPlaneType(Plane::PlaneVariant &expectedPlaneType_out)
{
    std::unique_lock<std::mutex> lock(typeMutex);

    // get the maximum vote
    double       maximumVotes = 0;
    PlaneVariant maximumType  = PlaneVariant::UNDEFINED;
    for (const std::pair<const Plane::PlaneVariant, double> &vote :
         semanticVotes)
    {
        if (vote.second > maximumVotes)
        {
            maximumVotes = vote.second;
            maximumType  = vote.first;
        }
    }
    expectedPlaneType_out = maximumType;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
