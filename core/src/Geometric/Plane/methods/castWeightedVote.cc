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

PlaneStatus Plane::castWeightedVote(Plane::PlaneVariant semanticType_in,
                                    double              voteWeight_in)
{
    unique_lock<mutex> lock(typeMutex);

    if (semanticType_in == PlaneVariant::UNDEFINED)
        return PlaneStatus::PLANE_STATUS_SUCCESS;

    // check if semantic type is already in the semanticVotes map
    if (semanticVotes.find(semanticType_in) == semanticVotes.end())
        semanticVotes[semanticType_in] = voteWeight_in;
    else
        semanticVotes[semanticType_in] += voteWeight_in;

    // update based on new vote rankings
    // find the semantic type with the maximum votes
    double       maximumVotes = 0;
    PlaneVariant maximumType  = PlaneVariant::UNDEFINED;
    for (const auto &vote : semanticVotes)
    {
        if (vote.second > maximumVotes)
        {
            maximumVotes = vote.second;
            maximumType  = vote.first;
        }
    }

    // set the plane type if votes above a certain threshold
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    if (maximumVotes >= p_params->semSeg.minVotes)
        planeType = maximumType;
    else
        planeType = PlaneVariant::UNDEFINED;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
