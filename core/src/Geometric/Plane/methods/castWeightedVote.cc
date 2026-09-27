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

void Plane::castWeightedVote(Plane::PlaneVariant semanticType,
                             double              voteWeight)
{
    unique_lock<mutex> lock(mMutexType);

    if (semanticType == PlaneVariant::UNDEFINED)
        return;

    // check if semantic type is already in the semanticVotes map
    if (semanticVotes.find(semanticType) == semanticVotes.end())
        semanticVotes[semanticType] = voteWeight;
    else
        semanticVotes[semanticType] += voteWeight;

    // update based on new vote rankings
    // find the semantic type with the maximum votes
    double       maxVotes = 0;
    PlaneVariant maxType  = PlaneVariant::UNDEFINED;
    for (const auto &vote : semanticVotes)
    {
        if (vote.second > maxVotes)
        {
            maxVotes = vote.second;
            maxType  = vote.first;
        }
    }

    // set the plane type if votes above a certain threshold
    if (maxVotes >= types::SystemParams::getParams()->semSeg.minVotes)
        planeType = maxType;
    else
        planeType = PlaneVariant::UNDEFINED;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
