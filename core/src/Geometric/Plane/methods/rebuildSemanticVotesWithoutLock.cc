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

PlaneStatus Plane::rebuildSemanticVotesWithoutLock(void)
{
    semanticVotes.clear();
    for (const auto &[p_keyFrame, observation] : observations)
    {
        static_cast<void>(p_keyFrame);
        if (!observation.semanticEvidence.empty())
        {
            for (const auto &[semanticType, weight] :
                 observation.semanticEvidence)
            {
                if (semanticType != PlaneVariant::UNDEFINED &&
                    std::isfinite(weight))
                {
                    semanticVotes[semanticType] += weight;
                }
            }
        }
        else if (observation.semanticType != PlaneVariant::UNDEFINED &&
                 std::isfinite(observation.confidence))
        {
            semanticVotes[observation.semanticType] += observation.confidence;
        }
    }

    double       maximumVotes = 0.0;
    PlaneVariant maximumType  = PlaneVariant::UNDEFINED;
    for (const auto &[semanticType, votes] : semanticVotes)
    {
        if (votes > maximumVotes)
        {
            maximumVotes = votes;
            maximumType  = semanticType;
        }
    }
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    planeType = maximumVotes >= p_params->semSeg.minVotes
                    ? maximumType
                    : PlaneVariant::UNDEFINED;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
