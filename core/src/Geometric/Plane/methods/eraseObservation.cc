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

void Plane::eraseObservation(core::KeyFrame *p_keyFrame_in)
{
    /* Confirm the keyframe is valid */
    if (p_keyFrame_in == nullptr)
    {
        return;
    }

    {
        /* Lock observations and semantics for one consistent vote rebuild. */
        std::scoped_lock lock(featuresMutex, typeMutex);

        const auto observationIt = observations.find(p_keyFrame_in);

        /* Return when the keyframe has no observation */
        if (observationIt == observations.end())
        {
            return;
        }

        /* Remove the observation */
        observations.erase(observationIt);

        /* Decrement the observation count safely */
        if (observationCount > 0)
        {
            observationCount--;
        }

        if (p_refKeyFrame == p_keyFrame_in)
        {
            p_refKeyFrame = nullptr;

            for (const auto &[p_candidateKeyFrame, candidateObservation] :
                 observations)
            {
                (void)candidateObservation;

                if (p_candidateKeyFrame == nullptr)
                {
                    continue;
                }

                if (p_refKeyFrame == nullptr ||
                    p_candidateKeyFrame->id < p_refKeyFrame->id)
                {
                    p_refKeyFrame = p_candidateKeyFrame;
                }
            }
        }

        rebuildSemanticVotesWithoutLock();
    }
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
