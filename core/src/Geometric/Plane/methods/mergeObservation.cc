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

void Plane::mergeObservation(core::KeyFrame    *p_keyFrame_in,
                             const Observation &observation_in)
{
    if (p_keyFrame_in == nullptr || p_keyFrame_in->isBad())
    {
        return;
    }

    std::scoped_lock lock(mMutexFeatures, mMutexType);
    auto evidenceFromObservation = [](const Observation &observation)
    {
        std::map<PlaneVariant, double> evidence = observation.semanticEvidence;
        if (evidence.empty() &&
            observation.semanticType != PlaneVariant::UNDEFINED &&
            std::isfinite(observation.confidence))
        {
            evidence[observation.semanticType] += observation.confidence;
        }
        return evidence;
    };

    const auto existingIterator = observations.find(p_keyFrame_in);
    if (existingIterator == observations.end())
    {
        Observation mergedObservation = observation_in;
        mergedObservation.semanticEvidence =
            evidenceFromObservation(observation_in);
        observations.emplace(p_keyFrame_in, std::move(mergedObservation));
        ++observationCount;
        if (p_refKeyFrame == nullptr)
        {
            p_refKeyFrame = p_keyFrame_in;
        }
    }
    else
    {
        Observation &retainedObservation = existingIterator->second;
        const double retainedConfidence  = retainedObservation.confidence;
        retainedObservation.pointPlaneConstraintMatrix +=
            observation_in.pointPlaneConstraintMatrix;

        std::map<PlaneVariant, double> combinedEvidence =
            evidenceFromObservation(retainedObservation);
        for (const auto &[semanticType, weight] :
             evidenceFromObservation(observation_in))
        {
            combinedEvidence[semanticType] += weight;
        }
        retainedObservation.semanticEvidence = std::move(combinedEvidence);
        retainedObservation.confidence += observation_in.confidence;

        if (observation_in.confidence > retainedConfidence)
        {
            retainedObservation.localPlane   = observation_in.localPlane;
            retainedObservation.semanticType = observation_in.semanticType;
        }
    }

    rebuildSemanticVotesWithoutLock();
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
