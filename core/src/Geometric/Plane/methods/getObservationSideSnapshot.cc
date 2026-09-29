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
#include "KeyFrame.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::getObservationSideSnapshot(
    const Eigen::Vector4d          &normalizedEquation_World_in,
    Plane::ObservationSideSnapshot &observationSideSnapshot_out) const
{
    ObservationSideSnapshot snapshot;
    if (!normalizedEquation_World_in.allFinite() ||
        std::abs(normalizedEquation_World_in.head<3>().norm() - 1.0) > 1e-3)
    {
        observationSideSnapshot_out = snapshot;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    constexpr double    minimumReliableSideDistance_m = 0.10;
    constexpr double    minimumSignConsensusRatio     = 0.75;
    std::vector<double> signedDistances_m;

    std::map<core::KeyFrame *, Plane::Observation> getObservations2{};
    if (getObservations(getObservations2) != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservations returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (const auto &[p_keyFrame, observation] : getObservations2)
    {
        static_cast<void>(observation);
        if (p_keyFrame == nullptr || p_keyFrame->isBad())
        {
            continue;
        }

        const Eigen::Vector3d cameraCenter_World_m =
            p_keyFrame->getCameraCenter().cast<double>();
        const double signedDistance_m =
            normalizedEquation_World_in.head<3>().dot(cameraCenter_World_m) +
            normalizedEquation_World_in(3);
        if (cameraCenter_World_m.allFinite() &&
            std::isfinite(signedDistance_m) &&
            std::abs(signedDistance_m) >= minimumReliableSideDistance_m)
        {
            signedDistances_m.push_back(signedDistance_m);
        }
    }

    snapshot.evidenceCount = signedDistances_m.size();
    if (signedDistances_m.empty())
    {
        observationSideSnapshot_out = snapshot;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    const std::size_t positiveCount  = static_cast<std::size_t>(std::count_if(
        signedDistances_m.begin(),
        signedDistances_m.end(),
        [](const double distance_m) { return distance_m > 0.0; }));
    const std::size_t negativeCount  = signedDistances_m.size() - positiveCount;
    const std::size_t consensusCount = std::max(positiveCount, negativeCount);
    snapshot.consensusRatio          = static_cast<double>(consensusCount) /
                              static_cast<double>(signedDistances_m.size());
    if (snapshot.consensusRatio < minimumSignConsensusRatio)
    {
        snapshot.face               = ObservationSideSnapshot::Face::AMBIGUOUS;
        observationSideSnapshot_out = snapshot;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    const bool positiveConsensus = positiveCount >= negativeCount;
    snapshot.face = positiveConsensus ? ObservationSideSnapshot::Face::POSITIVE
                                      : ObservationSideSnapshot::Face::NEGATIVE;
    signedDistances_m.erase(
        std::remove_if(signedDistances_m.begin(),
                       signedDistances_m.end(),
                       [positiveConsensus](const double distance_m)
                       { return (distance_m > 0.0) != positiveConsensus; }),
        signedDistances_m.end());
    const std::size_t medianIndex = signedDistances_m.size() / 2U;
    std::nth_element(signedDistances_m.begin(),
                     signedDistances_m.begin() + medianIndex,
                     signedDistances_m.end());
    snapshot.medianSignedDistance_m = signedDistances_m[medianIndex];
    observationSideSnapshot_out     = snapshot;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
