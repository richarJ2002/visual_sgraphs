/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            computeConfidence.cc
 *
 * @brief           Implements RoomTracker::computeConfidence(), declared in
 *                  Semantic/RoomTracker.h.
 */

#include "Semantic/RoomTracker.h"

#include <cmath>
#include <iostream>
#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomTrackerStatus
    RoomTracker::computeConfidence(double  inlierRatio_in,
                                   double  normalizedConditionNumber_in,
                                   double  angularResidual_rad_in,
                                   double  sigmaTheta_rad_in,
                                   double &confidence_out)
{
    if (!std::isfinite(inlierRatio_in) ||
        !std::isfinite(normalizedConditionNumber_in) ||
        !std::isfinite(angularResidual_rad_in))
    {
        confidence_out = 0.0;
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    if (inlierRatio_in < 0.0)
    {
        inlierRatio_in = 0.0;
    }
    else if (inlierRatio_in > 1.0)
    {
        inlierRatio_in = 1.0;
    }
    if (normalizedConditionNumber_in < 0.0)
    {
        normalizedConditionNumber_in = 0.0;
    }
    else if (normalizedConditionNumber_in > 1.0)
    {
        normalizedConditionNumber_in = 1.0;
    }

    double residualDecay = 1.0;
    if (std::isfinite(sigmaTheta_rad_in) && sigmaTheta_rad_in > 1e-12)
    {
        residualDecay = std::exp(-angularResidual_rad_in / sigmaTheta_rad_in);
    }
    else
    {
        residualDecay = std::abs(angularResidual_rad_in) < 1e-12 ? 1.0 : 0.0;
    }

    confidence_out =
        inlierRatio_in * (1.0 - normalizedConditionNumber_in) * residualDecay;
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
