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

double RoomTracker::computeConfidence(double inlier_ratio,
                                      double normalised_condition_number,
                                      double angular_residual_rad,
                                      double sigma_theta_rad)
{
    if (!std::isfinite(inlier_ratio) ||
        !std::isfinite(normalised_condition_number) ||
        !std::isfinite(angular_residual_rad))
    {
        return 0.0;
    }
    if (inlier_ratio < 0.0)
    {
        inlier_ratio = 0.0;
    }
    else if (inlier_ratio > 1.0)
    {
        inlier_ratio = 1.0;
    }
    if (normalised_condition_number < 0.0)
    {
        normalised_condition_number = 0.0;
    }
    else if (normalised_condition_number > 1.0)
    {
        normalised_condition_number = 1.0;
    }

    double residualDecay = 1.0;
    if (std::isfinite(sigma_theta_rad) && sigma_theta_rad > 1e-12)
    {
        residualDecay = std::exp(-angular_residual_rad / sigma_theta_rad);
    }
    else
    {
        residualDecay = std::abs(angular_residual_rad) < 1e-12 ? 1.0 : 0.0;
    }

    return inlier_ratio * (1.0 - normalised_condition_number) * residualDecay;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
