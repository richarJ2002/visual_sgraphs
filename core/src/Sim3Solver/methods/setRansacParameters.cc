/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "Sim3Solver.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace vs_graphs
{
namespace core
{

Sim3SolverStatus Sim3Solver::setRansacParameters(double probability_in,
                                                 int    minimumInliers_in,
                                                 int    maximumIterations_in)
{
    ransacProb          = probability_in;
    ransacMinInliers    = minimumInliers_in;
    ransacMaxIterations = maximumIterations_in;

    correspondenceCount = mapPoints1.size(); // number of correspondences

    inlierFlags.resize(correspondenceCount);

    // Adjust Parameters according to number of correspondences
    float epsilon = static_cast<float>(ransacMinInliers) / correspondenceCount;

    // Set RANSAC iterations according to probability, epsilon, and max
    // iterations
    int nIterations;

    if (ransacMinInliers == correspondenceCount)
        nIterations = 1;
    else
        nIterations = ceil(log(1 - ransacProb) / log(1 - std::pow(epsilon, 3)));

    ransacMaxIterations =
        std::max(1, std::min(nIterations, ransacMaxIterations));

    iterationCount = 0;

    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
