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

/*!****************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#include "MLPnPsolver.h"

#include <Eigen/Sparse>

namespace vs_graphs
{
namespace core
{

void MLPnPsolver::setRansacParameters(double probability_in,
                                      int    minimumInliers_in,
                                      int    maximumIterations_in,
                                      int    minimumSet_in,
                                      float  epsilon_in,
                                      float  threshold2_in)
{
    ransacProb          = probability_in;
    ransacMinInliers    = minimumInliers_in;
    ransacMaxIterations = maximumIterations_in;
    ransacEpsilon       = epsilon_in;
    ransacMinSet        = minimumSet_in;

    correspondenceCount = points2D.size(); // number of correspondences

    inlierFlags.resize(correspondenceCount);

    // Adjust Parameters according to number of correspondences
    int minimumInlierCount = correspondenceCount * ransacEpsilon;
    if (minimumInlierCount < ransacMinInliers)
        minimumInlierCount = ransacMinInliers;
    if (minimumInlierCount < minimumSet_in)
        minimumInlierCount = minimumSet_in;
    ransacMinInliers = minimumInlierCount;

    if (ransacEpsilon < (float)ransacMinInliers / correspondenceCount)
        ransacEpsilon = (float)ransacMinInliers / correspondenceCount;

    // Set RANSAC iterations according to probability, epsilon, and max
    // iterations
    int nIterations;

    if (ransacMinInliers == correspondenceCount)
        nIterations = 1;
    else
        nIterations =
            ceil(log(1 - ransacProb) / log(1 - pow(ransacEpsilon, 3)));

    ransacMaxIterations = max(1, min(nIterations, ransacMaxIterations));

    maxError.resize(sigmaSquared.size());
    for (size_t sigmaSquaredIndex = 0; sigmaSquaredIndex < sigmaSquared.size();
         sigmaSquaredIndex++)
        maxError[sigmaSquaredIndex] =
            sigmaSquared[sigmaSquaredIndex] * threshold2_in;
}

} // namespace core
} // namespace vs_graphs
