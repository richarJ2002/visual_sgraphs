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

bool MLPnPsolver::iterate(int              nIterations,
                          bool            &bNoMore,
                          vector<bool>    &vbInliers,
                          int             &nInliers,
                          Eigen::Matrix4f &Tout)
{
    Tout.setIdentity();
    bNoMore = false;
    vbInliers.clear();
    nInliers = 0;

    if (N < ransacMinInliers)
    {
        bNoMore = true;
        return false;
    }

    vector<size_t> vAvailableIndices;

    int nCurrentIterations = 0;
    while (iterationCount < ransacMaxIterations ||
           nCurrentIterations < nIterations)
    {
        nCurrentIterations++;
        iterationCount++;

        vAvailableIndices = allIndices;

        // Bearing vectors and 3D points used for this ransac iteration
        bearingVectors_t bearingVecs(ransacMinSet);
        points_t         p3DS(ransacMinSet);
        vector<int>      indexes(ransacMinSet);

        // Get min set of points
        for (short i = 0; i < ransacMinSet; ++i)
        {
            int randi =
                DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            bearingVecs[i] = bearingVectors[idx];
            p3DS[i]        = points3Dw[idx];
            indexes[i]     = i;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // By the moment, we are using MLPnP without covariance info
        cov3_mats_t covs(1);

        // Result
        transformation_t result;

        // Compute camera pose
        computePose(bearingVecs, p3DS, covs, indexes, result);

        // Save result
        mRi[0][0] = result(0, 0);
        mRi[0][1] = result(0, 1);
        mRi[0][2] = result(0, 2);

        mRi[1][0] = result(1, 0);
        mRi[1][1] = result(1, 1);
        mRi[1][2] = result(1, 2);

        mRi[2][0] = result(2, 0);
        mRi[2][1] = result(2, 1);
        mRi[2][2] = result(2, 2);

        mti[0] = result(0, 3);
        mti[1] = result(1, 3);
        mti[2] = result(2, 3);

        // Check inliers
        checkInliers();

        if (inlierCount >= ransacMinInliers)
        {
            // If it is the best solution so far, save it
            if (inlierCount > bestInlierCount)
            {
                bestInlierFlags = inlierFlags;
                bestInlierCount = inlierCount;

                cv::Mat Rcw(3, 3, CV_64F, mRi);
                cv::Mat tcw(3, 1, CV_64F, mti);
                Rcw.convertTo(Rcw, CV_32F);
                tcw.convertTo(tcw, CV_32F);
                mBestTcw.setIdentity();
                mBestTcw.block<3, 3>(0, 0) =
                    utils::converter::Converter::toMatrix3f(Rcw);
                mBestTcw.block<3, 1>(0, 3) =
                    utils::converter::Converter::toVector3f(tcw);

                Eigen::Matrix<double, 3, 3, Eigen::RowMajor> eigRcw(mRi[0]);
                Eigen::Vector3d                              eigtcw(mti);
            }

            if (refine())
            {
                nInliers  = refinedInlierCount;
                vbInliers = vector<bool>(mapPointMatches.size(), false);
                for (int i = 0; i < N; i++)
                {
                    if (refinedInlierFlags[i])
                        vbInliers[keypointIndices[i]] = true;
                }
                Tout = mRefinedTcw;
                return true;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
    {
        bNoMore = true;
        if (bestInlierCount >= ransacMinInliers)
        {
            nInliers  = bestInlierCount;
            vbInliers = vector<bool>(mapPointMatches.size(), false);
            for (int i = 0; i < N; i++)
            {
                if (bestInlierFlags[i])
                    vbInliers[keypointIndices[i]] = true;
            }
            Tout = mBestTcw;
            return true;
        }
    }

    return false;
}

} // namespace core
} // namespace vs_graphs
