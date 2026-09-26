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

Eigen::Matrix4f Sim3Solver::iterate(int           nIterations,
                                    bool         &bNoMore,
                                    vector<bool> &vbInliers,
                                    int          &nInliers)
{
    bNoMore   = false;
    vbInliers = vector<bool>(mN1, false);
    nInliers  = 0;

    if (N < ransacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;
    while (iterationCount < ransacMaxIterations &&
           nCurrentIterations < nIterations)
    {
        nCurrentIterations++;
        iterationCount++;

        vAvailableIndices = allIndices;

        // Get min set of points
        for (short i = 0; i < 3; ++i)
        {
            int randi =
                DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = points3Dc1[idx];
            P3Dc2i.col(i) = points3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        computeSim3(P3Dc1i, P3Dc2i);

        checkInliers();

        if (inlierCount >= bestInlierCount)
        {
            bestInlierFlags  = inlierFlags;
            bestInlierCount  = inlierCount;
            mBestT12         = mT12i;
            mBestRotation    = mR12i;
            mBestTranslation = mt12i;
            mBestScale       = ms12i;

            if (inlierCount > ransacMinInliers)
            {
                nInliers = inlierCount;
                for (int i = 0; i < N; i++)
                    if (inlierFlags[i])
                        vbInliers[indices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if (iterationCount >= ransacMaxIterations)
        bNoMore = true;

    return Eigen::Matrix4f::Identity();
}

} // namespace core
} // namespace vs_graphs
