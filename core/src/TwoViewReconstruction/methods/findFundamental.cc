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

#include "TwoViewReconstruction.h"

#include "GeometricTools.h"
#include "Utils/Converter/objects/Converter.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include <thread>

using namespace std;
namespace vs_graphs
{
namespace core
{

void TwoViewReconstruction::findFundamental(vector<bool>    &inliersFlags_inout,
                                            float           &score_inout,
                                            Eigen::Matrix3f &F21_out)
{
    // Number of putative matches
    const int N = inliersFlags_inout.size();

    // Normalize coordinates
    vector<cv::Point2f> normalizedPoints1, normalizedPoints2;
    Eigen::Matrix3f     T1, T2;
    normalize(keys1, normalizedPoints1, T1);
    normalize(keys2, normalizedPoints2, T2);
    Eigen::Matrix3f T2t = T2.transpose();

    // Best Results variables
    score_inout        = 0.0;
    inliersFlags_inout = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> sampledPoints1(8);
    vector<cv::Point2f> sampledPoints2(8);
    Eigen::Matrix3f     F21i;
    vector<bool>        currentInliersFlags(N, false);
    float               currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int iterationIndex = 0; iterationIndex < maxIterations;
         iterationIndex++)
    {
        // Select a minimum set
        for (int setPointIndex = 0; setPointIndex < 8; setPointIndex++)
        {
            int matchIndex = sets[iterationIndex][setPointIndex];

            sampledPoints1[setPointIndex] =
                normalizedPoints1[matches12[matchIndex].first];
            sampledPoints2[setPointIndex] =
                normalizedPoints2[matches12[matchIndex].second];
        }

        Eigen::Matrix3f Fn = computeF21(sampledPoints1, sampledPoints2);

        F21i = T2t * Fn * T1;

        currentScore = checkFundamental(F21i, currentInliersFlags, sigma);

        if (currentScore > score_inout)
        {
            F21_out            = F21i;
            inliersFlags_inout = currentInliersFlags;
            score_inout        = currentScore;
        }
    }
}

} // namespace core
} // namespace vs_graphs
