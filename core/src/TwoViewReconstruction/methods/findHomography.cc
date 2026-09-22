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

void TwoViewReconstruction::findHomography(vector<bool>    &vbMatchesInliers,
                                           float           &score,
                                           Eigen::Matrix3f &H21)
{
    // Number of putative matches
    const int N = matches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    Eigen::Matrix3f     T1, T2;
    normalize(keys1, vPn1, T1);
    normalize(keys2, vPn2, T2);
    Eigen::Matrix3f T2inv = T2.inverse();

    // Best Results variables
    score            = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    Eigen::Matrix3f     H21i, H12i;
    vector<bool>        vbCurrentInliers(N, false);
    float               currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < maxIterations; it++)
    {
        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int idx = sets[it][j];

            vPn1i[j] = vPn1[matches12[idx].first];
            vPn2i[j] = vPn2[matches12[idx].second];
        }

        Eigen::Matrix3f Hn = computeH21(vPn1i, vPn2i);
        H21i               = T2inv * Hn * T1;
        H12i               = H21i.inverse();

        currentScore = checkHomography(H21i, H12i, vbCurrentInliers, sigma);

        if (currentScore > score)
        {
            H21              = H21i;
            vbMatchesInliers = vbCurrentInliers;
            score            = currentScore;
        }
    }
}

} // namespace core
} // namespace vs_graphs
