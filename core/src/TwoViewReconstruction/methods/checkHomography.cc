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

float TwoViewReconstruction::checkHomography(const Eigen::Matrix3f &H21,
                                             const Eigen::Matrix3f &H12,
                                             vector<bool> &vbMatchesInliers,
                                             float         sigma)
{
    const int N = matches12.size();

    const float h11 = H21(0, 0);
    const float h12 = H21(0, 1);
    const float h13 = H21(0, 2);
    const float h21 = H21(1, 0);
    const float h22 = H21(1, 1);
    const float h23 = H21(1, 2);
    const float h31 = H21(2, 0);
    const float h32 = H21(2, 1);
    const float h33 = H21(2, 2);

    const float h11inv = H12(0, 0);
    const float h12inv = H12(0, 1);
    const float h13inv = H12(0, 2);
    const float h21inv = H12(1, 0);
    const float h22inv = H12(1, 1);
    const float h23inv = H12(1, 2);
    const float h31inv = H12(2, 0);
    const float h32inv = H12(2, 1);
    const float h33inv = H12(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 5.991;

    const float invSigmaSquare = 1.0 / (sigma * sigma);

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = keys1[matches12[i].first];
        const cv::KeyPoint &kp2 = keys2[matches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1    = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1    = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

        const float squareDist1 =
            (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
        const float u1in2    = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
        const float v1in2    = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

        const float squareDist2 =
            (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += th - chiSquare2;

        if (bIn)
            vbMatchesInliers[i] = true;
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

} // namespace core
} // namespace vs_graphs
