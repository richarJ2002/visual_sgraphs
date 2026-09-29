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

TwoViewReconstructionStatus TwoViewReconstruction::checkFundamental(
    const Eigen::Matrix3f &F21_in,
    vector<bool>          &matchesInliersFlags_inout,
    float                  sigma_in,
    float                 &score_out)
{
    const int N = matches12.size();

    const float f11 = F21_in(0, 0);
    const float f12 = F21_in(0, 1);
    const float f13 = F21_in(0, 2);
    const float f21 = F21_in(1, 0);
    const float f22 = F21_in(1, 1);
    const float f23 = F21_in(1, 2);
    const float f31 = F21_in(2, 0);
    const float f32 = F21_in(2, 1);
    const float f33 = F21_in(2, 2);

    matchesInliersFlags_inout.resize(N);

    float score = 0;

    const float threshold      = 3.841;
    const float thresholdScore = 5.991;

    const float invSigmaSquare = 1.0 / (sigma_in * sigma_in);

    for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        bool isInlier = true;

        const cv::KeyPoint &keyPoint1 = keys1[matches12[keyPointIndex].first];
        const cv::KeyPoint &keyPoint2 = keys2[matches12[keyPointIndex].second];

        const float u1 = keyPoint1.pt.x;
        const float v1 = keyPoint1.pt.y;
        const float u2 = keyPoint2.pt.x;
        const float v2 = keyPoint2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11 * u1 + f12 * v1 + f13;
        const float b2 = f21 * u1 + f22 * v1 + f23;
        const float c2 = f31 * u1 + f32 * v1 + f33;

        const float num2 = a2 * u2 + b2 * v2 + c2;

        const float squareDistance1 = num2 * num2 / (a2 * a2 + b2 * b2);

        const float chiSquare1 = squareDistance1 * invSigmaSquare;

        if (chiSquare1 > threshold)
            isInlier = false;
        else
            score += thresholdScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11 * u2 + f21 * v2 + f31;
        const float b1 = f12 * u2 + f22 * v2 + f32;
        const float c1 = f13 * u2 + f23 * v2 + f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;

        const float squareDistance2 = num1 * num1 / (a1 * a1 + b1 * b1);

        const float chiSquare2 = squareDistance2 * invSigmaSquare;

        if (chiSquare2 > threshold)
            isInlier = false;
        else
            score += thresholdScore - chiSquare2;

        if (isInlier)
            matchesInliersFlags_inout[keyPointIndex] = true;
        else
            matchesInliersFlags_inout[keyPointIndex] = false;
    }

    score_out = score;
    return TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
