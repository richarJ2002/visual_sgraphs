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

TwoViewReconstructionStatus TwoViewReconstruction::normalize(
    const vector<cv::KeyPoint> &keys_in,
    vector<cv::Point2f>        &normalizedPoints_inout,
    Eigen::Matrix3f            &T_out)
{
    float     meanX = 0;
    float     meanY = 0;
    const int N     = keys_in.size();

    normalizedPoints_inout.resize(N);

    for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        meanX += keys_in[keyPointIndex].pt.x;
        meanY += keys_in[keyPointIndex].pt.y;
    }

    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        normalizedPoints_inout[keyPointIndex].x =
            keys_in[keyPointIndex].pt.x - meanX;
        normalizedPoints_inout[keyPointIndex].y =
            keys_in[keyPointIndex].pt.y - meanY;

        meanDevX += fabs(normalizedPoints_inout[keyPointIndex].x);
        meanDevY += fabs(normalizedPoints_inout[keyPointIndex].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        normalizedPoints_inout[keyPointIndex].x =
            normalizedPoints_inout[keyPointIndex].x * sX;
        normalizedPoints_inout[keyPointIndex].y =
            normalizedPoints_inout[keyPointIndex].y * sY;
    }

    T_out.setZero();
    T_out(0, 0) = sX;
    T_out(1, 1) = sY;
    T_out(0, 2) = -meanX * sX;
    T_out(1, 2) = -meanY * sY;
    T_out(2, 2) = 1.f;

    return TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
