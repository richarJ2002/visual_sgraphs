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

Eigen::Matrix3f
    TwoViewReconstruction::computeH21(const vector<cv::Point2f> &points1_in,
                                      const vector<cv::Point2f> &points2_in)
{
    const int N = points1_in.size();

    Eigen::MatrixXf A(2 * N, 9);

    for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        const float u1 = points1_in[keyPointIndex].x;
        const float v1 = points1_in[keyPointIndex].y;
        const float u2 = points2_in[keyPointIndex].x;
        const float v2 = points2_in[keyPointIndex].y;

        A(2 * keyPointIndex, 0) = 0.0;
        A(2 * keyPointIndex, 1) = 0.0;
        A(2 * keyPointIndex, 2) = 0.0;
        A(2 * keyPointIndex, 3) = -u1;
        A(2 * keyPointIndex, 4) = -v1;
        A(2 * keyPointIndex, 5) = -1;
        A(2 * keyPointIndex, 6) = v2 * u1;
        A(2 * keyPointIndex, 7) = v2 * v1;
        A(2 * keyPointIndex, 8) = v2;

        A(2 * keyPointIndex + 1, 0) = u1;
        A(2 * keyPointIndex + 1, 1) = v1;
        A(2 * keyPointIndex + 1, 2) = 1;
        A(2 * keyPointIndex + 1, 3) = 0.0;
        A(2 * keyPointIndex + 1, 4) = 0.0;
        A(2 * keyPointIndex + 1, 5) = 0.0;
        A(2 * keyPointIndex + 1, 6) = -u2 * u1;
        A(2 * keyPointIndex + 1, 7) = -u2 * v1;
        A(2 * keyPointIndex + 1, 8) = -u2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> H(svd.matrixV().col(8).data());

    return H;
}

} // namespace core
} // namespace vs_graphs
