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
    TwoViewReconstruction::computeF21(const vector<cv::Point2f> &vP1,
                                      const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    Eigen::MatrixXf A(N, 9);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A(i, 0) = u2 * u1;
        A(i, 1) = u2 * v1;
        A(i, 2) = u2;
        A(i, 3) = v2 * u1;
        A(i, 4) = v2 * v1;
        A(i, 5) = v2;
        A(i, 6) = u1;
        A(i, 7) = v1;
        A(i, 8) = 1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,
                                          Eigen::ComputeFullU |
                                              Eigen::ComputeFullV);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Fpre(
        svd.matrixV().col(8).data());

    Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre,
                                           Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

    Eigen::Vector3f w = svd2.singularValues();
    w(2)              = 0;

    return svd2.matrixU() * Eigen::DiagonalMatrix<float, 3>(w) *
           svd2.matrixV().transpose();
}

} // namespace core
} // namespace vs_graphs
