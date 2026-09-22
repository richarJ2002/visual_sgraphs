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

void TwoViewReconstruction::decomposeE(const Eigen::Matrix3f &E,
                                       Eigen::Matrix3f       &R1,
                                       Eigen::Matrix3f       &R2,
                                       Eigen::Vector3f       &t)
{

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,
                                          Eigen::ComputeFullU |
                                              Eigen::ComputeFullV);

    Eigen::Matrix3f U  = svd.matrixU();
    Eigen::Matrix3f Vt = svd.matrixV().transpose();

    t = U.col(2);
    t = t / t.norm();

    Eigen::Matrix3f W;
    W.setZero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    R1 = U * W * Vt;
    if (R1.determinant() < 0)
        R1 = -R1;

    R2 = U * W.transpose() * Vt;
    if (R2.determinant() < 0)
        R2 = -R2;
}

} // namespace core
} // namespace vs_graphs
