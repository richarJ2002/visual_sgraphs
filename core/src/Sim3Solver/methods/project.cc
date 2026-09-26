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

void Sim3Solver::project(
    const vector<Eigen::Vector3f>                   &vP3Dw,
    vector<Eigen::Vector2f>                         &vP2D,
    Eigen::Matrix4f                                  Tcw,
    camera_models::geometriccamera::GeometricCamera *pCamera)
{
    Eigen::Matrix3f Rcw = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3f tcw = Tcw.block<3, 1>(0, 3);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for (size_t i = 0, iend = vP3Dw.size(); i < iend; i++)
    {
        Eigen::Vector3f P3Dc = Rcw * vP3Dw[i] + tcw;
        Eigen::Vector2f pt2D = pCamera->project(P3Dc);
        vP2D.push_back(pt2D);
    }
}

} // namespace core
} // namespace vs_graphs
