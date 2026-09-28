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

void Sim3Solver::computeCentroid(Eigen::Matrix3f &P_in,
                                 Eigen::Matrix3f &Pr_inout,
                                 Eigen::Vector3f &C_out)
{
    C_out = P_in.rowwise().sum();
    C_out = C_out / P_in.cols();
    for (int columnIndex = 0; columnIndex < P_in.cols(); columnIndex++)
        Pr_inout.col(columnIndex) = P_in.col(columnIndex) - C_out;
}

} // namespace core
} // namespace vs_graphs
