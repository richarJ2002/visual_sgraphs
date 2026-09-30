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

/*!
 * @file            normalizeRotation.cc
 *
 * @brief           Implements normalizeRotation(), declared in ImuTypes.h.
 */

#include "ImuTypes.h"

namespace vs_graphs
{
namespace core
{
namespace IMU
{

ImuTypesStatus normalizeRotation(const Eigen::Matrix3f &rotationMatrix_in,
                                 Eigen::Matrix3f       &rotation_out)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotationMatrix_in,
                                          Eigen::ComputeFullU |
                                              Eigen::ComputeFullV);
    rotation_out = svd.matrixU() * svd.matrixV().transpose();
    return ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
