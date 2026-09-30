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
 * @file            fromVector.cc
 *
 * @brief           Implements inverseRightJacobianSO3() (fromVector), declared
 *                  in ImuTypes.h.
 */

#include "ImuTypes.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

ImuTypesStatus
    inverseRightJacobianSO3(const Eigen::Vector3f &angleAxisVector_in,
                            Eigen::Matrix3f       &inverseRightJacobian_out)
{
    Eigen::Matrix3f inverseRightJacobian{};
    if (inverseRightJacobianSO3(angleAxisVector_in(0),
                                angleAxisVector_in(1),
                                angleAxisVector_in(2),
                                inverseRightJacobian) !=
        ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: inverseRightJacobianSO3 returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    inverseRightJacobian_out = inverseRightJacobian;
    return ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
