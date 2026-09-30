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
 * @file            getDeltaRotation.cc
 *
 * @brief           Implements Preintegrated::getDeltaRotation(), declared in
 *                  ImuTypes.h.
 */

#include "ImuTypes.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

PreintegratedStatus
    Preintegrated::getDeltaRotation(const Bias      &referenceBias_in,
                                    Eigen::Matrix3f &deltaRotation_out)
{
    std::unique_lock<std::mutex> lock(preintegrationMutex);
    Eigen::Vector3f              gyroBiasDelta;
    gyroBiasDelta << referenceBias_in.bwx - b.bwx, referenceBias_in.bwy - b.bwy,
        referenceBias_in.bwz - b.bwz;
    if (gyroBiasDelta.array().isNaN()[0])
        gyroBiasDelta = Eigen::Vector3f(0, 0, 0);
    if (JRg.array().isNaN()(0, 0))
        JRg.setZero();
    Eigen::Matrix3f rotation{};
    if (normalizeRotation(dR * Sophus::SO3f::exp(JRg * gyroBiasDelta).matrix(),
                          rotation) != ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: normalizeRotation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    deltaRotation_out = rotation;
    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
