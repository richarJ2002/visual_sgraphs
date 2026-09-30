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
 * @file            getDeltaPosition.cc
 *
 * @brief           Implements Preintegrated::getDeltaPosition(), declared in
 *                  ImuTypes.h.
 */

#include "ImuTypes.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace IMU
{

PreintegratedStatus
    Preintegrated::getDeltaPosition(const Bias      &referenceBias_in,
                                    Eigen::Vector3f &deltaPosition_out)
{
    std::unique_lock<std::mutex> lock(preintegrationMutex);
    Eigen::Vector3f              gyroBiasDelta, accelBiasDelta;
    gyroBiasDelta << referenceBias_in.bwx - b.bwx, referenceBias_in.bwy - b.bwy,
        referenceBias_in.bwz - b.bwz;
    accelBiasDelta << referenceBias_in.bax - b.bax,
        referenceBias_in.bay - b.bay, referenceBias_in.baz - b.baz;
    deltaPosition_out = dP + JPg * gyroBiasDelta + JPa * accelBiasDelta;
    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
