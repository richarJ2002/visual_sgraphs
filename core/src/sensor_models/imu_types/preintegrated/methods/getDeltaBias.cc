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
 * @file            getDeltaBias.cc
 *
 * @brief           Implements both Preintegrated::getDeltaBias() overloads.
 */

#include "sensor_models/imu_types/preintegrated/objects/PreintegratedClass.h"

#include <mutex>

#include "sensor_models/imu_types/bias/bias.h"

#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace preintegrated
{

PreintegratedStatus
    Preintegrated::getDeltaBias(Eigen::Matrix<float, 6, 1> &deltaBias_out)
{
    std::unique_lock<std::mutex> lock(preintegrationMutex);
    deltaBias_out = db;
    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

PreintegratedStatus
    Preintegrated::getDeltaBias(const bias::Bias      &referenceBias_in,
                                imu_types::bias::Bias &deltaBias_out)
{
    std::unique_lock<std::mutex> lock(preintegrationMutex);
    deltaBias_out = imu_types::bias::Bias(referenceBias_in.bax - b.bax,
                                          referenceBias_in.bay - b.bay,
                                          referenceBias_in.baz - b.baz,
                                          referenceBias_in.bwx - b.bwx,
                                          referenceBias_in.bwy - b.bwy,
                                          referenceBias_in.bwz - b.bwz);
    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

} /* namespace preintegrated */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */
