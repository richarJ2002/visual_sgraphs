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
    Preintegrated::mergePrevious(Preintegrated *p_previousPreintegrated_in)
{
    if (p_previousPreintegrated_in == this)
        return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;

    std::unique_lock<std::mutex> currentLock(preintegrationMutex);
    std::unique_lock<std::mutex> previousLock(
        p_previousPreintegrated_in->preintegrationMutex);
    Bias mergedBias;
    mergedBias.bwx = bu.bwx;
    mergedBias.bwy = bu.bwy;
    mergedBias.bwz = bu.bwz;
    mergedBias.bax = bu.bax;
    mergedBias.bay = bu.bay;
    mergedBias.baz = bu.baz;

    const std::vector<Integrable> previousMeasurements =
        p_previousPreintegrated_in->measurements;
    const std::vector<Integrable> currentMeasurements = measurements;

    if (initialize(mergedBias) !=
        PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: initialize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (size_t measurementIndex = 0;
         measurementIndex < previousMeasurements.size();
         measurementIndex++)
    {
        if (integrateNewMeasurement(previousMeasurements[measurementIndex].a,
                                    previousMeasurements[measurementIndex].w,
                                    previousMeasurements[measurementIndex].t) !=
            PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: integrateNewMeasurement returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    for (size_t measurementIndex = 0;
         measurementIndex < currentMeasurements.size();
         measurementIndex++)
    {
        if (integrateNewMeasurement(currentMeasurements[measurementIndex].a,
                                    currentMeasurements[measurementIndex].w,
                                    currentMeasurements[measurementIndex].t) !=
            PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: integrateNewMeasurement returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
