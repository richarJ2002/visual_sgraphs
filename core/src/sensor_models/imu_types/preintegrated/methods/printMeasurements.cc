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
 * @file            printMeasurements.cc
 *
 * @brief           Implements Preintegrated::printMeasurements().
 */

#include "sensor_models/imu_types/preintegrated/objects/PreintegratedClass.h"

#include <iostream>

#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace preintegrated
{

[[nodiscard]] PreintegratedStatus Preintegrated::printMeasurements() const
{
    std::cout << "\nIMU measures: \n";
    for (long unsigned int measurementIndex = 0;
         measurementIndex < measurements.size();
         measurementIndex++)
        std::cout << "- Measurement " << measurements[measurementIndex].t
                  << std::endl;
    std::cout << "Finished printing IMU measures ...\n";

    return PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS;
}

} /* namespace preintegrated */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */
