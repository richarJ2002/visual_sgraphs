/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            readIMU.cc
 *
 * @brief           Implements Settings::readIMU(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>

#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::readIMU(cv::FileStorage &storage_inout)
{
    bool found;
    accelWalkNoise = readParameter<float>(storage_inout, "IMU.AccWalk", found);
    accelNoise     = readParameter<float>(storage_inout, "IMU.NoiseAcc", found);
    gyroWalkNoise  = readParameter<float>(storage_inout, "IMU.GyroWalk", found);
    gyroNoise = readParameter<float>(storage_inout, "IMU.NoiseGyro", found);
    imuErrorThreshold =
        readParameter<float>(storage_inout, "IMU.Threshold", found);
    imuSampleRate = readParameter<float>(storage_inout, "IMU.Frequency", found);

    cv::Mat cvTbc = readParameter<cv::Mat>(storage_inout, "IMU.T_b_c1", found);
    bodyToCamera  = converter::Converter::toSophus(cvTbc);

    readParameter<int>(storage_inout, "IMU.InsertKFsWhenLost", found, false);
    if (found)
        shouldInsertKeyFramesWhenLost =
            (bool)readParameter<int>(storage_inout,
                                     "IMU.InsertKFsWhenLost",
                                     found,
                                     false);
    else
        shouldInsertKeyFramesWhenLost = true;

    isFastInitEnabled =
        readParameter<int>(storage_inout, "IMU.FastInit", found, false) != 0;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
