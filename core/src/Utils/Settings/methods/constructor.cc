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
 * @file            constructor.cc
 *
 * @brief           Implements Settings::Settings(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <string>

#include <opencv2/core/persistence.hpp>

#include "System.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

Settings::Settings(const std::string &configurationFilePath_in,
                   const int         &sensor_in) :
    isUndistortionNeeded(false),
    isRectificationNeeded(false),
    isFirstResizeNeeded(false),
    isSecondResizeNeeded(false)
{
    sensor = sensor_in;

    // Open settings file
    cv::FileStorage storage_in(configurationFilePath_in, cv::FileStorage::READ);
    if (!storage_in.isOpened())
    {
        VSLAM_LOG_ERROR("\n[Settings] Could not open the configuration file at "
                        "'%s'! Aborting...\n",
                        configurationFilePath_in.c_str());
        exit(-1);
    }
    else
        VSLAM_LOG_INFO("\n[Settings] Loading configurations from '%s'...\n",
                       configurationFilePath_in.c_str());

    // Read Camera#1 (monocular, stereo or RGB-D)
    if (readCamera1(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readCamera1 cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] Camera#1 settings loaded!\n");

    // Read Camera#2 (stereo)
    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
    {
        if (readCamera2(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // readCamera2 cannot fail; continue as before.
        }
        VSLAM_LOG_INFO("[Settings] Camera#2 settings loaded!\n");
    }

    // Read image info
    if (readImageInfo(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readImageInfo cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] Camera info loaded!\n");

    // Read IMU params
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        if (readIMU(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // readIMU cannot fail; continue as before.
        }
        VSLAM_LOG_INFO("[Settings] IMU calibration settings loaded!\n");
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        if (readRGBD(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // readRGBD cannot fail; continue as before.
        }
        VSLAM_LOG_INFO("[Settings] RGB-D settings loaded!\n");
    }

    // Read ORB parameters
    if (readORB(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readORB cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] ORB settings loaded!\n");

    // Read Viewer parameters
    if (readViewer(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readViewer cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] Viewer settings loaded!\n");

    // Read Atlas parameters
    if (readLoadAndSave(storage_in) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readLoadAndSave cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] ATLAS settings loaded!\n");

    // Read other parameters
    if (readOtherParameters(storage_in) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // readOtherParameters cannot fail; continue as before.
    }
    VSLAM_LOG_INFO("[Settings] Misc. parameters loaded!\n");

    if (isRectificationNeeded)
    {
        if (precomputeRectificationMaps() !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // precomputeRectificationMaps cannot fail; continue as before.
        }
        VSLAM_LOG_INFO("[Settings] Computed rectification maps!\n");
    }
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
