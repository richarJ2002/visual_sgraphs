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
 * @file            readRGBD.cc
 *
 * @brief           Implements Settings::readRGBD(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <utility>

#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

#include "Types/objects/SystemParams.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

SettingsStatus Settings::readRGBD(cv::FileStorage &storage_inout)
{
    bool found;

    float parameter{};
    if (readParameter<float>(storage_inout,
                             "RGBD.DepthMapFactor",
                             found,
                             parameter) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    depthMapScale = parameter;
    float parameter2{};
    if (readParameter<float>(storage_inout,
                             "Stereo.ThDepth",
                             found,
                             parameter2) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    depthThreshold = parameter2;
    float parameter3{};
    if (readParameter<float>(storage_inout, "Stereo.b", found, parameter3) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    stereoBaseline = parameter3;
    float calibration1Parameter{};
    if (p_calibration1->getParameter(0, calibration1Parameter) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    baselineFocal = stereoBaseline * calibration1Parameter;
    float parameter4{};
    if (readParameter<float>(storage_inout,
                             "RGBD.NearThresh",
                             found,
                             parameter4) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    nearThreshold = parameter4;
    float parameter5{};
    if (readParameter<float>(storage_inout,
                             "RGBD.FarThresh",
                             found,
                             parameter5) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    farThreshold = parameter5;

    // set distance threshold in the system params
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_params->pointcloud.distanceThresh =
        std::make_pair(nearThreshold, farThreshold);

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
