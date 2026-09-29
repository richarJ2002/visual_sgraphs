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
 * @file            readViewer.cc
 *
 * @brief           Implements Settings::readViewer(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

SettingsStatus Settings::readViewer(cv::FileStorage &storage_inout)
{
    bool found;

    float parameter{};
    if (readParameter<float>(storage_inout,
                             "Viewer.KeyFrameSize",
                             found,
                             parameter) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerKeyFrameSize = parameter;
    float parameter2{};
    if (readParameter<float>(storage_inout,
                             "Viewer.KeyFrameLineWidth",
                             found,
                             parameter2) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerKeyFrameLineWidth = parameter2;
    float parameter3{};
    if (readParameter<float>(storage_inout,
                             "Viewer.GraphLineWidth",
                             found,
                             parameter3) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerGraphLineWidth = parameter3;
    float parameter4{};
    if (readParameter<float>(storage_inout,
                             "Viewer.PointSize",
                             found,
                             parameter4) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerPointSize = parameter4;
    float parameter5{};
    if (readParameter<float>(storage_inout,
                             "Viewer.CameraSize",
                             found,
                             parameter5) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerCameraSize = parameter5;
    float parameter6{};
    if (readParameter<float>(storage_inout,
                             "Viewer.CameraLineWidth",
                             found,
                             parameter6) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerCameraLineWidth = parameter6;
    float parameter7{};
    if (readParameter<float>(storage_inout,
                             "Viewer.ViewpointX",
                             found,
                             parameter7) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerViewPointX = parameter7;
    float parameter8{};
    if (readParameter<float>(storage_inout,
                             "Viewer.ViewpointY",
                             found,
                             parameter8) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerViewPointY = parameter8;
    float parameter9{};
    if (readParameter<float>(storage_inout,
                             "Viewer.ViewpointZ",
                             found,
                             parameter9) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerViewPointZ = parameter9;
    float parameter10{};
    if (readParameter<float>(storage_inout,
                             "Viewer.ViewpointF",
                             found,
                             parameter10) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerViewPointF = parameter10;
    float parameter11{};
    if (readParameter<float>(storage_inout,
                             "Viewer.imageViewScale",
                             found,
                             parameter11,
                             false) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewerImageScale = parameter11;

    if (!found)
        viewerImageScale = 1.0f;

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
