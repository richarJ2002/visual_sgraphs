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
 * @file            newParameterLoader.cc
 *
 * @brief           Implements Viewer::newParameterLoader(), declared in
 *                  Viewer.h.
 */

#include "ResetCause.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ViewerStatus
    Viewer::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    imageViewerScale = 1.f;

    double framesPerSecondValue{};
    if (p_settings_inout->getFramesPerSecond(framesPerSecondValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFramesPerSecond returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    float fps = static_cast<float>(framesPerSecondValue);
    if (fps < 1)
        fps = 30;
    framePeriod = 1e3 / fps;

    cv::Size imageSize{};
    if (p_settings_inout->newImSize(imageSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: newImSize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    imageHeight = imageSize.height;
    imageWidth  = imageSize.width;

    double settingsImageViewerScale{};
    if (p_settings_inout->imageViewerScale(settingsImageViewerScale) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: imageViewerScale returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    imageViewerScale = settingsImageViewerScale;
    double settingsViewPointX{};
    if (p_settings_inout->viewPointX(settingsViewPointX) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: viewPointX returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewpointX = settingsViewPointX;
    double settingsViewPointY{};
    if (p_settings_inout->viewPointY(settingsViewPointY) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: viewPointY returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewpointY = settingsViewPointY;
    double settingsViewPointZ{};
    if (p_settings_inout->viewPointZ(settingsViewPointZ) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: viewPointZ returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewpointZ = settingsViewPointZ;
    double settingsViewPointF{};
    if (p_settings_inout->viewPointF(settingsViewPointF) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: viewPointF returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    viewpointF = settingsViewPointF;

    return ViewerStatus::VIEWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
