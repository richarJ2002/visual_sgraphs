/*!
 * @file            newParameterLoader.cc
 *
 * @brief           Implements MapDrawer::newParameterLoader declared in
 *                  MapDrawer.h.
 */

#include "MapDrawer.h"

#include "Utils/Settings/objects/Settings.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapDrawerStatus
    MapDrawer::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    double settingsKeyFrameSize{};
    if (p_settings_inout->keyFrameSize(settingsKeyFrameSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: keyFrameSize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    keyFrameSize = settingsKeyFrameSize;
    double settingsKeyFrameLineWidth{};
    if (p_settings_inout->keyFrameLineWidth(settingsKeyFrameLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: keyFrameLineWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    keyFrameLineWidth = settingsKeyFrameLineWidth;
    double settingsGraphLineWidth{};
    if (p_settings_inout->graphLineWidth(settingsGraphLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: graphLineWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    graphLineWidth = settingsGraphLineWidth;
    double settingsPointSize{};
    if (p_settings_inout->pointSize(settingsPointSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: pointSize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    pointSize = settingsPointSize;
    double settingsCameraSize{};
    if (p_settings_inout->cameraSize(settingsCameraSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: cameraSize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    cameraSize = settingsCameraSize;
    double settingsCameraLineWidth{};
    if (p_settings_inout->cameraLineWidth(settingsCameraLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: cameraLineWidth returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    cameraLineWidth = settingsCameraLineWidth;

    return MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
