/*!
 * @file            getImuFrequency.cc
 *
 * @brief           Implements System::getImuFrequency(), declared in
 *                  system/objects/SystemClass.h.
 */

#include "execution/system/objects/SystemClass.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace system
{

SystemStatus System::getImuFrequency(double &imuFrequency_hz_out) const
{
    if (p_settings == nullptr)
    {
        return SystemStatus::SYSTEM_STATUS_SETTING_UNAVAILABLE;
    }
    double settingsImuFrequency_hz{};
    if (p_settings->imuFrequency(settingsImuFrequency_hz) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: imuFrequency returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    imuFrequency_hz_out = settingsImuFrequency_hz;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */
