/*!
 * @file         newParameterLoader.cc
 *
 * @brief        Implements MapDrawer::newParameterLoader declared in
 *               MapDrawer.h.
 */

#include "MapDrawer.h"

#include "Utils/Settings/objects/Settings.h"

namespace vs_graphs
{
namespace core
{

void MapDrawer::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    double settingsKeyFrameSize{};
    if (p_settings_inout->keyFrameSize(settingsKeyFrameSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // keyFrameSize cannot fail; continue as before.
    }
    keyFrameSize = settingsKeyFrameSize;
    double settingsKeyFrameLineWidth{};
    if (p_settings_inout->keyFrameLineWidth(settingsKeyFrameLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // keyFrameLineWidth cannot fail; continue as before.
    }
    keyFrameLineWidth = settingsKeyFrameLineWidth;
    double settingsGraphLineWidth{};
    if (p_settings_inout->graphLineWidth(settingsGraphLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // graphLineWidth cannot fail; continue as before.
    }
    graphLineWidth = settingsGraphLineWidth;
    double settingsPointSize{};
    if (p_settings_inout->pointSize(settingsPointSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // pointSize cannot fail; continue as before.
    }
    pointSize = settingsPointSize;
    double settingsCameraSize{};
    if (p_settings_inout->cameraSize(settingsCameraSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // cameraSize cannot fail; continue as before.
    }
    cameraSize = settingsCameraSize;
    double settingsCameraLineWidth{};
    if (p_settings_inout->cameraLineWidth(settingsCameraLineWidth) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // cameraLineWidth cannot fail; continue as before.
    }
    cameraLineWidth = settingsCameraLineWidth;
}

} // namespace core
} // namespace vs_graphs
