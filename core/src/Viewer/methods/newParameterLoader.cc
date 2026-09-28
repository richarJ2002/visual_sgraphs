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

#include "ResetCause.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Viewer::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    imageViewerScale = 1.f;

    double framesPerSecondValue{};
    if (p_settings_inout->getFramesPerSecond(framesPerSecondValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // getFramesPerSecond cannot fail; continue as before.
    }
    float fps = static_cast<float>(framesPerSecondValue);
    if (fps < 1)
        fps = 30;
    framePeriod = 1e3 / fps;

    cv::Size imageSize{};
    if (p_settings_inout->newImSize(imageSize) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // newImSize cannot fail; continue as before.
    }
    imageHeight = imageSize.height;
    imageWidth  = imageSize.width;

    double settingsImageViewerScale{};
    if (p_settings_inout->imageViewerScale(settingsImageViewerScale) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // imageViewerScale cannot fail; continue as before.
    }
    imageViewerScale = settingsImageViewerScale;
    double settingsViewPointX{};
    if (p_settings_inout->viewPointX(settingsViewPointX) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // viewPointX cannot fail; continue as before.
    }
    viewpointX = settingsViewPointX;
    double settingsViewPointY{};
    if (p_settings_inout->viewPointY(settingsViewPointY) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // viewPointY cannot fail; continue as before.
    }
    viewpointY = settingsViewPointY;
    double settingsViewPointZ{};
    if (p_settings_inout->viewPointZ(settingsViewPointZ) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // viewPointZ cannot fail; continue as before.
    }
    viewpointZ = settingsViewPointZ;
    double settingsViewPointF{};
    if (p_settings_inout->viewPointF(settingsViewPointF) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // viewPointF cannot fail; continue as before.
    }
    viewpointF = settingsViewPointF;
}

} // namespace core
} // namespace vs_graphs
