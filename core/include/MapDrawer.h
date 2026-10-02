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
 * @file            MapDrawer.h
 *
 * @brief           Declares MapDrawer, which draws the map points, key frames
 *                  and camera in the viewer.
 */

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Atlas.h"
#include "KeyFrame.h"
#include "MapDrawerStatus.h"
#include "MapPoint.h"
#include "Utils/Settings/objects/Settings.h"
#include <pangolin/pangolin.h>

#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

class MapDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas                     *p_atlas_in,
              const std::string         &settingsFilePath_in,
              utils::settings::Settings *p_settings_in) :
        p_atlas(p_atlas_in)
    {
        if (p_settings_in)
        {
            if (newParameterLoader(p_settings_in) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: newParameterLoader returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            cv::FileStorage settingsFileStorage(settingsFilePath_in,
                                                cv::FileStorage::READ);
            bool            isViewerConfigValid{};
            if (parseViewerParamFile(settingsFileStorage,
                                     isViewerConfigValid) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: parseViewerParamFile returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (!isViewerConfigValid)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (std::exception &caughtException)
                {}
            }
        }
    }

    [[nodiscard]] MapDrawerStatus
        newParameterLoader(utils::settings::Settings *p_settings_inout);

    Atlas *p_atlas;

    [[nodiscard]] MapDrawerStatus drawMapPoints() const;
    [[nodiscard]] MapDrawerStatus
        drawKeyFrames(const bool shouldDrawKeyFrames_in,
                      const bool shouldDrawGraph_in,
                      const bool shouldDrawInertialGraph_in,
                      const bool shouldDrawOptimizedLba_in);
    [[nodiscard]] MapDrawerStatus
        drawCurrentCamera(pangolin::OpenGlMatrix &pose_cameraToWorld_in) const;
    [[nodiscard]] MapDrawerStatus
        setCurrentCameraPose(const Sophus::SE3f &pose_worldToCamera_in);
    [[nodiscard]] MapDrawerStatus
        getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M_in,
                                     pangolin::OpenGlMatrix &MOw_inout);

  private:
    [[nodiscard]] MapDrawerStatus
        parseViewerParamFile(cv::FileStorage &settings_in, bool &isParsed_out);

    float keyFrameSize;
    float keyFrameLineWidth;
    float graphLineWidth;
    float pointSize;
    float cameraSize;
    float cameraLineWidth;

    Sophus::SE3f cameraPose;

    std::mutex cameraMutex;

    float frameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                               {0.8f, 0.4f, 1.0f},
                               {1.0f, 0.2f, 0.4f},
                               {0.6f, 0.0f, 1.0f},
                               {1.0f, 1.0f, 0.0f},
                               {0.0f, 1.0f, 1.0f}};
};

} // namespace core
} // namespace vs_graphs

#endif // MAPDRAWER_H
