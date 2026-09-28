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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Atlas.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Utils/Settings/objects/Settings.h"
#include <pangolin/pangolin.h>

#include <iostream>
#include <mutex>

namespace vs_graphs
{
namespace core
{

class MapDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas                     *p_atlas_in,
              const string              &settingsFilePath_in,
              utils::settings::Settings *p_settings_in) :
        p_atlas(p_atlas_in)
    {
        if (p_settings_in)
        {
            newParameterLoader(p_settings_in);
        }
        else
        {
            cv::FileStorage settingsFileStorage(settingsFilePath_in,
                                                cv::FileStorage::READ);
            bool            isViewerConfigValid =
                parseViewerParamFile(settingsFileStorage);

            if (!isViewerConfigValid)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (exception &caughtException)
                {}
            }
        }
    }

    void newParameterLoader(utils::settings::Settings *p_settings_inout);

    Atlas *p_atlas;

    void drawMapPoints();
    void drawKeyFrames(const bool shouldDrawKeyFrames_in,
                       const bool shouldDrawGraph_in,
                       const bool shouldDrawInertialGraph_in,
                       const bool shouldDrawOptimizedLba_in);
    void drawCurrentCamera(pangolin::OpenGlMatrix &Twc_in);
    void setCurrentCameraPose(const Sophus::SE3f &Tcw_in);
    void setReferenceKeyFrame(KeyFrame *p_keyFrame_in);
    void getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M_in,
                                      pangolin::OpenGlMatrix &MOw_inout);

  private:
    bool parseViewerParamFile(cv::FileStorage &settings_in);

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
