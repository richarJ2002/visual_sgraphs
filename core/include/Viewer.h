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

#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Settings/objects/Settings.h"

#include <iostream>
#include <mutex>

namespace vs_graphs
{
namespace core
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System                    *p_system_in,
           FrameDrawer               *p_frameDrawer_in,
           MapDrawer                 *p_mapDrawer_in,
           Tracking                  *p_tracking_in,
           const string              &settingsFilePath_in,
           utils::settings::Settings *p_settings_in) :
        shouldDrawBothImages(false),
        p_system(p_system_in),
        p_frameDrawer(p_frameDrawer_in),
        p_mapDrawer(p_mapDrawer_in),
        p_tracker(p_tracking_in),
        isFinishRequested(false),
        hasFinished(true),
        hasStopped(true),
        isStopRequested(false)
    {
        if (p_settings_in)
        {
            newParameterLoader(p_settings_in);
        }
        else
        {

            cv::FileStorage settingsFileStorage(settingsFilePath_in,
                                                cv::FileStorage::READ);

            bool isConfigValid = parseViewerParamFile(settingsFileStorage);

            if (!isConfigValid)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (exception &parseError)
                {}
            }
        }

        isTrackingStopRequested = false;
    }

    void newParameterLoader(utils::settings::Settings *p_settings_inout);

    // Main thread function. Draw points, keyframes, the current camera pose and
    // the last processed frame. Drawing is refreshed according to the camera
    // fps. We use Pangolin.
    void run();

    void requestFinish();

    void requestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void release();

    // void SetTrackingPause();

    bool shouldDrawBothImages;

  private:
    bool parseViewerParamFile(cv::FileStorage &settings_in);

    bool stop();

    System      *p_system;
    FrameDrawer *p_frameDrawer;
    MapDrawer   *p_mapDrawer;
    Tracking    *p_tracker;

    // 1/fps in ms
    double framePeriod;
    float  imageWidth, imageHeight;
    float  imageViewerScale;

    float viewpointX, viewpointY, viewpointZ, viewpointF;

    bool       checkFinish();
    void       setFinish();
    bool       isFinishRequested;
    bool       hasFinished;
    std::mutex finishMutex;

    bool       hasStopped;
    bool       isStopRequested;
    std::mutex stopMutex;

    bool isTrackingStopRequested;
};

} // namespace core
} // namespace vs_graphs
#endif // VIEWER_H
