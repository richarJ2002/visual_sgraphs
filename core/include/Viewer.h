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
    Viewer(System                    *pSystem,
           FrameDrawer               *pFrameDrawer,
           MapDrawer                 *pMapDrawer,
           Tracking                  *pTracking,
           const string              &strSettingPath,
           utils::settings::Settings *settings) :
        both(false),
        p_system(pSystem),
        p_frameDrawer(pFrameDrawer),
        p_mapDrawer(pMapDrawer),
        p_tracker(pTracking),
        finishRequested(false),
        finished(true),
        stopped(true),
        stopRequestedFlag(false)
    {
        if (settings)
        {
            newParameterLoader(settings);
        }
        else
        {

            cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

            bool is_correct = parseViewerParamFile(fSettings);

            if (!is_correct)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (exception &e)
                {}
            }
        }

        stopTrack = false;
    }

    void newParameterLoader(utils::settings::Settings *settings);

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

    bool both;

  private:
    bool parseViewerParamFile(cv::FileStorage &fSettings);

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
    bool       finishRequested;
    bool       finished;
    std::mutex mMutexFinish;

    bool       stopped;
    bool       stopRequestedFlag;
    std::mutex mMutexStop;

    bool stopTrack;
};

} // namespace core
} // namespace vs_graphs
#endif // VIEWER_H
