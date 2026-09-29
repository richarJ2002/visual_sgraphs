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

#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"

namespace vs_graphs
{
namespace core
{

void Tracking::reset(bool isRequestedByLocalMapping_in)
{
    Verbose::printMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if (p_viewer)
    {
        p_viewer->requestStop();
        for (;;)
        {
            if (p_viewer->isStopped())
            {
                break;
            }
            usleep(3000);
        }
    }

    // Reset Local Mapping
    if (!isRequestedByLocalMapping_in)
    {
        Verbose::printMess("Reseting Local Mapper...",
                           Verbose::VERBOSITY_NORMAL);
        p_localMapper->requestReset();
        Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::printMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    p_loopClosing->requestReset();
    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::printMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    p_keyFrameDatabase->clear();
    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    p_atlas->clearAtlas();
    p_atlas->createNewMap();
    if (sensor == System::IMU_STEREO || sensor == System::IMU_MONOCULAR ||
        sensor == System::IMU_RGBD)
    {
        p_atlas->setInertialSensor();
    }
    initialFrameId = 0;

    KeyFrame::nextId = 0;
    Frame::nextId    = 0;
    state            = NO_IMAGES_YET;

    isReadyToInitialize = false;
    isInitSet           = false;

    relativeFramePoses.clear();
    referenceKeyFrames.clear();
    frameTimes.clear();
    lostFlags.clear();
    currentFrame     = Frame();
    lastRelocFrameId = 0;
    lastFrame        = Frame();
    p_referenceKF    = static_cast<KeyFrame *>(nullptr);
    p_lastKeyFrame   = static_cast<KeyFrame *>(nullptr);
    iniMatches.clear();

    if (p_viewer)
    {
        p_viewer->release();
    }

    Verbose::printMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

} // namespace core
} // namespace vs_graphs
