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
 * @file            reset.cc
 *
 * @brief           Implements Tracking::reset(), declared in Tracking.h.
 */

#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::reset(bool isRequestedByLocalMapping_in)
{
    if (Verbose::printMess("System Reseting", Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_viewer)
    {
        if (p_viewer->requestStop() != ViewerStatus::VIEWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestStop returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (;;)
        {
            bool viewerIsStopped{};
            if (p_viewer->isStopped(viewerIsStopped) !=
                ViewerStatus::VIEWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isStopped returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (viewerIsStopped)
            {
                break;
            }
            usleep(3000);
        }
    }

    // Reset Local Mapping
    if (!isRequestedByLocalMapping_in)
    {
        if (Verbose::printMess("Reseting Local Mapper...",
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_localMapper->requestReset() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestReset returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (Verbose::printMess("done", Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Reset Loop Closing
    if (Verbose::printMess("Reseting Loop Closing...",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_loopClosing->requestReset() !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestReset returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (Verbose::printMess("done", Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Clear BoW Database
    if (Verbose::printMess("Reseting Database...", Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameDatabase->clear() !=
        KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clear returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (Verbose::printMess("done", Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Clear Map (this erase MapPoints and KeyFrames)
    if (p_atlas->clearAtlas() != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearAtlas returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlas->createNewMap() != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createNewMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (sensor == System::IMU_STEREO || sensor == System::IMU_MONOCULAR ||
        sensor == System::IMU_RGBD)
    {
        if (p_atlas->setInertialSensor() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialSensor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
        if (p_viewer->release() != ViewerStatus::VIEWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: release returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (Verbose::printMess("   End reseting! ", Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
