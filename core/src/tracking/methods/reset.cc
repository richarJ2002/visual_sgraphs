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
 * @brief           Implements Tracking::reset(), declared in
 * tracking/objects/TrackingClass.h.
 */

#include "keyframe_database/keyframe_database.h"
#include "atlas/objects/AtlasStatusEnum.h"
#include "frame/frame.h"
#include "keyframe/keyframe.h"
#include "keyframe_database/objects/KeyFrameDatabaseStatusEnum.h"
#include "local_mapping/objects/LocalMappingStatusEnum.h"
#include "loop_closing/objects/LoopClosingStatusEnum.h"
#include "system/objects/VerboseStatusEnum.h"
#include "local_mapping/local_mapping.h"
#include "loop_closing/loop_closing.h"
#include "system/system.h"
#include "tracking/objects/TrackingClass.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::reset(bool isRequestedByLocalMapping_in)
{
    if (system::Verbose::printMess("system::System Reseting",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Reset Local Mapping */
    if (!isRequestedByLocalMapping_in)
    {
        if (system::Verbose::printMess("Reseting Local Mapper...",
                               system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_localMapper->requestReset() !=
            local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestReset returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (system::Verbose::printMess("done", system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Reset Loop Closing */
    if (system::Verbose::printMess("Reseting Loop Closing...",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_loopClosing->requestReset() !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestReset returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (system::Verbose::printMess("done", system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Clear BoW Database */
    if (system::Verbose::printMess("Reseting Database...", system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameDatabase->clear() !=
        keyframe_database::KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clear returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (system::Verbose::printMess("done", system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Clear map::Map (this erase MapPoints and KeyFrames) */
    if (p_atlas->clearAtlas() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearAtlas returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlas->createNewMap() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createNewMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (sensor == system::System::IMU_STEREO ||
        sensor == system::System::IMU_MONOCULAR ||
        sensor == system::System::IMU_RGBD)
    {
        if (p_atlas->setInertialSensor() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialSensor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    initialFrameId = 0;

    keyframe::KeyFrame::nextId = 0;
    frame::Frame::nextId       = 0;
    state                      = NO_IMAGES_YET;

    isReadyToInitialize = false;

    relativeFramePoses.clear();
    referenceKeyFrames.clear();
    frameTimes.clear();
    lostFlags.clear();
    currentFrame     = frame::Frame();
    lastRelocFrameId = 0;
    lastFrame        = frame::Frame();
    p_referenceKF    = static_cast<keyframe::KeyFrame *>(nullptr);
    p_lastKeyFrame   = static_cast<keyframe::KeyFrame *>(nullptr);
    iniMatches.clear();

    if (system::Verbose::printMess("   End reseting! ", system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
