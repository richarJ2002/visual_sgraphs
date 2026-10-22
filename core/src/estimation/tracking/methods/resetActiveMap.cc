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
 * @file            resetActiveMap.cc
 *
 * @brief           Implements Tracking::resetActiveMap(), declared in
 *                  Tracking.h.
 */

#include "place_recognition_and_loop_closure/keyframe_database/keyframe_database.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/frame/frame.h"
#include "map_state/keyframe/keyframe.h"
#include "place_recognition_and_loop_closure/keyframe_database/objects/KeyFrameDatabaseStatusEnum.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"
#include "mapping/local_mapping/local_mapping.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "execution/system/system.h"
#include "estimation/tracking/objects/TrackingClass.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::resetActiveMap(bool isRequestedByLocalMapping_in)
{
    if (p_loopClosing)
    {
        for (;;)
        {
            bool loopClosingIsMergeInProgress{};
            if (p_loopClosing->isMergeInProgress(
                    loopClosingIsMergeInProgress) !=
                loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isMergeInProgress returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!loopClosingIsMergeInProgress)
            {
                break;
            }
            usleep(1000);
        }
    }

    if (system::Verbose::printMess("Active map Reseting", system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_map = nullptr;
    if (p_atlas->getCurrentMap(p_map) != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (!isRequestedByLocalMapping_in)
    {
        if (system::Verbose::printMess(
                "[Tracking] Reseting 'local_mapping::LocalMapping' ...",
                system::Verbose::VERBOSITY_VERY_VERBOSE) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_localMapper->requestResetActiveMap(p_map) !=
            local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestResetActiveMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (system::Verbose::printMess(
                "[Tracking] Finished resetting 'local_mapping::LocalMapping'!",
                system::Verbose::VERBOSITY_VERY_VERBOSE) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Reset Loop Closing */
    if (system::Verbose::printMess(
            "[Tracking] Reseting 'loop_closing::LoopClosing' ...",
            system::Verbose::VERBOSITY_NORMAL) != system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_loopClosing->requestResetActiveMap(p_map) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestResetActiveMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (system::Verbose::printMess(
            "[Tracking] Finished resetting 'local_mapping::LocalMapping'!",
            system::Verbose::VERBOSITY_NORMAL) != system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Clear BoW Database */
    if (system::Verbose::printMess("[Tracking] Reseting 'Database' ...",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameDatabase->clearMap(p_map) !=
        keyframe_database::KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (system::Verbose::printMess("[Tracking] Finished resetting 'Database'!",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Clear map::Map (this erase MapPoints and KeyFrames) */
    if (p_atlas->clearMap() != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    lastInitFrameId = frame::Frame::nextId;
    state           = NO_IMAGES_YET;

    isReadyToInitialize = false;

    unsigned int            index = firstFrameId;
    std::vector<map::Map *> atlasAllMaps{};
    if (p_atlas->getAllMaps(atlasAllMaps) != atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (map::Map *p_atlasMap : atlasAllMaps)
    {
        std::vector<keyframe::KeyFrame *> mapAllKeyFrames{};
        if (p_atlasMap->getAllKeyFrames(mapAllKeyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (mapAllKeyFrames.size() > 0)
        {
            unsigned int mapLowerKeyFrameId{};
            if (p_atlasMap->getLowerKeyFrameId(mapLowerKeyFrameId) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getLowerKeyFrameId returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (index > mapLowerKeyFrameId)
            {
                unsigned int mapLowerKeyFrameId2{};
                if (p_atlasMap->getLowerKeyFrameId(mapLowerKeyFrameId2) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getLowerKeyFrameId returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                index = mapLowerKeyFrameId2;
            }
        }
    }

    /* Count lost frames */
    std::list<bool> lbLost;
    int             lostFrameCount = 0;
    for (std::list<bool>::iterator ilbL = lostFlags.begin();
         ilbL != lostFlags.end();
         ilbL++)
    {
        if (index < initialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            lostFrameCount += 1;
        }
        index++;
    }
    std::cout << "[Tracking] " << lostFrameCount << " frames were set to lost!"
              << std::endl;

    lostFlags = lbLost;

    initialFrameId   = currentFrame.id;
    lastRelocFrameId = currentFrame.id;

    currentFrame   = frame::Frame();
    lastFrame      = frame::Frame();
    p_referenceKF  = static_cast<keyframe::KeyFrame *>(nullptr);
    p_lastKeyFrame = static_cast<keyframe::KeyFrame *>(nullptr);
    iniMatches.clear();

    isVelocityAvailable = false;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
