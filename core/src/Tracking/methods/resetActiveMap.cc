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

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
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
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
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

    if (Verbose::printMess("Active map Reseting", Verbose::VERBOSITY_NORMAL) !=
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

    Map *p_map = nullptr;
    if (p_atlas->getCurrentMap(p_map) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (!isRequestedByLocalMapping_in)
    {
        if (Verbose::printMess("[Tracking] Reseting 'LocalMapping' ...",
                               Verbose::VERBOSITY_VERY_VERBOSE) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_localMapper->requestResetActiveMap(p_map) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestResetActiveMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                               Verbose::VERBOSITY_VERY_VERBOSE) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Reset Loop Closing
    if (Verbose::printMess("[Tracking] Reseting 'LoopClosing' ...",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_loopClosing->requestResetActiveMap(p_map) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestResetActiveMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Clear BoW Database
    if (Verbose::printMess("[Tracking] Reseting 'Database' ...",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameDatabase->clearMap(p_map) !=
        KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (Verbose::printMess("[Tracking] Finished resetting 'Database'!",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Clear Map (this erase MapPoints and KeyFrames)
    if (p_atlas->clearMap() != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    lastInitFrameId = Frame::nextId;
    state           = NO_IMAGES_YET;

    isReadyToInitialize = false;

    unsigned int       index = firstFrameId;
    std::vector<Map *> atlasAllMaps{};
    if (p_atlas->getAllMaps(atlasAllMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Map *p_atlasMap : atlasAllMaps)
    {
        std::vector<KeyFrame *> mapAllKeyFrames{};
        if (p_atlasMap->getAllKeyFrames(mapAllKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
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
                MapStatus::MAP_STATUS_SUCCESS)
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
                    MapStatus::MAP_STATUS_SUCCESS)
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

    // Count lost frames
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

    currentFrame   = Frame();
    lastFrame      = Frame();
    p_referenceKF  = static_cast<KeyFrame *>(nullptr);
    p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);
    iniMatches.clear();

    isVelocityAvailable = false;

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

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
