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

void Tracking::resetActiveMap(bool isRequestedByLocalMapping_in)
{
    if (p_loopClosing)
    {
        while (p_loopClosing->isMergeInProgress())
        {
            usleep(1000);
        }
    }

    Verbose::printMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if (p_viewer)
    {
        p_viewer->requestStop();
        while (!p_viewer->isStopped())
        {
            usleep(3000);
        }
    }

    Map *p_map = p_atlas->getCurrentMap();

    if (!isRequestedByLocalMapping_in)
    {
        Verbose::printMess("[Tracking] Reseting 'LocalMapping' ...",
                           Verbose::VERBOSITY_VERY_VERBOSE);
        p_localMapper->requestResetActiveMap(p_map);
        Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                           Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::printMess("[Tracking] Reseting 'LoopClosing' ...",
                       Verbose::VERBOSITY_NORMAL);
    p_loopClosing->requestResetActiveMap(p_map);
    Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                       Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::printMess("[Tracking] Reseting 'Database' ...",
                       Verbose::VERBOSITY_NORMAL);
    p_keyFrameDatabase->clearMap(p_map);
    Verbose::printMess("[Tracking] Finished resetting 'Database'!",
                       Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    p_atlas->clearMap();

    lastInitFrameId = Frame::nextId;
    state           = NO_IMAGES_YET;

    isReadyToInitialize = false;

    unsigned int index = firstFrameId;
    for (Map *p_map : p_atlas->getAllMaps())
    {
        std::vector<KeyFrame *> mapAllKeyFrames{};
        if (p_map->getAllKeyFrames(mapAllKeyFrames) !=
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
            if (p_map->getLowerKeyFrameId(mapLowerKeyFrameId) !=
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
                if (p_map->getLowerKeyFrameId(mapLowerKeyFrameId2) !=
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
    for (list<bool>::iterator ilbL = lostFlags.begin(); ilbL != lostFlags.end();
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
              << endl;

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
        p_viewer->release();
}

} // namespace core
} // namespace vs_graphs
