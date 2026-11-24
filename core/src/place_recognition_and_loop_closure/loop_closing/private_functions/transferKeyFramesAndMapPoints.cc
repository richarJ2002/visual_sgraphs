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
 * @file            transferKeyFramesAndMapPoints.cc
 *
 * @brief           Implements transferKeyFramesAndMapPoints(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void transferKeyFramesAndMapPoints(
    map::Map                                 *p_currentMap_inout,
    map::Map                                 *p_mergeMap_inout,
    const std::vector<keyframe::KeyFrame *>  &keyFrames_in,
    const std::vector<map_point::MapPoint *> &mapPoints_in)
{
    /*!
     * Loop over the KeyFrames of the current map and move them to the
     * new map
     */
    for (keyframe::KeyFrame *p_keyFrame : keyFrames_in)
    {
        bool keyFrameIsBad9{};
        if (!(!p_keyFrame) &&
            p_keyFrame->isBad(keyFrameIsBad9) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_keyFrameMap4 = nullptr;
        if (!(!p_keyFrame || keyFrameIsBad9) &&
            p_keyFrame->getMap(p_keyFrameMap4) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad9 || p_keyFrameMap4 != p_mergeMap_inout)
            continue;

        if (p_keyFrame->updateMap(p_currentMap_inout) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->addKeyFrame(p_keyFrame) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mergeMap_inout->eraseKeyFrame(p_keyFrame) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * Loop over the MapPoints of the current map and move them to the
     * new map
     */
    for (map_point::MapPoint *p_currentMapPoint : mapPoints_in)
    {
        bool currentMapPointIsBad4{};
        if (!(!p_currentMapPoint) &&
            p_currentMapPoint->isBad(currentMapPointIsBad4) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_currentMapPointMap4 = nullptr;
        if (!(!p_currentMapPoint || currentMapPointIsBad4) &&
            p_currentMapPoint->getMap(p_currentMapPointMap4) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_currentMapPoint || currentMapPointIsBad4 ||
            p_currentMapPointMap4 != p_mergeMap_inout)
            continue;

        if (p_currentMapPoint->updateMap(p_currentMap_inout) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->addMapPoint(p_currentMapPoint) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mergeMap_inout->eraseMapPoint(p_currentMapPoint) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapPoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
