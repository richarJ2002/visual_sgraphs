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
 * @file            buildCurrentLocalWindow.cc
 *
 * @brief           Implements buildCurrentLocalWindow(), declared in
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

void buildCurrentLocalWindow(
    keyframe::KeyFrame              *p_currentKeyFrame_in,
    map::Map                        *p_currentMap_in,
    map::Map                        *p_mergeMap_in,
    std::set<keyframe::KeyFrame *>  &windowKeyFrames_inout,
    std::set<map_point::MapPoint *> &windowMapPoints_inout)
{
    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    bool currentMapIsInertial{};
    if (p_currentMap_in->isInertial(currentMapIsInertial) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool mergeMapIsInertial{};
    if ((currentMapIsInertial) &&
        p_mergeMap_in->isInertial(mergeMapIsInertial) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentMapIsInertial && mergeMapIsInertial)
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        keyframe::KeyFrame *p_keyFrame    = p_currentKeyFrame_in;
        int                 insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            windowKeyFrames_inout.insert(p_keyFrame);

            std::set<map_point::MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPoints(mapPoints) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            windowMapPoints_inout.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_prevKF;
            insertedCount++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        p_keyFrame    = p_currentKeyFrame_in->p_nextKF;
        insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            windowKeyFrames_inout.insert(p_keyFrame);

            std::set<map_point::MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPoints(mapPoints) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            windowMapPoints_inout.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_nextKF;
            insertedCount++;
        }
    }
    else
    {
        windowKeyFrames_inout.insert(p_currentKeyFrame_in);
    }

    expandWindowByCovisibility(p_currentKeyFrame_in, windowKeyFrames_inout);
    collectWindowMapPoints(windowKeyFrames_inout, windowMapPoints_inout);
}

} /* namespace core */
} /* namespace vs_graphs */
