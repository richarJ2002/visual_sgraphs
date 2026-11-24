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
 * @file            collectWindowMapPoints.cc
 *
 * @brief           Implements collectWindowMapPoints(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void collectWindowMapPoints(
    const std::set<keyframe::KeyFrame *> &windowKeyFrames_in,
    std::set<map_point::MapPoint *>      &windowMapPoints_inout)
{
    for (keyframe::KeyFrame *p_keyFrame : windowKeyFrames_in)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        bool keyFrameIsBad{};
        if (!(!p_keyFrame) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad)
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        std::set<map_point::MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPoints(mapPoints) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Insert all the map points into the map point local window */
        windowMapPoints_inout.insert(mapPoints.begin(), mapPoints.end());
    }
}

} /* namespace core */
} /* namespace vs_graphs */
