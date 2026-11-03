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
 * @file            expandWindowByCovisibility.cc
 *
 * @brief           Implements expandWindowByCovisibility(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void expandWindowByCovisibility(
    keyframe::KeyFrame             *p_centerKeyFrame_in,
    std::set<keyframe::KeyFrame *> &windowKeyFrames_inout)
{
    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<keyframe::KeyFrame *> covisibleKeyFrames{};
    if (p_centerKeyFrame_in->getBestCovisibilityKeyFrames(
            COUNT_TEMPORAL_KEY_FRAMES,
            covisibleKeyFrames) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Insert keyframes with best connections into local window */
    windowKeyFrames_inout.insert(covisibleKeyFrames.begin(),
                                 covisibleKeyFrames.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    windowKeyFrames_inout.insert(p_centerKeyFrame_in);

    int expansionCount = 0;

    /*!
     * If there is not enough keyframes in the local window, then we look at the
     * keyframes currently in spLocalWindowKF and find there best covisibility
     * keyframes. We do this to gaurantee there is enough keyframes in the
     * `spLocalWindowKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spLocalWindowKFs`, if the current keyframe doesn't
     * have enough covisible keyframes attached to it.
     */
    while (windowKeyFrames_inout.size() < COUNT_TEMPORAL_KEY_FRAMES &&
           expansionCount < MAXIMUM_EXPANSION_ITERATIONS)
    {
        std::vector<keyframe::KeyFrame *> newCovisibles;

        for (keyframe::KeyFrame *p_keyFrame : windowKeyFrames_inout)
        {
            std::vector<keyframe::KeyFrame *> covisibles{};
            if (p_keyFrame->getBestCovisibilityKeyFrames(
                    COUNT_TEMPORAL_KEY_FRAMES / 2,
                    covisibles) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getBestCovisibilityKeyFrames returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            for (keyframe::KeyFrame *p_covisibleKeyFrame : covisibles)
            {
                if (!p_covisibleKeyFrame)
                    continue;

                bool covisibleKeyFrameIsBad{};
                if (p_covisibleKeyFrame->isBad(covisibleKeyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (covisibleKeyFrameIsBad)
                    continue;

                if (windowKeyFrames_inout.count(p_covisibleKeyFrame))
                    continue;

                newCovisibles.push_back(p_covisibleKeyFrame);
            }
        }

        windowKeyFrames_inout.insert(newCovisibles.begin(),
                                     newCovisibles.end());

        ++expansionCount;
    }
}

} /* namespace core */
} /* namespace vs_graphs */
