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
 * @file            reverseMergeSpanningTree.cc
 *
 * @brief           Implements reverseMergeSpanningTree(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void reverseMergeSpanningTree(keyframe::KeyFrame *p_currentKeyFrame_in,
                              keyframe::KeyFrame *p_mergeMatchedKeyFrame_inout,
                              map::Map           *p_mergeMap_in)
{
    /* If the oriign keyframe of the merp map is valid */
    keyframe::KeyFrame *p_mergeMapOriginKeyFrame = nullptr;
    if (p_mergeMap_in->getOriginKeyFrame(p_mergeMapOriginKeyFrame) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getOriginKeyFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mergeMapOriginKeyFrame)
    {
        /* Allow the former merge-map root to become a normal tree child */
        keyframe::KeyFrame *p_mergeMapOriginKeyFrame2 = nullptr;
        if (p_mergeMap_in->getOriginKeyFrame(p_mergeMapOriginKeyFrame2) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getOriginKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mergeMapOriginKeyFrame2->setFirstConnection(false) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFirstConnection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Init variables to track the new child and parent keyframes */
    keyframe::KeyFrame *p_newChild  = nullptr;
    keyframe::KeyFrame *p_newParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    keyframe::KeyFrame *p_mergeMatchedKFParent = nullptr;
    if (p_mergeMatchedKeyFrame_inout->getParent(p_mergeMatchedKFParent) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_newChild = p_mergeMatchedKFParent;

    /* The matched merge keyframe becomes the first reversed parent */
    p_newParent = p_mergeMatchedKeyFrame_inout;

    /* Attach the matched merge keyframe beneath the current keyframe */
    if (p_mergeMatchedKeyFrame_inout->changeParent(p_currentKeyFrame_in) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: changeParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Reverse each edge along the original merge-map parent chain */
    while (p_newChild)
    {
        /* Remove the old child edge before reversing its direction */
        if (p_newChild->eraseChild(p_newParent) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseChild returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Save the next original parent before changing this relation */
        keyframe::KeyFrame *p_oldParent = nullptr;
        if (p_newChild->getParent(p_oldParent) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Make the former parent a child of the previous keyframe */
        if (p_newChild->changeParent(p_newParent) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: changeParent returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Advance the new-parent pointer one level up the old chain */
        p_newParent = p_newChild;

        /* Continue with the next parent from the original tree chain */
        p_newChild = p_oldParent;
    }
}

} /* namespace core */
} /* namespace vs_graphs */
