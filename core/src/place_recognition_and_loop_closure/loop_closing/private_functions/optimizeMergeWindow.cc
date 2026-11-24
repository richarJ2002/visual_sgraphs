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
 * @file            optimizeMergeWindow.cc
 *
 * @brief           Implements optimizeMergeWindow(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "estimation/tracking/tracking.h"
#include "execution/system/system.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "optimization/optimizer/optimizer.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void optimizeMergeWindow(
    tracking::Tracking                         *p_tracker_in,
    keyframe::KeyFrame                         *p_currentKeyFrame_inout,
    keyframe::KeyFrame                         *p_mergeMatchedKeyFrame_inout,
    map::Map                                   *p_currentMap_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout,
    const std::vector<keyframe::KeyFrame *>    &mergeConnectedKeyFrames_in,
    const std::vector<keyframe::KeyFrame *>    &currentWindowKeyFrames_in)
{
    /* Shared stop flag passed to the selected optimisation routine */
    bool shouldStop = false;

    /* Check whether the active sensor configuration includes an IMU */
    if (p_tracker_in->sensor == system::System::IMU_MONOCULAR ||
        p_tracker_in->sensor == system::System::IMU_STEREO ||
        p_tracker_in->sensor == system::System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        if (optimizer::Optimizer::mergeInertialBA(
                p_currentKeyFrame_inout,
                p_mergeMatchedKeyFrame_inout,
                &shouldStop,
                p_currentMap_inout,
                correctedKeyFramePoses_inout) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeInertialBA returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        /* Refine the merged region using visual observations only */
        if (optimizer::Optimizer::loopClosureLocalBundleAdjustment(
                p_mergeMatchedKeyFrame_inout,
                mergeConnectedKeyFrames_in,
                currentWindowKeyFrames_in,
                &shouldStop) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: loopClosureLocalBundleAdjustment returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
