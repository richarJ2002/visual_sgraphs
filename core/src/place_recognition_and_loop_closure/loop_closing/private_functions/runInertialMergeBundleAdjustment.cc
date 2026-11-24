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
 * @file            runInertialMergeBundleAdjustment.cc
 *
 * @brief           Implements runInertialMergeBundleAdjustment(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "estimation/tracking/objects/TrackingStatusEnum.h"
#include "estimation/tracking/tracking.h"
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

bool runInertialMergeBundleAdjustment(
    tracking::Tracking                         *p_tracker_inout,
    keyframe::KeyFrame                         *p_mergeMatchedKeyFrame_inout,
    map::Map                                   *p_currentMap_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout,
    int                                         keyFrameCount_in)
{
    bool inertialBundleAdjustmentRan = false;

    if (keyFrameCount_in >= 10)
    {
        bool                isStopRequested   = false;
        keyframe::KeyFrame *p_currentKeyFrame = nullptr;
        if (p_tracker_inout->getLastKeyFrame(p_currentKeyFrame) !=
            tracking::TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_currentKeyFrame != nullptr)
        {
            if (optimizer::Optimizer::mergeInertialBA(
                    p_currentKeyFrame,
                    p_mergeMatchedKeyFrame_inout,
                    &isStopRequested,
                    p_currentMap_inout,
                    correctedKeyFramePoses_inout) !=
                optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: mergeInertialBA returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            inertialBundleAdjustmentRan = true;
        }
    }

    return inertialBundleAdjustmentRan;
}

} /* namespace core */
} /* namespace vs_graphs */
