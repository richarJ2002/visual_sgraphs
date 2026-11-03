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
 * @file            correctMergeKeyFramePoses.cc
 *
 * @brief           Implements correctMergeKeyFramePoses(), declared in
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

void correctMergeKeyFramePoses(
    const std::set<keyframe::KeyFrame *> &mergeConnectedKeyFrames_in,
    map::Map                             *p_currentMap_in,
    map::Map                             *p_mergeMap_in,
    const g2o::Sim3 &mapTransform_currentWorldToMergeWorld_in,
    loop_closing::LoopClosing::KeyFrameAndPose &nonCorrectedKeyFramePoses_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout)
{
    /* Iterate through every merge keyframe in the merge connected KF list */
    for (keyframe::KeyFrame *p_keyFrame : mergeConnectedKeyFrames_in)
    {
        /* Skip invalid keyframes */
        bool keyFrameIsBad3{};
        if (!(!p_keyFrame) &&
            p_keyFrame->isBad(keyFrameIsBad3) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_keyFrameMap = nullptr;
        if (!(!p_keyFrame || keyFrameIsBad3) &&
            p_keyFrame->getMap(p_keyFrameMap) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad3 || p_keyFrameMap != p_mergeMap_in)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        Sophus::SE3f keyFramePose{};
        if (p_keyFrame->getPose(keyFramePose) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const Sophus::SE3d TiwMerge = keyFramePose.cast<double>();

        /* Convert to a g2o::Sim3 object type */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Find the transform from current world to keyframe iteration (i) */
        const g2o::Sim3 g2oSiwCurrent =
            g2oSiwMerge * mapTransform_currentWorldToMergeWorld_in;

        /* Store transforms */
        nonCorrectedKeyFramePoses_inout[p_keyFrame] = g2oSiwMerge;
        correctedKeyFramePoses_inout[p_keyFrame]    = g2oSiwCurrent;

        /* Find the scale of transform */
        const double s = g2oSiwCurrent.scale();

        /* Find the transform from merge to current map */
        p_keyFrame->correctedScale = s;
        p_keyFrame->tcwMerge       = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                            g2oSiwCurrent.translation() / s)
                                   .cast<float>();

        /* If there is IMU, extract velocity */
        bool currentMapIsImuInitialized{};
        if (p_currentMap_in->isImuInitialized(currentMapIsImuInitialized) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentMapIsImuInitialized)
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            Eigen::Vector3f keyFrameVelocity{};
            if (p_keyFrame->getVelocity(keyFrameVelocity) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->vwbMerge = Rcor * keyFrameVelocity;
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
