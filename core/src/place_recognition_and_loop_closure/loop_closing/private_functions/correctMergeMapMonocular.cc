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
 * @file            correctMergeMapMonocular.cc
 *
 * @brief           Implements correctMergeMapMonocular(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void correctMergeMapMonocular(
    map::Map                             *p_currentMap_in,
    map::Map                             *p_mergeMap_inout,
    const loop_closing::MergeMapContents &mergeMapContents_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in,
    const g2o::Sim3 &mapTransform_currentWorldToMergeWorld_in,
    loop_closing::LoopClosing::KeyFrameAndPose &nonCorrectedKeyFramePoses_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout)
{
    /* Lock the merge map while updating its poses and landmarks */
    std::unique_lock<std::mutex> mergeLock(p_mergeMap_inout->mapUpdateMutex);

    /* Correct each remaining merge keyframe into current world */
    for (keyframe::KeyFrame *p_keyFrame : mergeMapContents_in.keyFrames)
    {
        /* Skip invalid keyframes or keyframes no longer in this map */
        bool keyFrameIsBad7{};
        if (!(!p_keyFrame) &&
            p_keyFrame->isBad(keyFrameIsBad7) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_keyFrameMap3 = nullptr;
        if (!(!p_keyFrame || keyFrameIsBad7) &&
            p_keyFrame->getMap(p_keyFrameMap3) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad7 || p_keyFrameMap3 != p_mergeMap_inout)
        {
            continue;
        }

        /* Read the keyframe pose in the merge map world frame */
        Sophus::SE3f keyFramePose3{};
        if (p_keyFrame->getPose(keyFramePose3) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const Sophus::SE3d TiwMerge = keyFramePose3.cast<double>();

        /* Convert the rigid keyframe pose into a unit scale Sim3 */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Express the keyframe pose in the current world frame */
        const g2o::Sim3 g2oSiwCurrent =
            g2oSiwMerge * mapTransform_currentWorldToMergeWorld_in;

        /* Store the original keyframe pose before correction */
        nonCorrectedKeyFramePoses_inout[p_keyFrame] = g2oSiwMerge;

        /* Store the corrected keyframe pose for later processing */
        correctedKeyFramePoses_inout[p_keyFrame] = g2oSiwCurrent;

        /* Extract the scale introduced by the map correction */
        const double s = g2oSiwCurrent.scale();

        /* Store the applied scale in the keyframe */
        p_keyFrame->correctedScale = s;

        /* Preserve the original world to camera pose */
        Sophus::SE3f keyFramePose4{};
        if (p_keyFrame->getPose(keyFramePose4) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_keyFrame->tcwBefMerge = keyFramePose4;

        /* Preserve the original camera to world pose */
        Sophus::SE3f keyFramePoseInverse2{};
        if (p_keyFrame->getPoseInverse(keyFramePoseInverse2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getPoseInverse returned a failure status although "
                "it cannot fail; continuing as before.",
                __func__);
        }
        p_keyFrame->twcBefMerge = keyFramePoseInverse2;

        /* Apply the corrected rigid pose in the current world frame */
        if (p_keyFrame->setPose(Sophus::SE3d(g2oSiwCurrent.rotation(),
                                             g2oSiwCurrent.translation() / s)
                                    .cast<float>()) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Rotate velocity when the surviving map uses inertial data */
        bool currentMapIsImuInitialized3{};
        if (p_currentMap_in->isImuInitialized(currentMapIsImuInitialized3) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentMapIsImuInitialized3)
        {
            /* Compute the rotation from old to corrected world frame */
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();

            /* Express the keyframe velocity in the corrected frame */
            Eigen::Vector3f keyFrameVelocity2{};
            if (p_keyFrame->getVelocity(keyFrameVelocity2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame->setVelocity(Rcor * keyFrameVelocity2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Correct each remaining merge landmark into current world */
    correctMapPointsToCurrentWorld(mergeMapContents_in.mapPoints,
                                   p_mergeMap_inout,
                                   mapTransform_mergeWorldToCurrentWorld_in);
}

} /* namespace core */
} /* namespace vs_graphs */
