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
 * @file            propagateMergedPoseCorrections.cc
 *
 * @brief           Implements propagateMergedPoseCorrections(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void propagateMergedPoseCorrections(
    map::Map *p_mergeMap_inout,
    const loop_closing::LoopClosing::KeyFrameAndPose
        &nonCorrectedKeyFramePoses_in,
    const loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in)
{
    using KeyFrameAndPose = loop_closing::LoopClosing::KeyFrameAndPose;

    KeyFrameAndPose finalKeyFramePoses_worldToCamera;

    for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
         nonCorrectedKeyFramePoses_in)
    {
        (void)poseBefore_worldToCamera;

        bool keyFrameIsBad8{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad8) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || keyFrameIsBad8)
        {
            continue;
        }

        Sophus::SE3f keyFramePose5{};
        if (p_keyFrame->getPose(keyFramePose5) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Sophus::SE3d poseAfter_worldToCamera =
            keyFramePose5.cast<double>();

        /*
         * A monocular map merge can change scale. ORB-SLAM stores the
         * corrected keyframe as SE3 by dividing the Sim3 translation by
         * its scale, so reconstruct the corresponding Sim3 before
         * deriving the semantic world-frame correction. Using a unit
         * scale here would leave planes, rooms and passages at their old
         * size while MapPoints receive the full map Sim3.
         */
        double poseAfterScale = 1.0;

        const KeyFrameAndPose::const_iterator correctedPoseIterator =
            correctedKeyFramePoses_in.find(p_keyFrame);

        if (correctedPoseIterator != correctedKeyFramePoses_in.end() &&
            std::isfinite(correctedPoseIterator->second.scale()) &&
            std::abs(correctedPoseIterator->second.scale()) > 1e-12)
        {
            poseAfterScale = correctedPoseIterator->second.scale();
        }

        finalKeyFramePoses_worldToCamera.insert_or_assign(
            p_keyFrame,
            g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                      poseAfterScale * poseAfter_worldToCamera.translation(),
                      poseAfterScale));
    }

    if (utils::utils::Utils::propagateSemanticPoseCorrections(
            p_mergeMap_inout,
            nonCorrectedKeyFramePoses_in,
            finalKeyFramePoses_worldToCamera,
            mapTransform_mergeWorldToCurrentWorld_in) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: propagateSemanticPoseCorrections returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
