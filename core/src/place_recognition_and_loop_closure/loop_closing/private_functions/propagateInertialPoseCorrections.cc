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
 * @file            propagateInertialPoseCorrections.cc
 *
 * @brief           Implements propagateInertialPoseCorrections(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void propagateInertialPoseCorrections(
    map::Map         *p_currentMap_inout,
    SemanticsManager *p_semanticsManager_in,
    const loop_closing::LoopClosing::KeyFrameAndPose
                                               &nonCorrectedKeyFramePoses_in,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout)
{
    /* Complete the post-BA pose map, including fixed deformation nodes. */
    for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
         nonCorrectedKeyFramePoses_in)
    {
        (void)poseBefore_worldToCamera;

        bool keyFrameIsBad4{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad4) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_keyFrameMap2 = nullptr;
        if (!(p_keyFrame == nullptr || keyFrameIsBad4) &&
            p_keyFrame->getMap(p_keyFrameMap2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || keyFrameIsBad4 ||
            p_keyFrameMap2 != p_currentMap_inout)
        {
            continue;
        }

        Sophus::SE3f keyFramePose2{};
        if (p_keyFrame->getPose(keyFramePose2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Sophus::SE3d poseAfter_worldToCamera =
            keyFramePose2.cast<double>();

        correctedKeyFramePoses_inout.insert_or_assign(
            p_keyFrame,
            g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                      poseAfter_worldToCamera.translation(),
                      1.0));
    }

    const g2o::Sim3 identityTransform_worldToWorld(
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::Zero(),
        1.0);

    if (utils::utils::Utils::propagateSemanticPoseCorrections(
            p_currentMap_inout,
            p_semanticsManager_in,
            nonCorrectedKeyFramePoses_in,
            correctedKeyFramePoses_inout,
            identityTransform_worldToWorld) !=
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
