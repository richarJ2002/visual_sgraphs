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
 * @file            calculateMergeMapTransform.cc
 *
 * @brief           Implements calculateMergeMapTransform(), declared in
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

g2o::Sim3 calculateMergeMapTransform(
    keyframe::KeyFrame *p_currentKeyFrame_in,
    const g2o::Sim3    &mergeSimilarity_mergeWorldToCurrentCamera_in)
{
    Sophus::SE3f currentKFPoseInverse{};
    if (p_currentKeyFrame_in->getPoseInverse(currentKFPoseInverse) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const Sophus::SE3d cameraPose_cameraToWorld =
        currentKFPoseInverse.cast<double>();
    const g2o::Sim3 g2oNonCorrectedSwc(
        cameraPose_cameraToWorld.unit_quaternion(),
        cameraPose_cameraToWorld.translation(),
        1.0);
    return g2oNonCorrectedSwc * mergeSimilarity_mergeWorldToCurrentCamera_in;
}

} /* namespace core */
} /* namespace vs_graphs */
