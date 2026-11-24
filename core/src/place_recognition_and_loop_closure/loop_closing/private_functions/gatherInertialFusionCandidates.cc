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
 * @file            gatherInertialFusionCandidates.cc
 *
 * @brief           Implements gatherInertialFusionCandidates(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void gatherInertialFusionCandidates(
    keyframe::KeyFrame                 *p_mergeMatchedKeyFrame_in,
    keyframe::KeyFrame                 *p_currentKeyFrame_inout,
    std::vector<keyframe::KeyFrame *>  &mergeConnectedKeyFrames_out,
    std::vector<keyframe::KeyFrame *>  &currentConnectedKeyFrames_out,
    std::vector<map_point::MapPoint *> &mapPointsToFuse_out)
{
    currentConnectedKeyFrames_out.clear();

    mergeConnectedKeyFrames_out.clear();
    mergeConnectedKeyFrames_out.push_back(p_mergeMatchedKeyFrame_in);
    std::vector<keyframe::KeyFrame *> aux{};
    if (p_mergeMatchedKeyFrame_in->getVectorCovisibleKeyFrames(aux) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    mergeConnectedKeyFrames_out.insert(mergeConnectedKeyFrames_out.end(),
                                       aux.begin(),
                                       aux.end());
    if (mergeConnectedKeyFrames_out.size() > 6)
        mergeConnectedKeyFrames_out.erase(mergeConnectedKeyFrames_out.begin() +
                                              6,
                                          mergeConnectedKeyFrames_out.end());

    if (p_currentKeyFrame_inout->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    currentConnectedKeyFrames_out.push_back(p_currentKeyFrame_inout);
    std::vector<keyframe::KeyFrame *> currentKFVectorCovisibleKeyFrames{};
    if (p_currentKeyFrame_inout->getVectorCovisibleKeyFrames(
            currentKFVectorCovisibleKeyFrames) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    aux = currentKFVectorCovisibleKeyFrames;
    currentConnectedKeyFrames_out.insert(currentConnectedKeyFrames_out.end(),
                                         aux.begin(),
                                         aux.end());
    if (currentConnectedKeyFrames_out.size() > 6)
        currentConnectedKeyFrames_out.erase(
            currentConnectedKeyFrames_out.begin() + 6,
            currentConnectedKeyFrames_out.end());

    std::set<map_point::MapPoint *> mapPointMerges;
    for (keyframe::KeyFrame *p_keyFrame : mergeConnectedKeyFrames_out)
    {
        std::set<map_point::MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPoints(mapPoints) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        mapPointMerges.insert(mapPoints.begin(), mapPoints.end());
        if (mapPointMerges.size() > 1000)
            break;
    }

    mapPointsToFuse_out.reserve(mapPointMerges.size());
    std::copy(mapPointMerges.begin(),
              mapPointMerges.end(),
              std::back_inserter(mapPointsToFuse_out));
}

} /* namespace core */
} /* namespace vs_graphs */
