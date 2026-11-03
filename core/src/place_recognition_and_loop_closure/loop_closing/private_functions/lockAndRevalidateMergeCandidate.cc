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
 * @file            lockAndRevalidateMergeCandidate.cc
 *
 * @brief           Implements lockAndRevalidateMergeCandidate(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/local_mapping/local_mapping.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool lockAndRevalidateMergeCandidate(
    atlas::Atlas                 *p_atlas_inout,
    local_mapping::LocalMapping  *p_localMapper_inout,
    keyframe::KeyFrame           *p_currentKeyFrame_in,
    keyframe::KeyFrame           *p_mergeMatchedKeyFrame_in,
    map::Map                     *p_currentMap_in,
    map::Map                     *p_mergeMap_in,
    std::unique_lock<std::mutex> &semanticUpdateLock_out)
{
    /*
     * Prevent semantic worker threads from modifying either graph while map
     * frames, ownership, and cross-entity references are being changed.
     */
    if (p_atlas_inout->acquireSemanticUpdateLock(semanticUpdateLock_out) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: acquireSemanticUpdateLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Revalidate after quiescing workers; retained retired maps keep stale
     * raw pointers alive, so pointer non-nullness alone is insufficient. */
    bool currentKFIsBad2{};
    if (p_currentKeyFrame_in->isBad(currentKFIsBad2) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMatchedKFIsBad2{};
    if (!(currentKFIsBad2) &&
        p_mergeMatchedKeyFrame_in->isBad(mergeMatchedKFIsBad2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    map::Map *p_currentKFMap = nullptr;
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2) &&
        p_currentKeyFrame_in->getMap(p_currentKFMap) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    map::Map *p_mergeMatchedKFMap = nullptr;
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap_in) &&
        p_mergeMatchedKeyFrame_in->getMap(p_mergeMatchedKFMap) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap3{};
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap_in ||
          p_mergeMatchedKFMap != p_mergeMap_in) &&
        p_atlas_inout->isActiveMap(p_currentMap_in, atlasIsActiveMap3) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap4{};
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap_in ||
          p_mergeMatchedKFMap != p_mergeMap_in || !atlasIsActiveMap3) &&
        p_atlas_inout->isActiveMap(p_mergeMap_in, atlasIsActiveMap4) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentKFIsBad2 || mergeMatchedKFIsBad2 ||
        p_currentKFMap != p_currentMap_in ||
        p_mergeMatchedKFMap != p_mergeMap_in || !atlasIsActiveMap3 ||
        !atlasIsActiveMap4)
    {
        resumeMergeWorkers(semanticUpdateLock_out, p_localMapper_inout);
        return false;
    }

    return true;
}

} /* namespace core */
} /* namespace vs_graphs */
