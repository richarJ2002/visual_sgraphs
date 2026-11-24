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
 * @file            selectMergeMaps.cc
 *
 * @brief           Implements selectMergeMaps(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool selectMergeMaps(keyframe::KeyFrame *p_currentKeyFrame_in,
                     keyframe::KeyFrame *p_mergeMatchedKeyFrame_in,
                     atlas::Atlas       *p_atlas_in,
                     map::Map          *&p_currentMap_out,
                     map::Map          *&p_mergeMap_out)
{
    p_currentMap_out = nullptr;
    p_mergeMap_out   = nullptr;

    /* Reject stale place-recognition candidates before stopping other workers.
     */
    bool currentKFIsBad{};
    if (!(p_currentKeyFrame_in == nullptr ||
          p_mergeMatchedKeyFrame_in == nullptr) &&
        p_currentKeyFrame_in->isBad(currentKFIsBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMatchedKFIsBad{};
    if (!(p_currentKeyFrame_in == nullptr ||
          p_mergeMatchedKeyFrame_in == nullptr || currentKFIsBad) &&
        p_mergeMatchedKeyFrame_in->isBad(mergeMatchedKFIsBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentKeyFrame_in == nullptr ||
        p_mergeMatchedKeyFrame_in == nullptr || currentKFIsBad ||
        mergeMatchedKFIsBad)
    {
        return false;
    }

    if (p_currentKeyFrame_in->getMap(p_currentMap_out) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_mergeMatchedKeyFrame_in->getMap(p_mergeMap_out) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    bool currentMapIsBad{};
    if (!(p_currentMap_out == nullptr || p_mergeMap_out == nullptr ||
          p_currentMap_out == p_mergeMap_out) &&
        p_currentMap_out->isBad(currentMapIsBad) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMapIsBad{};
    if (!(p_currentMap_out == nullptr || p_mergeMap_out == nullptr ||
          p_currentMap_out == p_mergeMap_out || currentMapIsBad) &&
        p_mergeMap_out->isBad(mergeMapIsBad) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap{};
    if (!(p_currentMap_out == nullptr || p_mergeMap_out == nullptr ||
          p_currentMap_out == p_mergeMap_out || currentMapIsBad ||
          mergeMapIsBad) &&
        p_atlas_in->isActiveMap(p_currentMap_out, atlasIsActiveMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap2{};
    if (!(p_currentMap_out == nullptr || p_mergeMap_out == nullptr ||
          p_currentMap_out == p_mergeMap_out || currentMapIsBad ||
          mergeMapIsBad || !atlasIsActiveMap) &&
        p_atlas_in->isActiveMap(p_mergeMap_out, atlasIsActiveMap2) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap_out == nullptr || p_mergeMap_out == nullptr ||
        p_currentMap_out == p_mergeMap_out || currentMapIsBad ||
        mergeMapIsBad || !atlasIsActiveMap || !atlasIsActiveMap2)
    {
        return false;
    }

    return true;
}

} /* namespace core */
} /* namespace vs_graphs */
