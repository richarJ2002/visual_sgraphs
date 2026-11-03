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
 * @file            evaluateSemanticMergeGate.cc
 *
 * @brief           Implements evaluateSemanticMergeGate(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void evaluateSemanticMergeGate(
    map::Map                          *p_currentMap_in,
    map::Map                          *p_mergeMap_in,
    const g2o::Sim3                   &mapTransform_mergeWorldToCurrentWorld_in,
    semantic::SemanticMergeGateResult &gateResult_out)
{
    semantic::SemanticVerifyConfig configuration{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    gateResult_out = semantic::SemanticMergeGateResult{};
    if (semantic::SemanticVerify::evaluateMapMergeGate(
            p_currentMap_in,
            p_mergeMap_in,
            mapTransform_mergeWorldToCurrentWorld_in,
            gateResult_out,
            configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateMapMergeGate returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const char *p_name = nullptr;
    if (semantic::SemanticVerify::mergeDecisionName(gateResult_out.decision,
                                                    p_name) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mergeDecisionName returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const char *p_name2 = nullptr;
    if (semantic::SemanticVerify::mergeReasonName(gateResult_out.reason,
                                                  p_name2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mergeReasonName returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long currentMapId{};
    if (p_currentMap_in->getId(currentMapId) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long mergeMapId{};
    if (p_mergeMap_in->getId(mergeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemanticMergeGate] surviving_map=" << currentMapId
              << " absorbed_map=" << mergeMapId << " decision=" << p_name
              << " reason=" << p_name2
              << " shared_rooms=" << gateResult_out.sharedRoomCount
              << " aligned_rooms=" << gateResult_out.alignedRoomCount
              << " matched_walls=" << gateResult_out.matchedWallCount
              << " matched_passages=" << gateResult_out.matchedPassageCount
              << " committed=0"
              << formatMergeTransform(mapTransform_mergeWorldToCurrentWorld_in)
              << std::endl;
    std::cout << "[SemanticMergePairs] surviving_map=" << currentMapId
              << " absorbed_map=" << mergeMapId << " decision=" << p_name
              << " reason=" << p_name2 << " " << gateResult_out.pairDetail
              << std::endl;
}

} /* namespace core */
} /* namespace vs_graphs */
