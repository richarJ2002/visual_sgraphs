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
 * @file            attemptConsecutiveMergeIfGated.cc
 *
 * @brief           Implements Atlas::attemptConsecutiveMergeIfGated(), declared
 *                  in Atlas.h.
 */

#include "Atlas.h"

#include "Semantic/SemanticVerify.h"
#include "Types/objects/SystemParams.h"
#include "Utils/Utils/objects/Utils.h"

#include "../private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::attemptConsecutiveMergeIfGated(void)
{
    /* LOCK ORDER: the caller holds semanticUpdateMutex for the whole call.
     * This method briefly takes atlasMutex for attempt-state bookkeeping,
     * and MergeMapPair takes both maps' mapUpdateMutex. No path in the
     * codebase acquires these in reverse, so the order
     * semantic-update -> atlas -> map-update is deadlock-free. */
    Map *p_currentMap = nullptr;
    if (getCurrentMap(p_currentMap) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool currentMapIsBad{};
    if (!(p_currentMap == nullptr) &&
        p_currentMap->isBad(currentMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap == nullptr || currentMapIsBad)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const unsigned int cooldown_s =
        p_params != nullptr ? p_params->mapMerge.mergeCooldown_s : 30U;
    const unsigned int minimumAnchors =
        p_params != nullptr ? p_params->mapMerge.minAnchorRooms : 2U;
    const unsigned int minimumRooms =
        p_params != nullptr ? p_params->mapMerge.minRoomsPerMap : 1U;
    const unsigned int minimumWalls =
        p_params != nullptr ? p_params->mapMerge.minWallsPerMap : 3U;
    semantic::SemanticVerify::MapMergeConfig mergeConfiguration{};
    if (semantic::SemanticVerify::mapMergeConfigFromSystemParams(
            mergeConfiguration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mapMergeConfigFromSystemParams returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<Map *> allMaps{};
    if (getAllMaps(allMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Map *p_oldMap : allMaps)
    {
        bool oldMapIsBad{};
        if (!(p_oldMap == nullptr || p_oldMap == p_currentMap) &&
            p_oldMap->isBad(oldMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_oldMap == nullptr || p_oldMap == p_currentMap || oldMapIsBad)
        {
            continue;
        }
        bool isMatch{};
        if (consecutiveSeedTagsMatch(p_oldMap, p_currentMap, isMatch) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: consecutiveSeedTagsMatch returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!isMatch)
        {
            continue;
        }
        std::size_t liveRooms{};
        if (countLiveRooms(p_oldMap, liveRooms) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t liveRooms2{};
        if (!(liveRooms < minimumRooms) &&
            countLiveRooms(p_currentMap, liveRooms2) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t liveWallPlanes{};
        if (!(liveRooms < minimumRooms || liveRooms2 < minimumRooms) &&
            countLiveWallPlanes(p_oldMap, liveWallPlanes) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveWallPlanes returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t liveWallPlanes2{};
        if (!(liveRooms < minimumRooms || liveRooms2 < minimumRooms ||
              liveWallPlanes < minimumWalls) &&
            countLiveWallPlanes(p_currentMap, liveWallPlanes2) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveWallPlanes returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t liveFloors{};
        if (!(liveRooms < minimumRooms || liveRooms2 < minimumRooms ||
              liveWallPlanes < minimumWalls ||
              liveWallPlanes2 < minimumWalls) &&
            countLiveFloors(p_oldMap, liveFloors) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveFloors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t liveFloors2{};
        if (!(liveRooms < minimumRooms || liveRooms2 < minimumRooms ||
              liveWallPlanes < minimumWalls || liveWallPlanes2 < minimumWalls ||
              liveFloors == 0U) &&
            countLiveFloors(p_currentMap, liveFloors2) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countLiveFloors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (liveRooms < minimumRooms || liveRooms2 < minimumRooms ||
            liveWallPlanes < minimumWalls || liveWallPlanes2 < minimumWalls ||
            liveFloors == 0U || liveFloors2 == 0U)
        {
            continue;
        }

        unsigned long oldMapIdValue{};
        if (p_oldMap->getId(oldMapIdValue) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const long unsigned int oldMapId =
            static_cast<long unsigned int>(oldMapIdValue);
        std::size_t contentHash{};
        if (consecutiveContentHash(p_oldMap, p_currentMap, contentHash) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: consecutiveContentHash returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now();
        MergeAttemptState attemptState;
        {
            std::unique_lock<std::mutex> atlasLock(atlasMutex);
            const std::map<unsigned long, MergeAttemptState>::iterator
                storedState = consecutiveMergeState.find(oldMapId);
            if (storedState != consecutiveMergeState.end())
            {
                attemptState = storedState->second;
            }
        }
        if (attemptState.hasEverAttempted)
        {
            if (now - attemptState.lastAttemptTime <
                std::chrono::seconds(cooldown_s))
            {
                continue;
            }
            if (attemptState.contentHashAtLastAttempt == contentHash)
            {
                continue;
            }
        }

        std::set<std::string> anchorTags{};
        if (collectAnchorTags(p_oldMap, p_currentMap, anchorTags) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: collectAnchorTags returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const std::size_t anchorCount   = anchorTags.size();
        auto              recordAttempt = [&](void)
        {
            attemptState.lastAttemptTime          = now;
            attemptState.contentHashAtLastAttempt = contentHash;
            attemptState.hasEverAttempted         = true;
            std::unique_lock<std::mutex> atlasLock(atlasMutex);
            consecutiveMergeState[oldMapId] = attemptState;
        };
        /* The seed room itself must be anchored: matching side rooms while
         * the prior-link room takes part nowhere would fuse on a
         * coincidental resemblance. Same DEFER as too few anchors. */
        semantic::Room *p_oldFinalRoom = nullptr;
        if (p_oldMap->getFinalRoom(p_oldFinalRoom) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFinalRoom returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        bool oldFinalRoomHasRoomTag{};
        if ((p_oldFinalRoom != nullptr) &&
            p_oldFinalRoom->hasRoomTag(oldFinalRoomHasRoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string oldFinalRoomRoomTag{};
        if ((p_oldFinalRoom != nullptr && oldFinalRoomHasRoomTag) &&
            p_oldFinalRoom->getRoomTag(oldFinalRoomRoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const bool isSeedAnchored = p_oldFinalRoom != nullptr &&
                                    oldFinalRoomHasRoomTag &&
                                    anchorTags.count(oldFinalRoomRoomTag) > 0U;
        if (anchorCount < minimumAnchors || !isSeedAnchored)
        {
            recordAttempt();
            unsigned long currentMapId{};
            if (p_currentMap->getId(currentMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << currentMapId
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "SHARED_ROOM_IDENTITY_MISSING\"}"
                      << std::endl;
            continue;
        }

        std::vector<Eigen::Vector3d> normalsCurrent, centroidsCurrent;
        std::vector<Eigen::Vector3d> normalsOld, centroidsOld;
        bool                         hasEnoughCorrespondences{};
        if (utils::utils::Utils::collectCorrespondingWalls(
                p_currentMap,
                p_oldMap,
                normalsCurrent,
                centroidsCurrent,
                normalsOld,
                centroidsOld,
                hasEnoughCorrespondences) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: collectCorrespondingWalls returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!hasEnoughCorrespondences)
        {
            recordAttempt();
            unsigned long currentMapId2{};
            if (p_currentMap->getId(currentMapId2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << currentMapId2
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "WALL_EVIDENCE_MISSING\"}"
                      << std::endl;
            continue;
        }
        Eigen::Isometry3d transformOldToCurrent{};
        if (utils::utils::Utils::computeMapTransform_Horn(
                normalsOld,
                centroidsOld,
                normalsCurrent,
                centroidsCurrent,
                transformOldToCurrent) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeMapTransform_Horn returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!transformOldToCurrent.matrix().allFinite())
        {
            recordAttempt();
            unsigned long currentMapId3{};
            if (p_currentMap->getId(currentMapId3) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << currentMapId3
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "WALL_EVIDENCE_MISSING\"}"
                      << std::endl;
            continue;
        }
        const g2o::Sim3 transformSim3(transformOldToCurrent.linear(),
                                      transformOldToCurrent.translation(),
                                      1.0);
        semantic::SemanticMergeGateResult gateResult{};
        if (semantic::SemanticVerify::evaluateConsecutiveMergeGate(
                p_currentMap,
                p_oldMap,
                transformSim3,
                gateResult,
                mergeConfiguration) !=
            semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateConsecutiveMergeGate returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        recordAttempt();
        const char *p_name = nullptr;
        if (semantic::SemanticVerify::mergeDecisionName(gateResult.decision,
                                                        p_name) !=
            semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeDecisionName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const char *p_name2 = nullptr;
        if (semantic::SemanticVerify::mergeReasonName(gateResult.reason,
                                                      p_name2) !=
            semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeReasonName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long currentMapId4{};
        if (p_currentMap->getId(currentMapId4) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_attempt\","
                     "\"old_map_id\":"
                  << oldMapId << ",\"new_map_id\":" << currentMapId4
                  << ",\"anchors\":" << anchorCount << ",\"decision\":\""
                  << p_name << "\",\"reason\":\"" << p_name2
                  << "\",\"matched_walls\":" << gateResult.matchedWallCount
                  << ",\"matched_passages\":" << gateResult.matchedPassageCount
                  << "}" << std::endl;
        if (gateResult.decision != semantic::SemanticMergeDecision::ACCEPT)
        {
            continue;
        }
        if (mergeMapPair(p_currentMap, p_oldMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeMapPair returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long currentMapId5{};
        if (p_currentMap->getId(currentMapId5) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                     "committed\",\"old_map_id\":"
                  << oldMapId << ",\"new_map_id\":" << currentMapId5 << "}"
                  << std::endl;
        /* One merge per call: the current map changed shape, so remaining
         * pairs re-evaluate from scratch next cycle. */
        break;
    }

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
