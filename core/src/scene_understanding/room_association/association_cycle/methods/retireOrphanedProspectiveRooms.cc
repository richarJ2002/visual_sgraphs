/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            retireOrphanedProspectiveRooms.cc
 *
 * @brief           Implements the method
 *                  SemanticsManager::retireOrphanedProspectiveRooms(),
 *                  declared in SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::retireOrphanedProspectiveRooms(void)
{
    /* PROSPECTIVE ROOM CLEANUP
     * A passage reference is the stable far-side handle. Zero-wall
     * prospectives remain alive while that reference and its geometry
     * are valid; only orphaned or invalid handles are retired. */
    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long activeMapId{};
    if (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::semantic::Room *> candidateRooms{};
    if (getMapCandidateRoomsInIdOrder(activeMapId, candidateRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapCandidateRoomsInIdOrder returned a "
                     "failure status; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_candidate : candidateRooms)
    {
        if (p_candidate == nullptr)
        {
            continue;
        }

        int roomId{};
        if (p_candidate->getId(roomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const bool isTrackedProspective =
            prospectiveRoomCycles.count(roomId) > 0U;

        std::vector<vs_graphs::core::semantic::Passage *> referencingPassages;
        for (vs_graphs::core::semantic::Passage *p_passage : allPassages)
        {
            bool passageIsBad{};
            if ((p_passage != nullptr) &&
                p_passage->isBad(passageIsBad) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::semantic::Room *p_passageProspectiveRoom = nullptr;
            if ((p_passage != nullptr && !passageIsBad) &&
                p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage != nullptr && !passageIsBad &&
                p_passageProspectiveRoom == p_candidate)
            {
                referencingPassages.push_back(p_passage);
            }
        }

        if (!isTrackedProspective && referencingPassages.empty())
        {
            continue;
        }

        bool candidateIsBad{};
        if (p_candidate->isBad(candidateIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3d candidateCentroid{};
        if ((!candidateIsBad) && p_candidate->getCentroid(candidateCentroid) !=
                                     semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getCentroid returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        map::Map *p_candidateMap2 = nullptr;
        if ((!candidateIsBad && candidateCentroid.allFinite()) &&
            p_candidate->getMap(p_candidateMap2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_candidateMap3 = nullptr;
        if ((!candidateIsBad && candidateCentroid.allFinite() &&
             p_candidateMap2 != nullptr) &&
            p_candidate->getMap(p_candidateMap3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool atlasIsActiveMap{};
        if ((!candidateIsBad && candidateCentroid.allFinite() &&
             p_candidateMap2 != nullptr) &&
            p_atlas->isActiveMap(p_candidateMap3, atlasIsActiveMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isActiveMap returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        bool hasValidGeometry = !candidateIsBad &&
                                candidateCentroid.allFinite() &&
                                p_candidateMap2 != nullptr && atlasIsActiveMap;

        for (vs_graphs::core::semantic::Passage *p_passage :
             referencingPassages)
        {
            g2o::Plane3D passageGlobalEquation{};
            if (p_passage->getGlobalEquation(passageGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector4d passageEquation_world =
                passageGlobalEquation.coeffs();

            Eigen::Vector3d passageCentroid{};
            if (p_passage->getCentroid(passageCentroid) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!passageCentroid.allFinite() ||
                !passageEquation_world.allFinite() ||
                passageEquation_world.head<3>().norm() < 1e-8)
            {
                hasValidGeometry = false;
                break;
            }
        }

        bool candidateIsBad2{};
        if (p_candidate->isBad(candidateIsBad2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!candidateIsBad2 && !referencingPassages.empty() &&
            hasValidGeometry)
        {
            /* Wall count and age are deliberately irrelevant here. */
            prospectiveRoomCycles[roomId] = 0;
            continue;
        }

        for (vs_graphs::core::semantic::Passage *p_passage :
             referencingPassages)
        {
            if (p_passage->setProspectiveRoom(nullptr) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int clearedPassageId{};
            if (p_passage->getId(clearedPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                        "[SemMgr] Passage#%d far-side handle Room#%d "
                        "cleared: prospective retired (bad room or "
                        "invalid passage geometry).",
                        clearedPassageId,
                        roomId);
        }

        bool candidateIsBad3{};
        if (p_candidate->isBad(candidateIsBad3) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!candidateIsBad3)
        {
            map::Map *p_candidateMap = nullptr;
            if (p_candidate->getMap(p_candidateMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateMap != nullptr)
            {
                unsigned long candidateMapId{};
                if (p_candidateMap->getId(candidateMapId) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (eraseRoom(candidateMapId, p_candidate) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: eraseRoom returned a failure "
                                 "status; continuing as before.",
                                 __func__);
                }
            }
            if (p_candidate->setBad() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (loggedRoomCleanupIds.insert(roomId).second)
            {
                std::cout << "[SemMgr] Cleaning up orphaned "
                             "prospective Room#"
                          << roomId
                          << " (no active passage reference or invalid "
                             "geometry)."
                          << std::endl;
            }
        }

        prospectiveRoomCycles.erase(roomId);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
