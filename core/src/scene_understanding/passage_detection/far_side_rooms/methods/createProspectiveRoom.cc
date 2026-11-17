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
 * @file            createProspectiveRoom.cc
 *
 * @brief           Implements SemanticsManager::createProspectiveRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::createProspectiveRoom(
    const std::vector<semantic::Passage *> &allPassages_in,
    semantic::Passage                      *p_passage_inout,
    PassageRoomLinkState                   &linkState_inout,
    const Eigen::Vector4d                  &passageEquation_world_in,
    const Eigen::Vector3d                  &passageCentroid_world_m_in,
    const Eigen::Vector3d                  &farSideNormal_world_in)
{
    /* Placeholder handle for "some room exists on the
     * far side of this doorway," not a real position
     * estimate -- it gets corrected the moment any real
     * far-side evidence (a wall, a cluster) arrives. Kept
     * close to the passage rather than out at a typical
     * room's centre depth so it doesn't visually or
     * spatially masquerade as a real room position in
     * the meantime. */
    constexpr double      estimatedRoomDepth_m = 0.15;
    const Eigen::Vector3d prospectiveCentroid =
        passageCentroid_world_m_in +
        farSideNormal_world_in * estimatedRoomDepth_m;

    /* SPATIAL DEDUPLICATION: Check if a
     * candidate/prospective room already exists near this
     * location (within 2.0m) across ALL passages. */
    bool      prospectiveExists = false;
    map::Map *p_activeMap       = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure "
                     "status although it cannot fail; "
                     "continuing as before.",
                     __func__);
    }
    unsigned long activeMapId{};
    if (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although "
                     "it cannot fail; continuing as before.",
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

    /* Count current prospective rooms (UNDEFINED variant
     * candidates) */
    int prospectiveRoomCount = 0;
    for (vs_graphs::core::semantic::Room *p_candidate : candidateRooms)
    {
        bool candidateIsBad{};
        if ((p_candidate != nullptr) &&
            p_candidate->isBad(candidateIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure "
                         "status although it cannot fail; "
                         "continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant candidateRoomVariant{};
        if ((p_candidate != nullptr && !candidateIsBad) &&
            p_candidate->getRoomVariant(candidateRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            /*!
             * getRoomVariant cannot fail; continue as
             * before.
             */
        }
        if (p_candidate != nullptr && !candidateIsBad &&
            candidateRoomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            prospectiveRoomCount++;
        }
    }

    /* Enforce max prospective rooms cap */
    bool passageIsPassable{};
    if ((prospectiveRoomCount >= kMaxProspectiveRooms) &&
        p_passage_inout->isPassable(passageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure "
                     "status although it cannot fail; "
                     "continuing as before.",
                     __func__);
    }
    bool passageTraversalEvidence{};
    if ((prospectiveRoomCount >= kMaxProspectiveRooms && !passageIsPassable) &&
        p_passage_inout->getTraversalEvidence(passageTraversalEvidence) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        /*!
         * getTraversalEvidence cannot fail; continue as
         * before.
         */
    }
    if (prospectiveRoomCount >= kMaxProspectiveRooms && !passageIsPassable &&
        !passageTraversalEvidence && !linkState_inout.hadProspectiveHandle)
    {
        int passageId15{};
        if (p_passage_inout->getId(passageId15) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure "
                         "status although it cannot fail; "
                         "continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Max prospective rooms (" << kMaxProspectiveRooms
                  << ") reached; skipping creation for "
                     "Passage#"
                  << passageId15 << std::endl;
    }
    else
    {
        if (findReusableProspectiveRoom(candidateRooms,
                                        allPassages_in,
                                        p_passage_inout,
                                        passageEquation_world_in,
                                        passageCentroid_world_m_in,
                                        prospectiveCentroid,
                                        prospectiveExists) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findReusableProspectiveRoom returned a failure "
                         "status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (!prospectiveExists)
        {
            /* Create the prospective room */
            std::unique_ptr<semantic::Room> p_prospectiveRoomOwner{};
            if (utils::geo_sem_helpers::GeoSemHelpers::createBlankRoomCandidate(
                    p_atlas,
                    p_prospectiveRoomOwner,
                    prospectiveCentroid) !=
                utils::geo_sem_helpers::GeoSemHelpersStatus::
                    GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createBlankRoomCandidate returned "
                             "a failure status although it cannot "
                             "fail; continuing as before.",
                             __func__);
            }

            vs_graphs::core::semantic::Room *p_prospectiveRoom =
                p_prospectiveRoomOwner.get();
            if (p_prospectiveRoom != nullptr)
            {
                /* Mark as provisional - will be promoted
                 * when walls are observed */
                if (p_prospectiveRoom->setRoomVariant(
                        vs_graphs::core::semantic::Room::RoomVariant::
                            UNDEFINED) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * setRoomVariant cannot fail; continue
                     * as before.
                     */
                }
                int prospectiveRoomId{};
                if (p_prospectiveRoom->getId(prospectiveRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getId cannot fail; continue as
                     * before.
                     */
                }
                if (p_prospectiveRoom->setName(
                        "Prospective#" + std::to_string(prospectiveRoomId)) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * setName cannot fail; continue as
                     * before.
                     */
                }

                /* Add to atlas as a candidate (not yet a
                 * confirmed room) */
                if (addCandidateRoom(activeMapId,
                                     std::move(p_prospectiveRoomOwner)) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addCandidateRoom returned a "
                                 "failure status; continuing as "
                                 "before.",
                                 __func__);
                }

                /* Link passage <-> prospective room */
                if (p_passage_inout->setProspectiveRoom(p_prospectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: setProspectiveRoom returned a "
                                 "failure status although it cannot "
                                 "fail; continuing as before.",
                                 __func__);
                }
                if (p_prospectiveRoom->addPassageAssociation(p_passage_inout) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * addPassageAssociation cannot fail; continue as
                     * before.
                     */
                }

                /* Register the live passage-created handle.
                 */
                int prospectiveRoomId2{};
                if (p_prospectiveRoom->getId(prospectiveRoomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getId cannot fail; continue as
                     * before.
                     */
                }
                prospectiveRoomCycles[prospectiveRoomId2] = 0;

                int prospectiveRoomId3{};
                if (p_prospectiveRoom->getId(prospectiveRoomId3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getId cannot fail; continue as
                     * before.
                     */
                }
                int passageId17{};
                if (p_passage_inout->getId(passageId17) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    /*!
                     * getId cannot fail; continue as
                     * before.
                     */
                }
                std::cout << "[SemMgr] Created prospective "
                             "Room#"
                          << prospectiveRoomId3 << " at "
                          << prospectiveCentroid.transpose() << " for Passage#"
                          << passageId17
                          << " (total prospective: " << prospectiveRoomCount + 1
                          << ")" << std::endl;
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
