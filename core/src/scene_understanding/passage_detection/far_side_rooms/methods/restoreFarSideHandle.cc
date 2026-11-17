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
 * @file            restoreFarSideHandle.cc
 *
 * @brief           Implements SemanticsManager::restoreFarSideHandle(),
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

SemanticsManagerStatus SemanticsManager::restoreFarSideHandle(
    const std::vector<semantic::Room *> &allRooms_in,
    semantic::Passage                   *p_passage_inout,
    PassageRoomLinkState                &linkState_inout)
{
    /*!
     * A confirmed room that lists this passage among its doorways and
     * whose centroid lies on the side opposite the known side is the
     * passage's far-side room, even after a rule elsewhere cleared the
     * handle. Without the handle the aperture backstop cannot route walls
     * seen through the opening, and the camera room takes them while the
     * camera stands in the doorway. Only a single such room restores it,
     * and never one the stale-handle rule above clears, so the two rules
     * cannot alternate.
     */
    semantic::Room *p_farSideHandle = nullptr;
    if (p_passage_inout->getProspectiveRoom(p_farSideHandle) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_farSideHandle == nullptr &&
        std::abs(linkState_inout.knownSideSign) >= 1e-8)
    {
        semantic::Room *p_doorwayFarSideRoom    = nullptr;
        std::size_t     doorwayFarSideRoomCount = 0U;
        for (semantic::Room *p_room : allRooms_in)
        {
            bool                             isCandidateRoomBad{true};
            semantic::Room::RoomVariant      candidateRoomVariant{};
            std::vector<semantic::Passage *> candidateRoomPassages{};
            Eigen::Vector3d                  candidateCentroid_world_m{};
            const bool                       isClearedByStaleRule =
                (linkState_inout.confirmedAssociatedRoomCount == 1U &&
                 p_room == linkState_inout.p_confirmedAssociatedRoom &&
                 !linkState_inout.isSoleRoomOnFarSide) ||
                (linkState_inout.confirmedAssociatedRoomCount == 2U &&
                 p_room != linkState_inout.p_negativeSideRoom &&
                 p_room != linkState_inout.p_positiveSideRoom);
            if (p_room == nullptr ||
                p_room == linkState_inout.knownSide.p_room ||
                isClearedByStaleRule ||
                p_room->isBad(isCandidateRoomBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                isCandidateRoomBad ||
                p_room->getRoomVariant(candidateRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                candidateRoomVariant ==
                    semantic::Room::RoomVariant::UNDEFINED ||
                p_room->getPassages(candidateRoomPassages) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                std::find(candidateRoomPassages.begin(),
                          candidateRoomPassages.end(),
                          p_passage_inout) == candidateRoomPassages.end() ||
                p_room->getCentroid(candidateCentroid_world_m) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                continue;
            }
            const double candidateSide_m =
                linkState_inout.passageEquation_world.head<3>().dot(
                    candidateCentroid_world_m) +
                linkState_inout.passageEquation_world(3);
            if (candidateSide_m * linkState_inout.knownSideSign < 0.0)
            {
                p_doorwayFarSideRoom = p_room;
                doorwayFarSideRoomCount++;
            }
        }
        semantic::Room *p_restoredHandle = nullptr;
        if (doorwayFarSideRoomCount == 1U &&
            (p_passage_inout->setProspectiveRoom(p_doorwayFarSideRoom) !=
                 semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
             p_passage_inout->getProspectiveRoom(p_restoredHandle) !=
                 semantic::PassageStatus::PASSAGE_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom or getProspectiveRoom "
                         "returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        if (p_restoredHandle != nullptr)
        {
            int restoredPassageId{};
            int restoredRoomId{};
            if (p_passage_inout->getId(restoredPassageId) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                p_restoredHandle->getId(restoredRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                        "[SemMgr] Passage#%d far-side handle restored to "
                        "Room#%d: the room lists the passage and lies on "
                        "the far side.",
                        restoredPassageId,
                        restoredRoomId);
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
