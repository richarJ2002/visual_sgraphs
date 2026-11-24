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
 * @file            moveImportedRoomWalls.cc
 *
 * @brief           Implements moveImportedRoomWalls(), declared in
 *                  graph/merging/private_functions.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void moveImportedRoomWalls(
    SemanticsManager                       *p_semanticsManager_inout,
    unsigned long                           mapId_in,
    semantic::Room                         *p_importedRoom_inout,
    semantic::Room                         *p_retainedRoom_inout,
    const std::vector<geometric::Plane *>  &importedWalls_in,
    const Eigen::Vector3d                  &retainedCentroid_world_m_in,
    const Eigen::Vector3d                  &mergeGroundNormal_world_in,
    double                                  mergeOpeningMargin_m_in,
    double                                  mergeMinimumSideDistance_m_in,
    const std::vector<semantic::Passage *> &mergePassages_in)
{
    struct WallTransfer
    {
        geometric::Plane  *p_wall;
        semantic::Room    *p_targetRoom;
        semantic::Passage *p_separatingPassage;
    };

    std::vector<WallTransfer> wallTransfers;
    wallTransfers.reserve(importedWalls_in.size());
    std::vector<semantic::Room *> mapRooms{};
    if (p_semanticsManager_inout->getMapRoomsInIdOrder(mapId_in, mapRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its rooms.",
            __func__,
            mapId_in);
    }
    std::sort(mapRooms.begin(),
              mapRooms.end(),
              semantic::isEntityIdLess<semantic::Room>);

    for (geometric::Plane *p_importedWall : importedWalls_in)
    {
        bool importedWallIsBad2{};
        if (!(p_importedWall == nullptr) &&
            p_importedWall->isBad(importedWallIsBad2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_importedWall == nullptr || importedWallIsBad2)
        {
            continue;
        }

        Eigen::Vector3d importedWallGetCentroid{};
        if (p_importedWall->getCentroid(importedWallGetCentroid) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d importedWallCentroid_world_m =
            importedWallGetCentroid.cast<double>();

        vs_graphs::core::semantic::Passage *p_separatingPassage = nullptr;

        for (vs_graphs::core::semantic::Passage *p_passage : mergePassages_in)
        {
            bool crossesOpening{};
            if ((p_passage != nullptr) &&
                crossesPassablePassageOpening(retainedCentroid_world_m_in,
                                              importedWallCentroid_world_m,
                                              p_passage,
                                              mergeGroundNormal_world_in,
                                              mergeOpeningMargin_m_in,
                                              mergeMinimumSideDistance_m_in,
                                              crossesOpening) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                /*!
                 * crossesPassablePassageOpening cannot fail; continue as
                 * before.
                 */
            }
            if (p_passage != nullptr && crossesOpening)
            {
                p_separatingPassage = p_passage;
                break;
            }
        }

        semantic::Room *p_targetRoom = p_retainedRoom_inout;

        /* Never introduce another owner when a distinct confirmed room
         * already owns this wall. The imported duplicate is retired below,
         * leaving that confirmed owner unchanged. */
        for (semantic::Room *p_existingOwner : mapRooms)
        {
            bool existingOwnerIsBad{};
            if (!(p_existingOwner == nullptr) &&
                p_existingOwner->isBad(existingOwnerIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant existingOwnerRoomVariant{};
            if (!(p_existingOwner == nullptr || existingOwnerIsBad ||
                  p_existingOwner == p_importedRoom_inout ||
                  p_existingOwner == p_retainedRoom_inout) &&
                p_existingOwner->getRoomVariant(existingOwnerRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getRoomVariant returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_existingOwner == nullptr || existingOwnerIsBad ||
                p_existingOwner == p_importedRoom_inout ||
                p_existingOwner == p_retainedRoom_inout ||
                existingOwnerRoomVariant != semantic::Room::RoomVariant::ROOM)
            {
                continue;
            }

            std::vector<geometric::Plane *> ownerWalls{};
            if (p_existingOwner->getWalls(ownerWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getWalls returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
            if (std::find(ownerWalls.begin(),
                          ownerWalls.end(),
                          p_importedWall) != ownerWalls.end())
            {
                p_targetRoom = p_existingOwner;
                break;
            }
        }

        if (p_separatingPassage != nullptr &&
            p_targetRoom == p_retainedRoom_inout)
        {
            vs_graphs::core::semantic::Room *p_farSideRoom = nullptr;
            if (p_separatingPassage->getProspectiveRoom(p_farSideRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            bool farSideRoomIsBad{};
            if (!(p_farSideRoom == nullptr) &&
                p_farSideRoom->isBad(farSideRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_farSideRoom == nullptr || farSideRoomIsBad ||
                p_farSideRoom == p_importedRoom_inout)
            {
                p_farSideRoom = nullptr;
            }

            p_targetRoom = p_farSideRoom;
        }

        wallTransfers.push_back(
            {p_importedWall, p_targetRoom, p_separatingPassage});
    }

    /* Commit the precomputed ownership transaction. Removing the duplicate
     * edge before adding its destination prevents transient double
     * ownership inside the authorized room fusion. */
    for (const WallTransfer &transfer : wallTransfers)
    {
        bool importedRoomWasWallRemoved{};
        if (p_importedRoom_inout->removeWall(transfer.p_wall,
                                             importedRoomWasWallRemoved) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            importedRoomWasWallRemoved = false;
            RCLCPP_WARN(
                rclcpp::get_logger("vs_graphs"),
                "%s: removeWall rejected its input; continuing as before.",
                __func__);
        }

        if (transfer.p_targetRoom == nullptr)
        {
            int id{};
            if (transfer.p_separatingPassage->getId(id) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int getId2{};
            if (transfer.p_wall->getId(getId2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemanticMerge] Far-side Wall#" << getId2
                      << " at Passage#" << id
                      << " has no prospective; left unbound." << std::endl;
            continue;
        }

        if (transfer.p_targetRoom->setWalls(transfer.p_wall) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (transfer.p_separatingPassage != nullptr &&
            transfer.p_targetRoom != p_retainedRoom_inout)
        {
            int id2{};
            if (transfer.p_targetRoom->getId(id2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int getId3{};
            if (transfer.p_wall->getId(getId3) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemanticMerge] Redirected far-side Wall#" << getId3
                      << " to stable Room#" << id2 << "." << std::endl;
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
