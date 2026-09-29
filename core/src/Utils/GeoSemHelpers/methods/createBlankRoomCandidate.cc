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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::createBlankRoomCandidate(
    vs_graphs::core::Atlas           *p_atlas_inout,
    vs_graphs::core::semantic::Room *&p_blankRoomCandidate_out,
    Eigen::Vector3d                   centroid_in,
    std::optional<int>                stableRoomId_in)
{
    /* Confirm that the p_atlas_inout is valid */
    if (p_atlas_inout == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create room: Atlas is null."
                  << std::endl;

        p_blankRoomCandidate_out = nullptr;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Extract the existing rooms from the map */
    const std::vector<vs_graphs::core::semantic::Room *> existingRooms =
        p_atlas_inout->getAllRooms();

    /*!
     * Hard invariant, enforced at this single room-creation choke point
     * (this is the only call site in the codebase that ever constructs a
     * new vs_graphs::core::semantic::Room): a map may hold at most one more
     * room than it has PASSABLE passages. A map's first room is either the
     * mission's cold bootstrap or the topology-only recovery proxy restored
     * after tracking loss; it needs no active-map passage yet -- that is the
     * "+1". Every new semantic room after that must be the confirmed or
     * prospective far side of a genuine passage.
     *
     * Deliberately counts isPassable() passages only, not every registered
     * Passage object: a Passage can also be created "blocked" purely from a
     * classified door plane sitting near a wall (detectDoorsAndDoorways(),
     * GeoSemHelpers.cc's createMapPassage() called with isOpenPassage_in =
     * false) -- no free-space evidence at all. Counting that toward the
     * budget would let a semantic door classification alone unlock a new
     * room the same way real passage evidence does, which is exactly the
     * loophole this gate exists to close. A blocked passage earns its
     * budget slot only once it is actually observed passable (Connected
     * ESDF free space through the wall -- see createMapPassage()'s
     * isOpenPassage_in = true path, reached only from
     * updatePassages()'s skeleton-crossing candidates), matching this
     * project's rule: a room may only be created from a genuine
     * free-space-skeleton-crosses-wall observation, never a toggled
     * passable/blocked state alone.
     */
    const std::vector<vs_graphs::core::semantic::Passage *> currentMapPassages =
        p_atlas_inout->getAllPassages();
    const std::size_t passablePassageCount = std::count_if(
        currentMapPassages.begin(),
        currentMapPassages.end(),
        [](vs_graphs::core::semantic::Passage *p_passage)
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
            bool passageIsPassable{};
            if ((p_passage != nullptr && !passageIsBad) &&
                p_passage->isPassable(passageIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            return p_passage != nullptr && !passageIsBad && passageIsPassable;
        });

    /* Recovery (explicit stable ID) restores an already-discovered identity
     * after a tracking-loss reset; it is not new discovery and must not be
     * blocked by the passage budget. The budget gates discovery only. */
    if (!stableRoomId_in.has_value() &&
        existingRooms.size() > passablePassageCount)
    {
        std::cerr << "[GeoSemHelper] Refusing to create a new room: "
                  << existingRooms.size() << " room(s) already exist against "
                  << passablePassageCount
                  << " passable passage(s) in this map -- room count may "
                     "never exceed passable-passage count + 1."
                  << std::endl;

        p_blankRoomCandidate_out = nullptr;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    const int roomId = stableRoomId_in.has_value()
                           ? *stableRoomId_in
                           : p_atlas_inout->reserveRoomIdentity();
    p_atlas_inout->observeRoomIdentity(roomId);

    /* Create new room */
    vs_graphs::core::semantic::Room *p_newRoom =
        new vs_graphs::core::semantic::Room();

    /*!
     * Fill the parameters of room. The caller is responsible for inserting it
     * with:
     *      p_atlas_inout->AddCandidateMapRoom(newRoom);
     */

    if (p_newRoom->setId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newRoom->setCentroid(centroid_in) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newRoom->setMap(p_atlas_inout->getCurrentMap()) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (p_newRoom->setName("SE#" + std::to_string(roomId)) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setName returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (p_newRoom->setRoomVariant(
            vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRoomVariant returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    int newRoomId{};
    if (p_newRoom->getId(newRoomId) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d newRoomCentroid{};
    if (p_newRoom->getCentroid(newRoomCentroid) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "[GeoSemHelper] Created provisional SE#" << newRoomId << " at "
              << newRoomCentroid.transpose() << "." << std::endl;

    p_blankRoomCandidate_out = p_newRoom;
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
