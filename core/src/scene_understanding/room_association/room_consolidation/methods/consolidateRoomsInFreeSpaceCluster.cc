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
 * @file            consolidateRoomsInFreeSpaceCluster.cc
 *
 * @brief           Implements
 *                  SemanticsManager::consolidateRoomsInFreeSpaceCluster(),
 *                  declared in SemanticsManager.h.
 */

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
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::consolidateRoomsInFreeSpaceCluster(
    vs_graphs::core::semantic::Room    *p_retainedRoom_inout,
    const std::vector<Eigen::Vector3d> &freeSpaceCluster_world_m_in,
    const std::vector<vs_graphs::core::geometric::Plane *> &wallList_world_in)
{
    map::Map *p_currentMap = nullptr;
    if (p_atlas->getCurrentMap(p_currentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    bool retainedRoom_inoutIsBad{};
    if (!(p_retainedRoom_inout == nullptr) &&
        p_retainedRoom_inout->isBad(retainedRoom_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_retainedRoom_inout == nullptr || retainedRoom_inoutIsBad ||
        p_currentMap == nullptr || freeSpaceCluster_world_m_in.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double maximumClusterSupportDistance_m =
        static_cast<double>(p_sysParams->roomSeg.centerDistanceThresh);
    const double finiteWallBoundsMargin_m =
        static_cast<double>(p_sysParams->roomSeg.finiteWallBoundsMargin_m);
    Eigen::Vector3d retainedCentroid_world_m{};
    if (p_retainedRoom_inout->getCentroid(retainedCentroid_world_m) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<semantic::Room *> currentMapAllRooms{};
    unsigned long                 currentMapId{};
    if (p_currentMap->getId(currentMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (getMapRoomsInIdOrder(currentMapId, currentMapAllRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_duplicateRoom : currentMapAllRooms)
    {
        bool duplicateRoomIsBad{};
        if (!(p_duplicateRoom == nullptr ||
              p_duplicateRoom == p_retainedRoom_inout) &&
            p_duplicateRoom->isBad(duplicateRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_duplicateRoom == nullptr ||
            p_duplicateRoom == p_retainedRoom_inout || duplicateRoomIsBad)
        {
            continue;
        }

        std::vector<semantic::Passage *> activePassages{};
        if (getAllPassagesInIdOrder(activePassages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassagesInIdOrder returned a failure "
                         "status; continuing as before.",
                         __func__);
        }
        semantic::Passage *p_duplicateSeedPassage = nullptr;
        for (semantic::Passage *p_passage : activePassages)
        {
            vs_graphs::core::semantic::Room *p_passageProspectiveRoom = nullptr;
            if ((p_passage != nullptr) &&
                p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage != nullptr &&
                p_passageProspectiveRoom == p_duplicateRoom)
            {
                p_duplicateSeedPassage = p_passage;
                break;
            }
        }
        const bool duplicateIsLiveProspective =
            p_duplicateSeedPassage != nullptr;

        /*!
         * A live prospective room waits for its own cluster validation,
         * except a wall-less seed standing inside this room's wall
         * footprint and on this room's side of its own passage plane: it is
         * the far-side handle of space this room already holds, and nothing
         * else can ever give it walls. The footprint is padded only by half
         * the thinnest wall (optimization.wall_thickness.minimum_thickness),
         * well below the seed's 0.15 m offset beyond its passage, so a seed
         * beyond the near-side room's wall never counts as inside it.
         */
        if (duplicateIsLiveProspective)
        {
            bool isSeedFusable{};
            if (isSeedFusableIntoRoom(p_retainedRoom_inout,
                                      p_duplicateRoom,
                                      p_duplicateSeedPassage,
                                      p_currentMap,
                                      retainedCentroid_world_m,
                                      isSeedFusable) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: isSeedFusableIntoRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!isSeedFusable)
            {
                continue;
            }
        }

        semantic::Room::RoomVariant retainedType{};
        if (p_retainedRoom_inout->getRoomVariant(retainedType) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant duplicateType{};
        if (p_duplicateRoom->getRoomVariant(duplicateType) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (retainedType != semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED &&
            retainedType != duplicateType)
        {
            continue;
        }

        Eigen::Vector3d duplicateCentroid_world_m{};
        if (p_duplicateRoom->getCentroid(duplicateCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const double centroidDistance_m =
            (duplicateCentroid_world_m - retainedCentroid_world_m).norm();

        double duplicateClusterDistance_m{};
        if (computeDistanceToCluster(freeSpaceCluster_world_m_in,
                                     duplicateCentroid_world_m,
                                     duplicateClusterDistance_m) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeDistanceToCluster returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /*
         * Do not impose a room-centroid separation limit here. One connected
         * free-space component can legitimately span a large office, and its
         * centroid moves as exploration expands. Membership in that component
         * plus the finite-wall veto is the relevant topological evidence.
         */
        const bool isDuplicateOutsideCluster =
            !duplicateCentroid_world_m.allFinite() ||
            !std::isfinite(centroidDistance_m) ||
            duplicateClusterDistance_m > maximumClusterSupportDistance_m;
        const bool isSeparatedByFiniteWall =
            !isDuplicateOutsideCluster &&
            semantic_constraints::hasSeparatingFiniteWall(
                wallList_world_in,
                retainedCentroid_world_m,
                duplicateCentroid_world_m,
                finiteWallBoundsMargin_m,
                false);
        if (isDuplicateOutsideCluster || isSeparatedByFiniteWall)
        {
            continue;
        }

        std::vector<semantic::Passage *> duplicatePassages{};
        if (p_duplicateRoom->getPassages(duplicatePassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Rooms across a passage (centres on its two sides) or a wall's twin
         * faces never fuse. */
        bool areRoomsSeparated{};
        if (p_retainedRoom_inout->getIsSeparatedFrom(p_duplicateRoom,
                                                     areRoomsSeparated) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getIsSeparatedFrom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (areRoomsSeparated)
        {
            continue;
        }

        /* Move the duplicate's walls; one rejected wall restores all */
        bool allDuplicateWallsWereAdmitted{};
        if (transferDuplicateRoomWalls(p_retainedRoom_inout,
                                       p_duplicateRoom,
                                       currentMapId,
                                       allDuplicateWallsWereAdmitted) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: transferDuplicateRoomWalls returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!allDuplicateWallsWereAdmitted)
        {
            continue;
        }

        for (semantic::Passage *p_passage : duplicatePassages)
        {
            if (p_retainedRoom_inout->addPassageAssociation(p_passage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: addPassageAssociation returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        geometric::Plane *p_retainedRoom_inoutGroundPlane = nullptr;
        if (p_retainedRoom_inout->getGroundPlane(
                p_retainedRoom_inoutGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_retainedRoom_inoutGroundPlane == nullptr)
        {
            geometric::Plane *p_duplicateRoomGroundPlane = nullptr;
            if (p_duplicateRoom->getGroundPlane(p_duplicateRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGroundPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedRoom_inout->setGroundPlane(
                    p_duplicateRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGroundPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        bool retainedRoom_inoutHasKnownLabel2{};
        if (p_retainedRoom_inout->getHasKnownLabel(
                retainedRoom_inoutHasKnownLabel2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool duplicateRoomHasKnownLabel2{};
        if ((!retainedRoom_inoutHasKnownLabel2) &&
            p_duplicateRoom->getHasKnownLabel(duplicateRoomHasKnownLabel2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!retainedRoom_inoutHasKnownLabel2 && duplicateRoomHasKnownLabel2)
        {
            if (p_retainedRoom_inout->setHasKnownLabel(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setHasKnownLabel returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::string duplicateRoomName{};
            if (p_duplicateRoom->getName(duplicateRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getName returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedRoom_inout->setName(duplicateRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setName returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }

        if (retainedType == semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED)
        {
            if (p_retainedRoom_inout->setRoomVariant(duplicateType) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        std::vector<semantic::Floor *> currentMapAllFloors{};
        if (getMapFloorsInIdOrder(currentMapId, currentMapAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapFloorsInIdOrder returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        for (semantic::Floor *p_floor : currentMapAllFloors)
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_duplicateRoom,
                                         p_retainedRoom_inout,
                                         floorWasRoomReplaced) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    floorWasRoomReplaced = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: replaceRoom rejected its input; "
                                "continuing as before.",
                                __func__);
                }
            }
        }

        if (eraseRoom(currentMapId, p_duplicateRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseRoom returned a failure status; continuing "
                         "as before.",
                         __func__);
        }
        if (p_duplicateRoom->clearWalls() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_duplicateRoom->transferKeyFrames(p_retainedRoom_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearWalls/transferKeyFrames returned a failure "
                         "status although they cannot fail; continuing.",
                         __func__);
        }
        if (p_duplicateRoom->clearPassages() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (markRoomBad(currentMapId, p_duplicateRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: markRoomBad found no scene graph for map %lu; the "
                "room is flagged alone; continuing as before.",
                __func__,
                currentMapId);
        }
        int linkFusedRoomId{};
        int linkKeptRoomId{};
        if ((p_duplicateRoom->getId(linkFusedRoomId) !=
             semantic::RoomStatus::ROOM_STATUS_SUCCESS) ||
            (p_retainedRoom_inout->getId(linkKeptRoomId) !=
             semantic::RoomStatus::ROOM_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->replaceLinkedRoom(linkFusedRoomId, linkKeptRoomId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: replaceLinkedRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        /* A fused seed's passage now leads to the retained room. */
        for (semantic::Passage *p_passage : activePassages)
        {
            bool passageWasRoomReplaced{};
            if (p_passage != nullptr &&
                p_passage->replaceProspectiveRoom(p_duplicateRoom,
                                                  p_retainedRoom_inout,
                                                  passageWasRoomReplaced) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: replaceProspectiveRoom rejected its input; "
                            "continuing as before.",
                            __func__);
            }
        }

        int duplicateRoomId{};
        if (p_duplicateRoom->getId(duplicateRoomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int retainedRoom_inoutId{};
        if (p_retainedRoom_inout->getId(retainedRoom_inoutId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Fused Room#" << duplicateRoomId << " into Room#"
                  << retainedRoom_inoutId
                  << " using connected free-space evidence (centroid distance "
                  << centroidDistance_m << " m)." << std::endl;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
