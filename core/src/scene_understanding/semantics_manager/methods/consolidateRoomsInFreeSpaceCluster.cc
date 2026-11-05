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

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantic/marker/marker.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

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

    const auto distanceToCluster_m =
        [&freeSpaceCluster_world_m_in](
            const Eigen::Vector3d &clusterPoint_world_m_in)
    {
        double minimumDistance_m = std::numeric_limits<double>::infinity();

        for (const Eigen::Vector3d &clusterPoint_world_m :
             freeSpaceCluster_world_m_in)
        {
            if (clusterPoint_world_m.allFinite())
            {
                minimumDistance_m = std::min(
                    minimumDistance_m,
                    (clusterPoint_world_m_in - clusterPoint_world_m).norm());
            }
        }

        return minimumDistance_m;
    };

    std::vector<semantic::Room *> currentMapAllRooms{};
    if (p_currentMap->getAllRooms(currentMapAllRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
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
        if (p_atlas->getAllPassages(activePassages) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool duplicateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [p_duplicateRoom](semantic::Passage *p_passage)
            {
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                    nullptr;
                if ((p_passage != nullptr) &&
                    p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                return p_passage != nullptr &&
                       p_passageProspectiveRoom == p_duplicateRoom;
            });

        if (duplicateIsLiveProspective)
        {
            continue;
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

        bool retainedRoom_inoutHasKnownLabel{};
        if (p_retainedRoom_inout->getHasKnownLabel(
                retainedRoom_inoutHasKnownLabel) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool duplicateRoomHasKnownLabel{};
        if ((retainedRoom_inoutHasKnownLabel) &&
            p_duplicateRoom->getHasKnownLabel(duplicateRoomHasKnownLabel) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int retainedRoom_inoutMetaMarkerId{};
        if ((retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel) &&
            p_retainedRoom_inout->getMetaMarkerId(
                retainedRoom_inoutMetaMarkerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMetaMarkerId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int duplicateRoomMetaMarkerId{};
        if ((retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel) &&
            p_duplicateRoom->getMetaMarkerId(duplicateRoomMetaMarkerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMetaMarkerId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel &&
            retainedRoom_inoutMetaMarkerId != duplicateRoomMetaMarkerId)
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

        /*
         * Do not impose a room-centroid separation limit here. One connected
         * free-space component can legitimately span a large office, and its
         * centroid moves as exploration expands. Membership in that component
         * plus the finite-wall veto is the relevant topological evidence.
         */
        const bool isDuplicateOutsideCluster =
            !duplicateCentroid_world_m.allFinite() ||
            !std::isfinite(centroidDistance_m) ||
            distanceToCluster_m(duplicateCentroid_world_m) >
                maximumClusterSupportDistance_m;
        bool hasSeparatingFiniteWall2{};
        if (!(isDuplicateOutsideCluster) &&
            hasSeparatingFiniteWall(wallList_world_in,
                                    retainedCentroid_world_m,
                                    duplicateCentroid_world_m,
                                    finiteWallBoundsMargin_m,
                                    false,
                                    hasSeparatingFiniteWall2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: hasSeparatingFiniteWall returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (isDuplicateOutsideCluster || hasSeparatingFiniteWall2)
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

        /* Rooms across a passage or a wall's twin faces never fuse. */
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

        std::vector<
            std::pair<semantic::Room *, std::vector<geometric::Plane *>>>
                                      wallSnapshots;
        std::vector<semantic::Room *> currentMapAllRooms2{};
        if (p_currentMap->getAllRooms(currentMapAllRooms2) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_snapshotRoom : currentMapAllRooms2)
        {
            bool snapshotRoomIsBad{};
            if ((p_snapshotRoom != nullptr) &&
                p_snapshotRoom->isBad(snapshotRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_snapshotRoom != nullptr && !snapshotRoomIsBad)
            {
                std::vector<geometric::Plane *> snapshotRoomWalls{};
                if (p_snapshotRoom->getWalls(snapshotRoomWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                wallSnapshots.emplace_back(p_snapshotRoom, snapshotRoomWalls);
            }
        }

        bool allDuplicateWallsWereAdmitted = true;

        std::vector<geometric::Plane *> duplicateRoomWalls{};
        if (p_duplicateRoom->getWalls(duplicateRoomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : duplicateRoomWalls)
        {
            bool wasAdmitted{};
            if (admitWallToRoom(p_retainedRoom_inout, p_wall, wasAdmitted) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: admitWallToRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!wasAdmitted)
            {
                allDuplicateWallsWereAdmitted = false;
                break;
            }
        }

        /*
         * A topology-rejected provisional wall is not a duplicate room. Keep
         * that structural element alive so another free-space component can
         * claim it instead of repeatedly deleting and recreating it.
         */
        if (!allDuplicateWallsWereAdmitted)
        {
            for (auto &[p_snapshotRoom, snapshotWalls] : wallSnapshots)
            {
                if (p_snapshotRoom->clearWalls() !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: clearWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (geometric::Plane *p_snapshotWall : snapshotWalls)
                {
                    if (p_snapshotRoom->setWalls(p_snapshotWall) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setWalls returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
            continue;
        }

        for (semantic::Passage *p_passage : duplicatePassages)
        {
            if (p_retainedRoom_inout->setDoorways(p_passage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
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
            semantic::Marker *p_duplicateRoomMetaMarker = nullptr;
            if (p_duplicateRoom->getMetaMarker(p_duplicateRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMetaMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedRoom_inout->setMetaMarker(
                    p_duplicateRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMetaMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int duplicateRoomMetaMarkerId2{};
            if (p_duplicateRoom->getMetaMarkerId(duplicateRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMetaMarkerId returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedRoom_inout->setMetaMarkerId(
                    duplicateRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMetaMarkerId returned a failure status "
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
        if (p_currentMap->getAllFloors(currentMapAllFloors) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
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

        if (p_currentMap->eraseDetectedMapRoom(p_duplicateRoom) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseDetectedMapRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->eraseMarkerBasedMapRoom(p_duplicateRoom) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: eraseMarkerBasedMapRoom returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_duplicateRoom->clearWalls() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
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
        if (p_duplicateRoom->setBad() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->replaceLinkedRoom(p_duplicateRoom,
                                            p_retainedRoom_inout) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: replaceLinkedRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
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
