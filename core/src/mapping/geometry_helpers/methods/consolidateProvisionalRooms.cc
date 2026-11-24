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
 * @file            consolidateProvisionalRooms.cc
 *
 * @brief           Implements Utils::consolidateProvisionalRooms(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::consolidateProvisionalRooms(
    vs_graphs::core::semantic::Room *p_selectedRoom_inout,
    atlas::Atlas                    *p_atlas_in)
{
    /* Confirm the selected room is valid */
    bool selectedRoom_inoutIsBad{};
    if (!(p_selectedRoom_inout == nullptr) &&
        p_selectedRoom_inout->isBad(selectedRoom_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_selectedRoom_inout == nullptr || selectedRoom_inoutIsBad)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Extract all walls assigned to the cluster-backed room */
    std::vector<vs_graphs::core::geometric::Plane *> selectedWalls{};
    if (p_selectedRoom_inout->getWalls(selectedWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Create a set containing the selected wall IDs */
    std::unordered_set<int> selectedWallIds;
    selectedWallIds.reserve(selectedWalls.size());

    /* Insert every valid selected wall ID into the set */
    for (vs_graphs::core::geometric::Plane *p_wall : selectedWalls)
    {
        bool wallIsBad{};
        if ((p_wall != nullptr) &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall != nullptr && !wallIsBad)
        {
            int wallGetId{};
            if (p_wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            selectedWallIds.insert(wallGetId);
        }
    }

    /* A room without walls cannot absorb another structural element */
    if (selectedWallIds.empty())
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* The rooms below come from the current map, which owns the room chain */
    map::Map *p_currentMap = nullptr;
    if (p_atlas_in->getCurrentMap(p_currentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    SemanticsManager *p_semanticsManager = nullptr;
    if (p_atlas_in->getSemanticsManager(p_semanticsManager) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getSemanticsManager returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    /* Extract all rooms and provisional structural elements */
    std::vector<vs_graphs::core::semantic::Room *> allRooms{};
    if (p_semanticsManager->getAllRoomsInIdOrder(allRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for the active map; "
            "continuing without its rooms.",
            __func__);
    }

    /* Iterate through every possible redundant structural element */
    for (vs_graphs::core::semantic::Room *p_candidateRoom : allRooms)
    {
        /* Skip invalid rooms and the selected room itself */
        bool candidateRoomIsBad{};
        if (!(p_candidateRoom == nullptr ||
              p_candidateRoom == p_selectedRoom_inout) &&
            p_candidateRoom->isBad(candidateRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_candidateRoom == nullptr ||
            p_candidateRoom == p_selectedRoom_inout || candidateRoomIsBad)
        {
            continue;
        }

        std::vector<vs_graphs::core::semantic::Passage *> activePassages{};
        if (p_semanticsManager->getAllPassagesInIdOrder(activePassages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the semantics manager has no scene graph for the "
                         "active map; continuing without its passages.",
                         __func__);
        }
        bool candidateIsLiveProspective = false;
        for (vs_graphs::core::semantic::Passage *p_passage : activePassages)
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
                p_passageProspectiveRoom == p_candidateRoom)
            {
                candidateIsLiveProspective = true;
                break;
            }
        }

        if (candidateIsLiveProspective)
        {
            continue;
        }
        /* Rooms across a passage or a wall's twin faces never merge. */
        bool areRoomsSeparated{};
        if (p_selectedRoom_inout->getIsSeparatedFrom(p_candidateRoom,
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

        /*!
         * Only automatically absorb undefined structural elements.
         * Classified rooms and corridors must never be merged automatically.
         */
        semantic::Room::RoomVariant candidateRoomRoomVariant{};
        if (p_candidateRoom->getRoomVariant(candidateRoomRoomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (candidateRoomRoomVariant !=
            vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the candidate room walls */
        std::vector<vs_graphs::core::geometric::Plane *> candidateWalls{};
        if (p_candidateRoom->getWalls(candidateWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Orphan-wall fallback creates one provisional SE per wall. Restrict
         * automatic consolidation to those single-wall provisional elements.
         * This prevents an emerging room on the opposite side of a shared wall
         * from being removed before it has enough evidence for classification.
         */
        std::vector<vs_graphs::core::geometric::Plane *> validCandidateWalls;
        validCandidateWalls.reserve(candidateWalls.size());

        for (vs_graphs::core::geometric::Plane *p_candidateWall :
             candidateWalls)
        {
            bool candidateWallIsBad{};
            if ((p_candidateWall != nullptr) &&
                p_candidateWall->isBad(candidateWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateWall != nullptr && !candidateWallIsBad)
            {
                validCandidateWalls.push_back(p_candidateWall);
            }
        }

        /* Only single-wall provisional elements are safe to absorb */
        if (validCandidateWalls.size() != 1)
        {
            continue;
        }

        /* Extract the single wall represented by the provisional element */
        vs_graphs::core::geometric::Plane *p_candidateWall =
            validCandidateWalls.front();

        /* The selected room must already contain the candidate wall */
        int candidateWallGetId{};
        if (p_candidateWall->getId(candidateWallGetId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (selectedWallIds.count(candidateWallGetId) == 0)
        {
            continue;
        }

        /*!
         * Confirm the candidate centroid is still close to its wall plane.
         * This identifies a wall-centred orphan SE rather than a free-space
         * cluster which may represent a genuine room on the opposite side.
         */
        g2o::Plane3D candidateWallGetGlobalEquation{};
        if (p_candidateWall->getGlobalEquation(
                candidateWallGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d wallEquation = candidateWallGetGlobalEquation.coeffs();

        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= normalNorm;

        Eigen::Vector3d candidateRoomCentroid{};
        if (p_candidateRoom->getCentroid(candidateRoomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const double candidatePlaneDistance =
            std::abs(wallEquation.head<3>().dot(candidateRoomCentroid) +
                     wallEquation(3));

        constexpr double provisionalWallDistanceThreshold = 0.25;

        if (candidatePlaneDistance > provisionalWallDistanceThreshold)
        {
            continue;
        }

        /* Preserve passage relationships before invalidating the candidate */
        std::vector<vs_graphs::core::semantic::Passage *> candidatePassages{};
        if (p_candidateRoom->getPassages(candidatePassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        for (vs_graphs::core::semantic::Passage *p_candidatePassage :
             candidatePassages)
        {
            /* Skip invalid passages */
            if (p_candidatePassage == nullptr)
            {
                continue;
            }

            /* Check whether the selected room already contains the passage */
            std::vector<vs_graphs::core::semantic::Passage *>
                selectedPassages{};
            if (p_selectedRoom_inout->getPassages(selectedPassages) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            bool alreadyPresent = false;
            for (vs_graphs::core::semantic::Passage *p_existingPassage :
                 selectedPassages)
            {
                int existingPassageId{};
                if ((p_existingPassage != nullptr) &&
                    p_existingPassage->getId(existingPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int candidatePassageId{};
                if ((p_existingPassage != nullptr) &&
                    p_candidatePassage->getId(candidatePassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_existingPassage != nullptr &&
                    existingPassageId == candidatePassageId)
                {
                    alreadyPresent = true;
                    break;
                }
            }

            /* Copy the passage relationship if required */
            if (!alreadyPresent)
            {
                if (p_selectedRoom_inout->addPassageAssociation(
                        p_candidatePassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addPassageAssociation returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
            }
        }

        /* Preserve floor membership before retiring the provisional room. */
        std::vector<semantic::Floor *> atlasAllFloors{};
        if (p_semanticsManager->getAllFloorsInIdOrder(atlasAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the semantics manager has no scene graph for the "
                         "active map; continuing without its floors.",
                         __func__);
        }
        for (vs_graphs::core::semantic::Floor *p_floor : atlasAllFloors)
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_candidateRoom,
                                         p_selectedRoom_inout,
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

        /* The selected room takes the candidate's key frames. */
        if (p_candidateRoom->transferKeyFrames(p_selectedRoom_inout) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: transferKeyFrames refused the candidate itself "
                         "as its survivor; continuing as before.",
                         __func__);
        }

        /* Repoint the map's room chain before the candidate is retired. */
        int linkFusedRoomId{};
        int linkKeptRoomId{};
        if ((p_candidateRoom->getId(linkFusedRoomId) !=
             semantic::RoomStatus::ROOM_STATUS_SUCCESS) ||
            (p_selectedRoom_inout->getId(linkKeptRoomId) !=
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

        /* Mark the redundant provisional structural element as invalid */
        unsigned long currentMapId{};
        if (p_currentMap->getId(currentMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_semanticsManager->markRoomBad(currentMapId, p_candidateRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: markRoomBad found no scene graph for map %lu; the "
                "room is flagged alone; continuing as before.",
                __func__,
                currentMapId);
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
