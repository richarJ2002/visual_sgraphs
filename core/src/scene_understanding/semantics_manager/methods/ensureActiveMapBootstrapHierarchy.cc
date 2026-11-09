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
 * @file            ensureActiveMapBootstrapHierarchy.cc
 *
 * @brief           Implements
 *                  SemanticsManager::ensureActiveMapBootstrapHierarchy(),
 *                  declared in SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::ensureActiveMapBootstrapHierarchy(
    SemanticsManager::ActiveMapBootstrapResult &bootstrapResult_out,
    const std::optional<Eigen::Vector3d> &cameraPositionOverride_world_m_in)
{
    map::Map *p_atlasCurrentMap = nullptr;
    if ((p_atlas != nullptr) && p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_activeMap = p_atlas != nullptr ? p_atlasCurrentMap : nullptr;
    if (p_activeMap == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"reason\":\"NO_ACTIVE_MAP\",\"semantic_cycle\":"
                  << pipelineSemanticCycle << "}" << std::endl;
        bootstrapResult_out = ActiveMapBootstrapResult::NO_ACTIVE_MAP;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<semantic::Room *> activeRooms{};
    unsigned long                 activeMapId{};
    if (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (getMapRoomsInIdOrder(activeMapId, activeRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    const auto resolveLiveRoomById =
        [&activeRooms](const int roomId_in) -> semantic::Room *
    {
        if (roomId_in < 0)
        {
            return nullptr;
        }
        for (semantic::Room *p_room : activeRooms)
        {
            bool roomIsBad{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant roomVariant{};
            if ((p_room != nullptr && !roomIsBad) &&
                p_room->getRoomVariant(roomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int roomId{};
            if ((p_room != nullptr && !roomIsBad &&
                 roomVariant == semantic::Room::RoomVariant::ROOM) &&
                p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad &&
                roomVariant == semantic::Room::RoomVariant::ROOM &&
                roomId == roomId_in)
            {
                return p_room;
            }
        }
        return nullptr;
    };

    int currentRoomIdSnapshot = -1;
    {
        std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
        currentRoomIdSnapshot = currentRoomId;
    }
    int recoveryRoomId{};
    if (p_atlas->getCurrentSemanticRoomIdentity(recoveryRoomId) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    semantic::Room *p_bootstrapRoom =
        resolveLiveRoomById(currentRoomIdSnapshot);
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = resolveLiveRoomById(recoveryRoomId);
    }
    /* When a recovery identity exists (tracking-loss reset), never fall back
     * to an arbitrary lowest-ID live room: a spurious free-space SE# created
     * during the reset transient would otherwise hijack `currentRoomId` away
     * from the last-known hierarchy. The lowest-ID seed applies to cold start
     * only (no recovery identity). */
    if (p_bootstrapRoom == nullptr && recoveryRoomId < 0)
    {
        for (semantic::Room *p_room : activeRooms)
        {
            bool roomIsBad{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant roomVariant{};
            if ((p_room != nullptr && !roomIsBad) &&
                p_room->getRoomVariant(roomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int roomId2{};
            if ((p_room != nullptr && !roomIsBad &&
                 roomVariant == semantic::Room::RoomVariant::ROOM) &&
                !(p_bootstrapRoom == nullptr) &&
                p_room->getId(roomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int bootstrapRoomId{};
            if ((p_room != nullptr && !roomIsBad &&
                 roomVariant == semantic::Room::RoomVariant::ROOM) &&
                !(p_bootstrapRoom == nullptr) &&
                p_bootstrapRoom->getId(bootstrapRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad &&
                roomVariant == semantic::Room::RoomVariant::ROOM &&
                (p_bootstrapRoom == nullptr || roomId2 < bootstrapRoomId))
            {
                p_bootstrapRoom = p_room;
            }
        }
    }

    std::optional<semantic::RoomContextSnapshot> atlasRoomContext{};
    if ((p_bootstrapRoom == nullptr && recoveryRoomId >= 0) &&
        p_atlas->copyLatestRoomContext(recoveryRoomId, atlasRoomContext) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: copyLatestRoomContext returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<semantic::RoomContextSnapshot> recoveryContext =
        p_bootstrapRoom == nullptr && recoveryRoomId >= 0 ? atlasRoomContext
                                                          : std::nullopt;

    Eigen::Vector3d cameraPosition_world_m  = Eigen::Vector3d::Zero();
    bool            hasUsableCameraPosition = false;
    if (cameraPositionOverride_world_m_in.has_value() &&
        cameraPositionOverride_world_m_in->allFinite())
    {
        cameraPosition_world_m  = *cameraPositionOverride_world_m_in;
        hasUsableCameraPosition = true;
    }
    else
    {
        /* Refreshed at the start of this cycle; an exactly-zero centre is
         * never accepted there (refreshCurrentCameraCenter()): accepting it
         * misplaces the bootstrap room and poisons centroid-distance matching
         * for the cycles until walls correct it. Fall through to the snapshot
         * centroid, else yield and retry once poses exist. */
        if (hasCameraCenter && p_cameraCenterMap == p_activeMap)
        {
            cameraPosition_world_m  = currentCameraCenter_world_m;
            hasUsableCameraPosition = true;
        }
    }

    if (p_bootstrapRoom == nullptr && !hasUsableCameraPosition)
    {
        /* Same-map reset clears keyframes, so no camera pose exists yet while
         * a valid recovery snapshot does. Recreate the last-known hierarchy
         * at the snapshot centroid now (refined once keyframes return) rather
         * than yielding the cycle to a free-space SE# with a fresh ID. */
        if (recoveryContext.has_value() &&
            recoveryContext->centroid.allFinite())
        {
            cameraPosition_world_m  = recoveryContext->centroid;
            hasUsableCameraPosition = true;
        }
        else
        {
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << activeMapId
                      << ",\"reason\":\"NO_USABLE_CAMERA_POSE\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle << "}" << std::endl;
            bootstrapResult_out =
                ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }

    bool initializedRoom = false;
    bool recoveredRoom   = false;
    if (p_bootstrapRoom == nullptr)
    {
        std::unique_ptr<semantic::Room> p_blankRoomCandidate{};
        if (utils::geo_sem_helpers::GeoSemHelpers::createBlankRoomCandidate(
                p_atlas,
                p_blankRoomCandidate,
                cameraPosition_world_m,
                recoveryContext.has_value()) !=
            utils::geo_sem_helpers::GeoSemHelpersStatus::
                GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: createBlankRoomCandidate returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        p_bootstrapRoom = p_blankRoomCandidate.get();
        if (p_bootstrapRoom == nullptr)
        {
            unsigned long activeMapId2{};
            if (p_activeMap->getId(activeMapId2) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << activeMapId2
                      << ",\"reason\":\"ROOM_CREATION_FAILED\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle << "}" << std::endl;
            bootstrapResult_out =
                ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
        if (addCandidateRoom(activeMapId, std::move(p_blankRoomCandidate)) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addCandidateRoom returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        if (promoteCandidateRoom(activeMapId, p_bootstrapRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: promoteCandidateRoom returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        if (p_bootstrapRoom->setRoomVariant(
                semantic::Room::RoomVariant::ROOM) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int bootstrapRoomId2{};
        if (p_bootstrapRoom->getId(bootstrapRoomId2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_bootstrapRoom->setName("Room#" +
                                     std::to_string(bootstrapRoomId2)) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_bootstrapRoom->setBoundaryStatus(
                semantic::Room::BoundaryStatus::UNOBSERVED) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (recoveryContext.has_value())
        {
            if (p_bootstrapRoom->setPreviouslyVisited(
                    recoveryContext->wasPreviouslyVisited) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setPreviouslyVisited returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        initializedRoom = !recoveryContext.has_value();
        recoveredRoom   = recoveryContext.has_value();
    }

    std::vector<semantic::Floor *> floors{};
    if (getMapFloorsInIdOrder(activeMapId, floors) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapFloorsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    semantic::Floor *p_canonicalFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(floors, p_canonicalFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_canonicalFloor == nullptr)
    {
        if (utils::geo_sem_helpers::GeoSemHelpers::createMapFloor(p_atlas) !=
            utils::geo_sem_helpers::GeoSemHelpersStatus::
                GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Floor *> activeMapAllFloors{};
        if (getMapFloorsInIdOrder(activeMapId, activeMapAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapFloorsInIdOrder returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        floors                       = activeMapAllFloors;
        semantic::Floor *p_bestFloor = nullptr;
        if (semantic::Floor::selectBestObservedFloor(floors, p_bestFloor) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: selectBestObservedFloor returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        p_canonicalFloor = p_bestFloor;
    }
    if (p_canonicalFloor == nullptr)
    {
        int bootstrapRoomId4{};
        if (p_bootstrapRoom->getId(bootstrapRoomId4) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long activeMapId3{};
        if (p_activeMap->getId(activeMapId3) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << activeMapId3
                  << ",\"reason\":\"FLOOR_CREATION_FAILED\","
                     "\"room_id\":"
                  << bootstrapRoomId4
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        bootstrapResult_out = ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (p_canonicalFloor->addRoom(p_bootstrapRoom) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addRoom returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (resolveLiveRoomById(currentRoomIdSnapshot) == nullptr)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
            int                         bootstrapRoomId5{};
            if (p_bootstrapRoom->getId(bootstrapRoomId5) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            currentRoomId = bootstrapRoomId5;
            int bootstrapRoomId6{};
            if (p_bootstrapRoom->getId(bootstrapRoomId6) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_atlas->setCurrentSemanticRoomIdentity(bootstrapRoomId6) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setCurrentSemanticRoomIdentity returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        /* The UAV starts inside the bootstrap room: presence evidences entry.
         * Marked outside the current-room lock; the room owns its mutex. */
        if (p_bootstrapRoom->setPreviouslyVisited(true) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPreviouslyVisited returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        /* Mission-chain trace: the room this map started with. Set once;
         * later bootstrap cycles must not overwrite it. */
        int activeMapStartingRoomId = map::Map::NO_LINKED_ROOM_ID;
        if (p_activeMap->getStartingRoomId(activeMapStartingRoomId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getStartingRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (activeMapStartingRoomId == map::Map::NO_LINKED_ROOM_ID)
        {
            int bootstrapStartingRoomId{};
            if ((p_bootstrapRoom->getId(bootstrapStartingRoomId) !=
                 semantic::RoomStatus::ROOM_STATUS_SUCCESS) ||
                (p_activeMap->setStartingRoomId(bootstrapStartingRoomId) !=
                 map::MapStatus::MAP_STATUS_SUCCESS))
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId/setStartingRoomId returned a failure "
                             "status although they cannot fail; continuing as "
                             "before.",
                             __func__);
            }
        }
    }

    std::size_t restoredPassageCount = 0U;
    if (recoveredRoom)
    {
        for (const semantic::PassageContext &passageContext :
             recoveryContext->passageContexts)
        {
            semantic::Passage *p_recoveryPassage = nullptr;
            if (getMapPassageById(activeMapId,
                                  passageContext.id,
                                  p_recoveryPassage) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPassageById returned a failure status; "
                             "continuing as before.",
                             __func__);
            }
            if (p_recoveryPassage == nullptr)
            {
                /* New object, fresh ID: position, orientation, and aperture
                 * dimensions are not knowable across a map break, so only
                 * frame-free state (passable, traversal history, live links
                 * below) is restored. A zero-sized aperture shrinks geometric
                 * tests to their margin sliver, as before this change. */
                std::unique_ptr<semantic::Passage> p_recoveryPassageOwner =
                    std::make_unique<semantic::Passage>();
                p_recoveryPassage = p_recoveryPassageOwner.get();
                int recoveryPassageId{};
                if (p_atlas->reservePassageIdentity(recoveryPassageId) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
                    p_recoveryPassage->setId(recoveryPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: reservePassageIdentity/setId returned a "
                                 "failure status although they cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (p_recoveryPassage->setMap(p_activeMap) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_recoveryPassage->setPassable(passageContext.isPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPassable returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_recoveryPassage->setPassageType(
                        semantic::Passage::PassageVariant::DOORWAY) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPassageType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_recoveryPassage->setRecoveryProxy(true) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setRecoveryProxy returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalKnownToFarCount;
                     ++observationIndex)
                {
                    if (p_recoveryPassage->addTraversalObservation(
                            semantic::Passage::TraversalDirection::
                                KNOWN_TO_FAR) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        /*!
                         * addTraversalObservation cannot fail; continue as
                         * before.
                         */
                    }
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalFarToKnownCount;
                     ++observationIndex)
                {
                    if (p_recoveryPassage->addTraversalObservation(
                            semantic::Passage::TraversalDirection::
                                FAR_TO_KNOWN) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        /*!
                         * addTraversalObservation cannot fail; continue as
                         * before.
                         */
                    }
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalUnknownCount;
                     ++observationIndex)
                {
                    if (p_recoveryPassage->addTraversalObservation(
                            semantic::Passage::TraversalDirection::UNKNOWN) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        /*!
                         * addTraversalObservation cannot fail; continue as
                         * before.
                         */
                    }
                }
                if (addPassage(activeMapId,
                               std::move(p_recoveryPassageOwner)) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addPassage returned a failure status; "
                                 "continuing as before.",
                                 __func__);
                }
            }

            /* Context room ids are the recovered room's previous identity. */
            if (passageContext.hasKnownSideRoom &&
                passageContext.knownSideRoomId == recoveryContext->roomId)
            {
                if (p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            if (passageContext.hasFarSideRoom &&
                passageContext.secondaryRoomId == recoveryContext->roomId)
            {
                if (p_recoveryPassage->setProspectiveRoom(p_bootstrapRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            semantic::Passage::KnownSideProvenance
                recoveryPassageKnownSideProvenance{};
            if (p_recoveryPassage->getKnownSideProvenance(
                    recoveryPassageKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            vs_graphs::core::semantic::Room *p_recoveryPassageProspectiveRoom =
                nullptr;
            if ((recoveryPassageKnownSideProvenance.p_room == nullptr) &&
                p_recoveryPassage->getProspectiveRoom(
                    p_recoveryPassageProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (recoveryPassageKnownSideProvenance.p_room == nullptr &&
                p_recoveryPassageProspectiveRoom == nullptr)
            {
                if (p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            if (p_bootstrapRoom->setDoorways(p_recoveryPassage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            ++restoredPassageCount;
        }
    }

    if (initializedRoom)
    {
        int bootstrapRoomId9{};
        if (p_bootstrapRoom->getId(bootstrapRoomId9) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int canonicalFloorId{};
        if (p_canonicalFloor->getId(canonicalFloorId) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long activeMapId4{};
        if (p_activeMap->getId(activeMapId4) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << activeMapId4
                  << ",\"reason\":\"BOOTSTRAP_CREATED\",\"room_id\":"
                  << bootstrapRoomId9 << ",\"floor_id\":" << canonicalFloorId
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        bootstrapResult_out = ActiveMapBootstrapResult::INITIALIZED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (recoveredRoom)
    {
        int bootstrapRoomId10{};
        if (p_bootstrapRoom->getId(bootstrapRoomId10) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int canonicalFloorId2{};
        if (p_canonicalFloor->getId(canonicalFloorId2) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long activeMapId5{};
        if (p_activeMap->getId(activeMapId5) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << activeMapId5
                  << ",\"reason\":\"RECOVERY_RESTORED\",\"room_id\":"
                  << bootstrapRoomId10 << ",\"floor_id\":" << canonicalFloorId2
                  << ",\"restored_passages\":" << restoredPassageCount
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        bootstrapResult_out = ActiveMapBootstrapResult::RECOVERED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    bootstrapResult_out = ActiveMapBootstrapResult::REUSED;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
