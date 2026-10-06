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

#include "GeoSemHelpers.h"
#include "GeoSemHelpersStatus.h"
#include "SemanticsManager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::ensureActiveMapBootstrapHierarchy(
    SemanticsManager::ActiveMapBootstrapResult &bootstrapResult_out,
    const std::optional<Eigen::Vector3d> &cameraPositionOverride_world_m_in)
{
    Map *p_atlasCurrentMap = nullptr;
    if ((p_atlas != nullptr) && p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                                    AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Map *p_activeMap = p_atlas != nullptr ? p_atlasCurrentMap : nullptr;
    if (p_activeMap == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"reason\":\"NO_ACTIVE_MAP\",\"semantic_cycle\":"
                  << pipelineSemanticCycle << "}" << std::endl;
        bootstrapResult_out = ActiveMapBootstrapResult::NO_ACTIVE_MAP;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<semantic::Room *> activeRooms{};
    if (p_activeMap->getAllRooms(activeRooms) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
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
        AtlasStatus::ATLAS_STATUS_SUCCESS)
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
            AtlasStatus::ATLAS_STATUS_SUCCESS)
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
        std::vector<KeyFrame *> keyFrames{};
        if (p_activeMap->getAllKeyFrames(keyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);
        for (std::vector<KeyFrame *>::reverse_iterator keyFrameIterator =
                 keyFrames.rbegin();
             keyFrameIterator != keyFrames.rend();
             ++keyFrameIterator)
        {
            KeyFrame *p_keyFrame = *keyFrameIterator;
            bool      keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad)
            {
                continue;
            }
            Eigen::Vector3f keyFrameCameraCenter{};
            if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector3d candidatePosition_world_m =
                keyFrameCameraCenter.cast<double>();
            /* An exactly-zero center marks an uninitialized first-frame pose
             * (live-observed: brand-new map, identity pose, room planted at
             * the origin), never a genuine measurement: real computed centers
             * carry rotation/translation noise. Accepting it misplaces the
             * bootstrap room and poisons centroid-distance matching for the
             * cycles until walls correct it. Fall through to the snapshot
             * centroid, else yield and retry once poses exist. */
            if (candidatePosition_world_m.allFinite() &&
                !candidatePosition_world_m.isZero())
            {
                cameraPosition_world_m  = candidatePosition_world_m;
                hasUsableCameraPosition = true;
                break;
            }
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
            unsigned long activeMapId{};
            if (p_activeMap->getId(activeMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
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
        vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
        if (GeoSemHelpers::createBlankRoomCandidate(
                p_atlas,
                p_blankRoomCandidate,
                cameraPosition_world_m,
                recoveryContext.has_value()
                    ? std::optional<int>(recoveryContext->roomId)
                    : std::nullopt) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: createBlankRoomCandidate returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        p_bootstrapRoom = p_blankRoomCandidate;
        if (p_bootstrapRoom == nullptr)
        {
            unsigned long activeMapId2{};
            if (p_activeMap->getId(activeMapId2) !=
                MapStatus::MAP_STATUS_SUCCESS)
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
        if (p_atlas->addCandidateMapRoom(p_bootstrapRoom) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addCandidateMapRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_activeMap->promoteCandidateMapRoom(p_bootstrapRoom) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: promoteCandidateMapRoom returned a failure status "
                "although it cannot fail; continuing as before.",
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
        int bootstrapRoomId3{};
        if (!(recoveryContext.has_value() &&
              !recoveryContext->roomTag.empty()) &&
            p_bootstrapRoom->getId(bootstrapRoomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_bootstrapRoom->setRoomTag(
                recoveryContext.has_value() && !recoveryContext->roomTag.empty()
                    ? recoveryContext->roomTag
                    : "room_" + std::to_string(bootstrapRoomId3)) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_bootstrapRoom->setRecoveryProxy(recoveryContext.has_value()) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRecoveryProxy returned a failure status "
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
    if (p_activeMap->getAllFloors(floors) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
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
        const std::optional<int> recoveryFloorId =
            recoveryContext.has_value() && recoveryContext->floorId >= 0
                ? std::optional<int>(recoveryContext->floorId)
                : std::nullopt;
        if (GeoSemHelpers::createMapFloor(p_atlas, recoveryFloorId) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Floor *> activeMapAllFloors{};
        if (p_activeMap->getAllFloors(activeMapAllFloors) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
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
        if (p_activeMap->getId(activeMapId3) != MapStatus::MAP_STATUS_SUCCESS)
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
                AtlasStatus::ATLAS_STATUS_SUCCESS)
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
        semantic::Room *p_activeMapStartingRoom = nullptr;
        if (p_activeMap->getStartingRoom(p_activeMapStartingRoom) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getStartingRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_activeMapStartingRoom == nullptr)
        {
            if (p_activeMap->setStartingRoom(p_bootstrapRoom) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setStartingRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
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
            if (p_activeMap->getPassageById(passageContext.id,
                                            p_recoveryPassage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassageById returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_recoveryPassage == nullptr)
            {
                /* New object, stable ID: position, orientation, and aperture
                 * dimensions are not knowable across a map break, so only
                 * frame-free state (passable, traversal history, live links
                 * below) is restored. A zero-sized aperture shrinks geometric
                 * tests to their margin sliver, as before this change. */
                p_recoveryPassage = new semantic::Passage();
                if (p_recoveryPassage->setId(passageContext.id) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: setId returned a failure status although "
                                 "it cannot fail; continuing as before.",
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
                        // addTraversalObservation cannot fail; continue as
                        // before.
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
                        // addTraversalObservation cannot fail; continue as
                        // before.
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
                        // addTraversalObservation cannot fail; continue as
                        // before.
                    }
                }
                if (p_atlas->addMapPassage(p_recoveryPassage) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addMapPassage returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            int bootstrapRoomId7{};
            if ((passageContext.hasKnownSideRoom) &&
                p_bootstrapRoom->getId(bootstrapRoomId7) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (passageContext.hasKnownSideRoom &&
                passageContext.knownSideRoomId == bootstrapRoomId7)
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
            int bootstrapRoomId8{};
            if ((passageContext.hasFarSideRoom) &&
                p_bootstrapRoom->getId(bootstrapRoomId8) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (passageContext.hasFarSideRoom &&
                passageContext.secondaryRoomId == bootstrapRoomId8)
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
        if (p_activeMap->getId(activeMapId4) != MapStatus::MAP_STATUS_SUCCESS)
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
        if (p_activeMap->getId(activeMapId5) != MapStatus::MAP_STATUS_SUCCESS)
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

} // namespace core
} // namespace vs_graphs
