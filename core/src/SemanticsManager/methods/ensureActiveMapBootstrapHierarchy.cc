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

#include "GeoSemHelpers.h"
#include "GeoSemHelpersStatus.h"
#include "SemanticsManager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManager::ActiveMapBootstrapResult
    SemanticsManager::ensureActiveMapBootstrapHierarchy(
        const std::optional<Eigen::Vector3d> &cameraPositionOverride_World_m_in)
{
    Map *p_activeMap = p_atlas != nullptr ? p_atlas->getCurrentMap() : nullptr;
    if (p_activeMap == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"reason\":\"NO_ACTIVE_MAP\",\"semantic_cycle\":"
                  << pipelineSemanticCycle << "}" << std::endl;
        return ActiveMapBootstrapResult::NO_ACTIVE_MAP;
    }

    const std::vector<semantic::Room *> activeRooms =
        p_activeMap->getAllRooms();
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
    const int recoveryRoomId = p_atlas->getCurrentSemanticRoomIdentity();

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

    const std::optional<semantic::RoomContextSnapshot> recoveryContext =
        p_bootstrapRoom == nullptr && recoveryRoomId >= 0
            ? p_atlas->copyLatestRoomContext(recoveryRoomId)
            : std::nullopt;

    Eigen::Vector3d cameraPosition_World_m  = Eigen::Vector3d::Zero();
    bool            hasUsableCameraPosition = false;
    if (cameraPositionOverride_World_m_in.has_value() &&
        cameraPositionOverride_World_m_in->allFinite())
    {
        cameraPosition_World_m  = *cameraPositionOverride_World_m_in;
        hasUsableCameraPosition = true;
    }
    else
    {
        std::vector<KeyFrame *> keyFrames = p_activeMap->getAllKeyFrames();
        std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);
        for (std::vector<KeyFrame *>::reverse_iterator keyFrameIterator =
                 keyFrames.rbegin();
             keyFrameIterator != keyFrames.rend();
             ++keyFrameIterator)
        {
            KeyFrame *p_keyFrame = *keyFrameIterator;
            if (p_keyFrame == nullptr || p_keyFrame->isBad())
            {
                continue;
            }
            const Eigen::Vector3d candidatePosition_World_m =
                p_keyFrame->getCameraCenter().cast<double>();
            /* An exactly-zero center marks an uninitialized first-frame pose
             * (live-observed: brand-new map, identity pose, room planted at
             * the origin), never a genuine measurement: real computed centers
             * carry rotation/translation noise. Accepting it misplaces the
             * bootstrap room and poisons centroid-distance matching for the
             * cycles until walls correct it. Fall through to the snapshot
             * centroid, else yield and retry once poses exist. */
            if (candidatePosition_World_m.allFinite() &&
                !candidatePosition_World_m.isZero())
            {
                cameraPosition_World_m  = candidatePosition_World_m;
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
            cameraPosition_World_m  = recoveryContext->centroid;
            hasUsableCameraPosition = true;
        }
        else
        {
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << p_activeMap->getId()
                      << ",\"reason\":\"NO_USABLE_CAMERA_POSE\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle << "}" << std::endl;
            return ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
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
                cameraPosition_World_m,
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
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << p_activeMap->getId()
                      << ",\"reason\":\"ROOM_CREATION_FAILED\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle << "}" << std::endl;
            return ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
        }
        p_atlas->addCandidateMapRoom(p_bootstrapRoom);
        p_activeMap->promoteCandidateMapRoom(p_bootstrapRoom);
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
        if (p_bootstrapRoom->setName("semantic::Room#" +
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

    std::vector<semantic::Floor *> floors = p_activeMap->getAllFloors();
    semantic::Floor               *p_canonicalFloor = nullptr;
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
        floors                       = p_activeMap->getAllFloors();
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
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"FLOOR_CREATION_FAILED\","
                     "\"room_id\":"
                  << bootstrapRoomId4
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
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
            p_atlas->setCurrentSemanticRoomIdentity(bootstrapRoomId6);
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
        if (p_activeMap->getStartingRoom() == nullptr)
        {
            p_activeMap->setStartingRoom(p_bootstrapRoom);
        }
    }

    std::size_t restoredPassageCount = 0U;
    if (recoveredRoom)
    {
        for (const semantic::PassageContext &passageContext :
             recoveryContext->passageContexts)
        {
            semantic::Passage *p_recoveryPassage =
                p_activeMap->getPassageById(passageContext.id);
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
                p_atlas->addMapPassage(p_recoveryPassage);
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
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"BOOTSTRAP_CREATED\",\"room_id\":"
                  << bootstrapRoomId9 << ",\"floor_id\":" << canonicalFloorId
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::INITIALIZED;
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
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"RECOVERY_RESTORED\",\"room_id\":"
                  << bootstrapRoomId10 << ",\"floor_id\":" << canonicalFloorId2
                  << ",\"restored_passages\":" << restoredPassageCount
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::RECOVERED;
    }

    return ActiveMapBootstrapResult::REUSED;
}

} // namespace core
} // namespace vs_graphs
