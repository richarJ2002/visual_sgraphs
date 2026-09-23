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

#include "SemanticsManager.h"

#include <algorithm>

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
                  << pipelineSemanticCycle_ << "}" << std::endl;
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
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM &&
                p_room->getId() == roomId_in)
            {
                return p_room;
            }
        }
        return nullptr;
    };

    int currentRoomId = -1;
    {
        std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
        currentRoomId = currentRoomId_;
    }
    const int recoveryRoomId = p_atlas->getCurrentSemanticRoomIdentity();

    semantic::Room *p_bootstrapRoom = resolveLiveRoomById(currentRoomId);
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = resolveLiveRoomById(recoveryRoomId);
    }
    /* When a recovery identity exists (tracking-loss reset), never fall back
     * to an arbitrary lowest-ID live room: a spurious free-space SE# created
     * during the reset transient would otherwise hijack `currentRoomId_` away
     * from the last-known hierarchy. The lowest-ID seed applies to cold start
     * only (no recovery identity). */
    if (p_bootstrapRoom == nullptr && recoveryRoomId < 0)
    {
        for (semantic::Room *p_room : activeRooms)
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM &&
                (p_bootstrapRoom == nullptr ||
                 p_room->getId() < p_bootstrapRoom->getId()))
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
                      << pipelineSemanticCycle_ << "}" << std::endl;
            return ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
        }
    }

    bool initializedRoom = false;
    bool recoveredRoom   = false;
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = GeoSemHelpers::createBlankRoomCandidate(
            p_atlas,
            cameraPosition_World_m,
            recoveryContext.has_value()
                ? std::optional<int>(recoveryContext->roomId)
                : std::nullopt);
        if (p_bootstrapRoom == nullptr)
        {
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << p_activeMap->getId()
                      << ",\"reason\":\"ROOM_CREATION_FAILED\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle_ << "}" << std::endl;
            return ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
        }
        p_atlas->addCandidateMapRoom(p_bootstrapRoom);
        p_activeMap->promoteCandidateMapRoom(p_bootstrapRoom);
        p_bootstrapRoom->setRoomVariant(semantic::Room::RoomVariant::ROOM);
        p_bootstrapRoom->setName("semantic::Room#" +
                                 std::to_string(p_bootstrapRoom->getId()));
        p_bootstrapRoom->setBoundaryStatus(
            semantic::Room::BoundaryStatus::UNOBSERVED);
        p_bootstrapRoom->setRoomTag(
            recoveryContext.has_value() && !recoveryContext->roomTag.empty()
                ? recoveryContext->roomTag
                : "room_" + std::to_string(p_bootstrapRoom->getId()));
        p_bootstrapRoom->setRecoveryProxy(recoveryContext.has_value());
        if (recoveryContext.has_value())
        {
            p_bootstrapRoom->setPreviouslyVisited(
                recoveryContext->wasPreviouslyVisited);
        }
        initializedRoom = !recoveryContext.has_value();
        recoveredRoom   = recoveryContext.has_value();
    }

    std::vector<semantic::Floor *> floors = p_activeMap->getAllFloors();
    semantic::Floor               *p_canonicalFloor =
        semantic::Floor::selectBestObservedFloor(floors);
    if (p_canonicalFloor == nullptr)
    {
        const std::optional<int> recoveryFloorId =
            recoveryContext.has_value() && recoveryContext->floorId >= 0
                ? std::optional<int>(recoveryContext->floorId)
                : std::nullopt;
        GeoSemHelpers::createMapFloor(p_atlas, recoveryFloorId);
        floors           = p_activeMap->getAllFloors();
        p_canonicalFloor = semantic::Floor::selectBestObservedFloor(floors);
    }
    if (p_canonicalFloor == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"FLOOR_CREATION_FAILED\","
                     "\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
    }

    p_canonicalFloor->addRoom(p_bootstrapRoom);
    if (resolveLiveRoomById(currentRoomId) == nullptr)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
            currentRoomId_ = p_bootstrapRoom->getId();
            p_atlas->setCurrentSemanticRoomIdentity(p_bootstrapRoom->getId());
        }
        /* The UAV starts inside the bootstrap room: presence evidences entry.
         * Marked outside the current-room lock; the room owns its mutex. */
        p_bootstrapRoom->setPreviouslyVisited(true);
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
                p_recoveryPassage->setId(passageContext.id);
                p_recoveryPassage->setMap(p_activeMap);
                p_recoveryPassage->setPassable(passageContext.passable);
                p_recoveryPassage->setPassageType(
                    semantic::Passage::PassageVariant::DOORWAY);
                p_recoveryPassage->setRecoveryProxy(true);
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalKnownToFarCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalFarToKnownCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        semantic::Passage::TraversalDirection::FAR_TO_KNOWN);
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalUnknownCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        semantic::Passage::TraversalDirection::UNKNOWN);
                }
                p_atlas->addMapPassage(p_recoveryPassage);
            }

            if (passageContext.hasKnownSideRoom &&
                passageContext.knownSideRoomId == p_bootstrapRoom->getId())
            {
                p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom);
            }
            if (passageContext.hasFarSideRoom &&
                passageContext.secondaryRoomId == p_bootstrapRoom->getId())
            {
                p_recoveryPassage->setProspectiveRoom(p_bootstrapRoom);
            }
            if (p_recoveryPassage->getKnownSideProvenance().pRoom == nullptr &&
                p_recoveryPassage->getProspectiveRoom() == nullptr)
            {
                p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom);
            }
            p_bootstrapRoom->setDoorways(p_recoveryPassage);
            ++restoredPassageCount;
        }
    }

    if (initializedRoom)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"BOOTSTRAP_CREATED\",\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"floor_id\":" << p_canonicalFloor->getId()
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::INITIALIZED;
    }

    if (recoveredRoom)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->getId()
                  << ",\"reason\":\"RECOVERY_RESTORED\",\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"floor_id\":" << p_canonicalFloor->getId()
                  << ",\"restored_passages\":" << restoredPassageCount
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::RECOVERED;
    }

    return ActiveMapBootstrapResult::REUSED;
}

} // namespace core
} // namespace vs_graphs
