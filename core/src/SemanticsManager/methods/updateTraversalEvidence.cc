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

#include "../private_functions.h"

#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::updateTraversalEvidence(vs_graphs::core::Atlas *pAtlas)
{
    if (pAtlas == nullptr)
    {
        return;
    }

    Map *p_activeMap = pAtlas->getCurrentMap();

    if (p_activeMap == nullptr)
    {
        return;
    }

    resetTemporalStateForMap(p_activeMap);

    seedCurrentRoomFromActiveMap(p_activeMap);

    std::vector<KeyFrame *> orderedKeyFrames = p_activeMap->getAllKeyFrames();
    orderedKeyFrames.erase(std::remove_if(orderedKeyFrames.begin(),
                                          orderedKeyFrames.end(),
                                          [](KeyFrame *p_keyFrame) {
                                              return p_keyFrame == nullptr ||
                                                     p_keyFrame->isBad();
                                          }),
                           orderedKeyFrames.end());
    std::sort(orderedKeyFrames.begin(),
              orderedKeyFrames.end(),
              [](const KeyFrame *p_first, const KeyFrame *p_second)
              {
                  if (p_first->frameId != p_second->frameId)
                  {
                      return p_first->frameId < p_second->frameId;
                  }
                  return p_first->mnId < p_second->mnId;
              });

    if (orderedKeyFrames.empty())
    {
        return;
    }

    /* Prepare the ground normal: aperture height/width tests need the vertical
     * axis. Without it there is no reliable opening bounds test. */
    vs_graphs::core::geometric::Plane *groundPlane =
        pAtlas->getBiggestGroundPlane();

    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();

    bool hasValidGroundNormal = false;

    if (groundPlane != nullptr && !groundPlane->isBad())
    {
        Eigen::Vector4d groundEquation =
            groundPlane->getGlobalEquation().coeffs();
        const double groundNormalNorm = groundEquation.head<3>().norm();

        if (groundEquation.allFinite() && std::isfinite(groundNormalNorm) &&
            groundNormalNorm > 1e-8)
        {
            groundEquation /= groundNormalNorm;
            groundNormal_World   = groundEquation.head<3>();
            hasValidGroundNormal = true;
        }
    }

    if (!hasValidGroundNormal)
    {
        return;
    }

    const double openingMargin_m = static_cast<double>(
        p_sysParams->roomSeg.passagePartition.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        p_sysParams->roomSeg.passagePartition.minimumSideDistance_m);
    const std::vector<semantic::Passage *> passages =
        p_activeMap->getAllPassages();

    /* Passage confirmation is delayed relative to flight. Replay a bounded
     * recent trajectory on every semantic cycle; Passage deduplicates segment
     * IDs, so a crossing observed before the passage existed is retained once
     * the persistent passage appears. */
    constexpr std::size_t maximumTraversalHistoryKeyFrames = 128U;
    const std::size_t     historyStartIndex =
        orderedKeyFrames.size() > maximumTraversalHistoryKeyFrames
                ? orderedKeyFrames.size() - maximumTraversalHistoryKeyFrames
                : 0U;
    KeyFrame *p_seedKeyFrame = orderedKeyFrames[historyStartIndex];
    currentCameraCenter_World_m =
        p_seedKeyFrame->getCameraCenter().cast<double>();
    if (!currentCameraCenter_World_m.allFinite())
    {
        return;
    }
    hasCameraCenter_  = true;
    pCameraCenterMap_ = p_activeMap;

    for (std::size_t keyFrameIndex = historyStartIndex + 1U;
         keyFrameIndex < orderedKeyFrames.size();
         ++keyFrameIndex)
    {
        KeyFrame *p_keyFrame = orderedKeyFrames[keyFrameIndex];

        const Eigen::Vector3d nextCameraCenter_World_m =
            p_keyFrame->getCameraCenter().cast<double>();

        lastTraversalFrameId_    = p_keyFrame->frameId;
        lastTraversalKeyFrameId_ = p_keyFrame->mnId;

        if (!nextCameraCenter_World_m.allFinite())
        {
            continue;
        }

        previousCameraCenter_World_m = currentCameraCenter_World_m;
        currentCameraCenter_World_m  = nextCameraCenter_World_m;

        for (vs_graphs::core::semantic::Passage *p_passage : passages)
        {
            if (p_passage == nullptr || !p_passage->isPassable())
            {
                continue;
            }

            /* The segment joining the last two camera centres approximates the
             * UAV trajectory. When it crosses inside the finite aperture while
             * the passage is passable, the UAV has flown through the opening:
             * record traversal evidence. */
            if (segmentCrossesPassageOpening(previousCameraCenter_World_m,
                                             currentCameraCenter_World_m,
                                             p_passage,
                                             groundNormal_World,
                                             openingMargin_m,
                                             minimumSideDistance_m,
                                             true))
            {
                const bool wasSettled = p_passage->getTraversalEvidence();

                semantic::Passage::TraversalDirection traversalDirection =
                    semantic::Passage::TraversalDirection::UNKNOWN;
                semantic::Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (!knownSide.hasDirection())
                {
                    Eigen::Vector4d passageEquation =
                        p_passage->getGlobalEquation().coeffs();
                    const double normalNorm = passageEquation.head<3>().norm();
                    if (passageEquation.allFinite() && normalNorm > 1e-8)
                    {
                        passageEquation /= normalNorm;
                        const double observationSide =
                            passageEquation.head<3>().dot(
                                previousCameraCenter_World_m) +
                            passageEquation(3);
                        if (std::abs(observationSide) > minimumSideDistance_m)
                        {
                            p_passage->setKnownSideDirection(
                                (observationSide > 0.0 ? 1.0 : -1.0) *
                                passageEquation.head<3>());
                            knownSide = p_passage->getKnownSideProvenance();
                        }
                    }
                }
                if (knownSide.hasDirection())
                {
                    const Eigen::Vector3d startFromPassage_World_m =
                        previousCameraCenter_World_m - p_passage->getCentroid();
                    traversalDirection =
                        startFromPassage_World_m.dot(
                            knownSide.direction_World) >= 0.0
                            ? semantic::Passage::TraversalDirection::
                                  KNOWN_TO_FAR
                            : semantic::Passage::TraversalDirection::
                                  FAR_TO_KNOWN;
                }

                /* Resolve the room entered after crossing this passage: a
                 * KNOWN_TO_FAR crossing reaches the far side, a FAR_TO_KNOWN
                 * crossing returns to the known side. */
                vs_graphs::core::semantic::Room *p_reachedRoom = nullptr;
                if (traversalDirection ==
                    semantic::Passage::TraversalDirection::KNOWN_TO_FAR)
                {
                    p_reachedRoom = p_passage->getProspectiveRoom();
                }
                else if (traversalDirection ==
                         semantic::Passage::TraversalDirection::FAR_TO_KNOWN)
                {
                    p_reachedRoom = knownSide.pRoom;
                }
                const std::vector<semantic::Room *> activeRooms =
                    p_activeMap->getAllRooms();
                const bool reachedRoomIsLive =
                    p_reachedRoom != nullptr && !p_reachedRoom->isBad() &&
                    p_reachedRoom->getMap() == p_activeMap &&
                    std::find(activeRooms.begin(),
                              activeRooms.end(),
                              p_reachedRoom) != activeRooms.end();
                if (reachedRoomIsLive)
                {
                    if (p_reachedRoom->getRoomVariant() ==
                        semantic::Room::RoomVariant::UNDEFINED)
                    {
                        p_activeMap->promoteCandidateMapRoom(p_reachedRoom);
                        p_reachedRoom->setRoomVariant(
                            semantic::Room::RoomVariant::ROOM);
                        p_reachedRoom->setName(
                            "semantic::Room#" +
                            std::to_string(p_reachedRoom->getId()));
                        p_reachedRoom->setBoundaryStatus(
                            semantic::Room::BoundaryStatus::UNOBSERVED);
                        prospectiveRoomCycles_.erase(p_reachedRoom->getId());

                        semantic::Floor *p_floor =
                            semantic::Floor::selectBestObservedFloor(
                                p_activeMap->getAllFloors());
                        if (p_floor != nullptr)
                        {
                            p_floor->addRoom(p_reachedRoom);
                        }
                        std::cout
                            << "SG_PIPELINE {\"event\":\"room_promotion\","
                               "\"map_id\":"
                            << p_activeMap->getId()
                            << ",\"room_id\":" << p_reachedRoom->getId()
                            << ",\"passage_id\":" << p_passage->getId()
                            << ",\"reason\":\"PASSAGE_TRAVERSAL\","
                               "\"semantic_cycle\":"
                            << pipelineSemanticCycle_ << "}" << std::endl;
                    }
                    const int reachedRoomId = p_reachedRoom->getId();
                    {
                        std::lock_guard<std::mutex> currentRoomLock(
                            mMutexCurrentRoom);
                        currentRoomId_ = reachedRoomId;
                    }
                    p_atlas->setCurrentSemanticRoomIdentity(reachedRoomId);
                    /* Completed passage traversal into this room: entry
                     * evidence marks it visited. */
                    p_reachedRoom->setPreviouslyVisited(true);
                }

                const bool addedTraversal =
                    p_passage->addTraversalObservation(traversalDirection,
                                                       p_keyFrame->frameId,
                                                       p_keyFrame->mnId);

                /* Only newly accepted segment evidence is a new tracker event.
                 * The passage owns segment deduplication, so replayed history
                 * must not republish crossing evidence. */
                if (addedTraversal)
                {
                    std::lock_guard<std::mutex> currentRoomLock(
                        mMutexCurrentRoom);
                    crossingEventPending_ = true;
                    crossingBothSidesPending_ =
                        crossingBothSidesPending_ ||
                        p_passage->hasBidirectionalTraversalEvidence();
                    /* Test seam: empty outside tests (SemanticsManager.h). */
                    std::function<void()> publishHook =
                        std::move(roomTrackerPendingPublishHook_);
                    if (publishHook)
                    {
                        publishHook();
                    }
                }

                if (addedTraversal && !wasSettled)
                {
                    std::cout << "[SemMgr] semantic::Passage#"
                              << p_passage->getId()
                              << " traversed (traversal evidence settled)."
                              << std::endl;
                }
            }
        }
    }

    KeyFrame *p_latestKeyFrame  = orderedKeyFrames.back();
    lastTraversalFrameId_       = p_latestKeyFrame->frameId;
    lastTraversalKeyFrameId_    = p_latestKeyFrame->mnId;
    hasTraversalKeyFrameCursor_ = true;
}

} // namespace core
} // namespace vs_graphs
