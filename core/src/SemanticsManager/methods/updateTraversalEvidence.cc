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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::updateTraversalEvidence(
    vs_graphs::core::Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return;
    }

    Map *p_activeMap = p_atlas_in->getCurrentMap();

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
                  return p_first->id < p_second->id;
              });

    if (orderedKeyFrames.empty())
    {
        return;
    }

    /* Prepare the ground normal: aperture height/width tests need the vertical
     * axis. Without it there is no reliable opening bounds test. */
    vs_graphs::core::geometric::Plane *p_groundPlane =
        p_atlas_in->getBiggestGroundPlane();

    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();

    bool hasValidGroundNormal = false;

    bool groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d groundEquation = groundPlaneGetGlobalEquation.coeffs();
        const double    groundNormalNorm = groundEquation.head<3>().norm();

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
    hasCameraCenter   = true;
    p_cameraCenterMap = p_activeMap;

    for (std::size_t keyFrameIndex = historyStartIndex + 1U;
         keyFrameIndex < orderedKeyFrames.size();
         ++keyFrameIndex)
    {
        KeyFrame *p_keyFrame = orderedKeyFrames[keyFrameIndex];

        const Eigen::Vector3d nextCameraCenter_World_m =
            p_keyFrame->getCameraCenter().cast<double>();

        lastTraversalFrameId    = p_keyFrame->frameId;
        lastTraversalKeyFrameId = p_keyFrame->id;

        if (!nextCameraCenter_World_m.allFinite())
        {
            continue;
        }

        previousCameraCenter_World_m = currentCameraCenter_World_m;
        currentCameraCenter_World_m  = nextCameraCenter_World_m;

        for (vs_graphs::core::semantic::Passage *p_passage : passages)
        {
            bool passageIsPassable{};
            if (!(p_passage == nullptr) &&
                p_passage->isPassable(passageIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage == nullptr || !passageIsPassable)
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
                bool wasSettled{};
                if (p_passage->getTraversalEvidence(wasSettled) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getTraversalEvidence returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                semantic::Passage::TraversalDirection traversalDirection =
                    semantic::Passage::TraversalDirection::UNKNOWN;
                semantic::Passage::KnownSideProvenance knownSide{};
                if (p_passage->getKnownSideProvenance(knownSide) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                bool knownSideHasDirection{};
                if (knownSide.hasDirection(knownSideHasDirection) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (!knownSideHasDirection)
                {
                    g2o::Plane3D passageGlobalEquation{};
                    if (p_passage->getGlobalEquation(passageGlobalEquation) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector4d passageEquation =
                        passageGlobalEquation.coeffs();
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
                            if (p_passage->setKnownSideDirection(
                                    (observationSide > 0.0 ? 1.0 : -1.0) *
                                    passageEquation.head<3>()) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                RCLCPP_WARN(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: setKnownSideDirection rejected its "
                                    "input; continuing as before.",
                                    __func__);
                            }
                            semantic::Passage::KnownSideProvenance
                                passageKnownSideProvenance{};
                            if (p_passage->getKnownSideProvenance(
                                    passageKnownSideProvenance) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                // getKnownSideProvenance cannot fail; continue
                                // as before.
                            }
                            knownSide = passageKnownSideProvenance;
                        }
                    }
                }
                bool knownSideHasDirection2{};
                if (knownSide.hasDirection(knownSideHasDirection2) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (knownSideHasDirection2)
                {
                    Eigen::Vector3d passageCentroid{};
                    if (p_passage->getCentroid(passageCentroid) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector3d startFromPassage_World_m =
                        previousCameraCenter_World_m - passageCentroid;
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
                    vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                        nullptr;
                    if (p_passage->getProspectiveRoom(
                            p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getProspectiveRoom returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    p_reachedRoom = p_passageProspectiveRoom;
                }
                else if (traversalDirection ==
                         semantic::Passage::TraversalDirection::FAR_TO_KNOWN)
                {
                    p_reachedRoom = knownSide.p_room;
                }
                const std::vector<semantic::Room *> activeRooms =
                    p_activeMap->getAllRooms();
                bool reachedRoomIsBad{};
                if ((p_reachedRoom != nullptr) &&
                    p_reachedRoom->isBad(reachedRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                core::Map *p_reachedRoomMap = nullptr;
                if ((p_reachedRoom != nullptr && !reachedRoomIsBad) &&
                    p_reachedRoom->getMap(p_reachedRoomMap) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const bool reachedRoomIsLive =
                    p_reachedRoom != nullptr && !reachedRoomIsBad &&
                    p_reachedRoomMap == p_activeMap &&
                    std::find(activeRooms.begin(),
                              activeRooms.end(),
                              p_reachedRoom) != activeRooms.end();
                if (reachedRoomIsLive)
                {
                    semantic::Room::RoomVariant reachedRoomRoomVariant{};
                    if (p_reachedRoom->getRoomVariant(reachedRoomRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRoomVariant returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (reachedRoomRoomVariant ==
                        semantic::Room::RoomVariant::UNDEFINED)
                    {
                        p_activeMap->promoteCandidateMapRoom(p_reachedRoom);
                        if (p_reachedRoom->setRoomVariant(
                                semantic::Room::RoomVariant::ROOM) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setRoomVariant returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        int reachedRoomId2{};
                        if (p_reachedRoom->getId(reachedRoomId2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (p_reachedRoom->setName(
                                "semantic::Room#" +
                                std::to_string(reachedRoomId2)) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setName returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        if (p_reachedRoom->setBoundaryStatus(
                                semantic::Room::BoundaryStatus::UNOBSERVED) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // setBoundaryStatus cannot fail; continue as
                            // before.
                        }
                        int reachedRoomId3{};
                        if (p_reachedRoom->getId(reachedRoomId3) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        prospectiveRoomCycles.erase(reachedRoomId3);

                        semantic::Floor *p_floor = nullptr;
                        if (semantic::Floor::selectBestObservedFloor(
                                p_activeMap->getAllFloors(),
                                p_floor) !=
                            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                        {
                            // selectBestObservedFloor cannot fail; continue as
                            // before.
                        }
                        if (p_floor != nullptr)
                        {
                            if (p_floor->addRoom(p_reachedRoom) !=
                                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: addRoom returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                        }
                        int reachedRoomId4{};
                        if (p_reachedRoom->getId(reachedRoomId4) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        int passageId{};
                        if (p_passage->getId(passageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        std::cout
                            << "SG_PIPELINE {\"event\":\"room_promotion\","
                               "\"map_id\":"
                            << p_activeMap->getId()
                            << ",\"room_id\":" << reachedRoomId4
                            << ",\"passage_id\":" << passageId
                            << ",\"reason\":\"PASSAGE_TRAVERSAL\","
                               "\"semantic_cycle\":"
                            << pipelineSemanticCycle << "}" << std::endl;
                    }
                    int reachedRoomId{};
                    if (p_reachedRoom->getId(reachedRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    {
                        std::lock_guard<std::mutex> currentRoomLock(
                            currentRoomMutex);
                        currentRoomId = reachedRoomId;
                    }
                    p_atlas->setCurrentSemanticRoomIdentity(reachedRoomId);
                    /* Completed passage traversal into this room: entry
                     * evidence marks it visited. */
                    if (p_reachedRoom->setPreviouslyVisited(true) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: setPreviouslyVisited returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                }

                bool addedTraversal{};
                if (p_passage->addTraversalObservation(traversalDirection,
                                                       p_keyFrame->frameId,
                                                       p_keyFrame->id,
                                                       addedTraversal) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addTraversalObservation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                /* Only newly accepted segment evidence is a new tracker event.
                 * The passage owns segment deduplication, so replayed history
                 * must not republish crossing evidence. */
                if (addedTraversal)
                {
                    std::lock_guard<std::mutex> currentRoomLock(
                        currentRoomMutex);
                    isCrossingEventPending = true;
                    bool passageHasBidirectionalTraversalEvidence{};
                    if (!(isCrossingBothSidesPending) &&
                        p_passage->hasBidirectionalTraversalEvidence(
                            passageHasBidirectionalTraversalEvidence) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: hasBidirectionalTraversalEvidence "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    isCrossingBothSidesPending =
                        isCrossingBothSidesPending ||
                        passageHasBidirectionalTraversalEvidence;
                    /* Test seam: empty outside tests (SemanticsManager.h). */
                    std::function<void()> publishHook =
                        std::move(roomTrackerPendingPublishHook);
                    if (publishHook)
                    {
                        publishHook();
                    }
                }

                if (addedTraversal && !wasSettled)
                {
                    int passageId2{};
                    if (p_passage->getId(passageId2) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] semantic::Passage#" << passageId2
                              << " traversed (traversal evidence settled)."
                              << std::endl;
                }
            }
        }
    }

    KeyFrame *p_latestKeyFrame = orderedKeyFrames.back();
    lastTraversalFrameId       = p_latestKeyFrame->frameId;
    lastTraversalKeyFrameId    = p_latestKeyFrame->id;
    hasTraversalKeyFrameCursor = true;
}

} // namespace core
} // namespace vs_graphs
