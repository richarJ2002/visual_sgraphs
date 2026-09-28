/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"

namespace vs_graphs
{
namespace core
{

System::MissionHealthSnapshot
    System::getMissionHealthSnapshot(bool includeSemantics_in)
{
    MissionHealthSnapshot snapshot;
    snapshot.isInertial =
        sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        snapshot.frameTimestamp   = lastFrameTimestamp;
        snapshot.trackingState    = trackingState;
        snapshot.trackingInliers  = trackingInliers;
        snapshot.isPoseValid      = isCurrentCameraPoseValid;
        snapshot.cameraPose_World = currentCameraPose_World;
    }

    std::unique_lock<std::mutex> semanticUpdateLock;
    if (includeSemantics_in)
    {
        semanticUpdateLock = p_atlas->acquireSemanticUpdateLock();
    }
    Map *p_activeMap = p_atlas->getCurrentMap();
    snapshot.mapCount =
        static_cast<std::uint32_t>(std::max(0, p_atlas->countMaps()));
    snapshot.isInertialInitialized =
        snapshot.isInertial && p_atlas->isImuInitialized();
    snapshot.resetCount = resetCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendAcceptedCount =
        rgbdFrontendAcceptedCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendProcessedCount =
        rgbdFrontendProcessedCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendOverwrittenCount =
        rgbdFrontendOverwrittenCount.load(std::memory_order_relaxed);
    snapshot.isRgbdFrontendWorkerInFlight =
        isRgbdFrontendWorkerInFlight.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendLastProcessedSensorTimestampNanoseconds =
        rgbdFrontendLastProcessedSensorTimestampNanoseconds.load(
            std::memory_order_relaxed);
    snapshot.segmentationPublishedCount =
        segmentationPublishedCount.load(std::memory_order_relaxed);
    snapshot.segmentationReturnedCount =
        segmentationReturnedCount.load(std::memory_order_relaxed);
    snapshot.lastReturnedKeyFrameId =
        lastReturnedKeyFrameId.load(std::memory_order_relaxed);

    if (types::SystemParams::getParams()->general.modeOfOperation ==
        types::SystemParams::General::ModeOfOperation::GEO)
    {
        snapshot.segmentationTerminalCount = snapshot.segmentationReturnedCount;
        snapshot.lastTerminalKeyFrameId    = snapshot.lastReturnedKeyFrameId;
    }
    else if (p_semanticSegmentation != nullptr)
    {
        const SemanticSegmentation::ProcessingStats processingStats =
            p_semanticSegmentation->getProcessingStats();
        snapshot.segmentationEnqueuedCount = processingStats.enqueuedCount;
        snapshot.segmentationDequeuedCount = processingStats.dequeuedCount;
        snapshot.segmentationTerminalCount = processingStats.terminalCount;
        snapshot.segmentationAcceptedCount = processingStats.acceptedCount;
        snapshot.segmentationDroppedCount  = processingStats.droppedCount;
        snapshot.segmentationMissingKeyFrameCount =
            processingStats.missingKeyFrameCount;
        snapshot.segmentationMissingCloudCount =
            processingStats.missingCloudCount;
        snapshot.segmentationStaleMapCount = processingStats.staleMapCount;
        snapshot.lastTerminalKeyFrameId =
            processingStats.lastTerminalKeyFrameId;
        snapshot.segmentationQueueDepth = processingStats.queueDepth;
        snapshot.segmentationQueueHighWatermark =
            processingStats.queueHighWatermark;
    }

    if (p_semanticsManager != nullptr)
    {
        snapshot.currentRoomId = p_semanticsManager->getCurrentRoomId();
        if (snapshot.trackingState == Tracking::LOST)
        {
            p_semanticsManager->onTrackingLost();
        }
        else
        {
            p_semanticsManager->onTrackingRecovered();
        }
        snapshot.lastKnownRoomId = p_semanticsManager->getLastKnownRoomId();
    }

    if (p_activeMap != nullptr)
    {
        snapshot.mapId = static_cast<std::uint64_t>(p_activeMap->getId());
        const std::vector<KeyFrame *> keyFrames =
            p_activeMap->getAllKeyFrames();
        snapshot.keyFrameCount = static_cast<std::uint32_t>(keyFrames.size());
        KeyFrame *p_latestKeyFrame = nullptr;
        for (KeyFrame *p_keyFrame : keyFrames)
        {
            if (p_keyFrame != nullptr && !p_keyFrame->isBad() &&
                (p_latestKeyFrame == nullptr ||
                 p_keyFrame->id > p_latestKeyFrame->id))
            {
                p_latestKeyFrame = p_keyFrame;
            }
        }
        if (p_latestKeyFrame != nullptr)
        {
            snapshot.latestKeyFrameTimestamp = p_latestKeyFrame->timeStamp;
            snapshot.latestKeyFramePose_World =
                p_latestKeyFrame->getPoseInverse();
            snapshot.isLatestKeyFramePoseValid =
                snapshot.latestKeyFramePose_World.translation().allFinite() &&
                snapshot.latestKeyFramePose_World.rotationMatrix().allFinite();
        }

        if (includeSemantics_in)
        {
            for (semantic::Room *p_room : p_activeMap->getAllRooms())
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    continue;
                }
                if (p_room->getRoomVariant() !=
                    semantic::Room::RoomVariant::ROOM)
                {
                    ++snapshot.unresolvedRoomCount;
                    continue;
                }

                ++snapshot.confirmedRoomCount;
                RoomHealth room;
                room.id = p_room->getId();
                for (semantic::Passage *p_passage : p_room->getPassages())
                {
                    if (p_passage != nullptr)
                    {
                        room.passageIds.push_back(p_passage->getId());
                    }
                }
                std::sort(room.passageIds.begin(), room.passageIds.end());
                snapshot.rooms.push_back(std::move(room));
            }

            for (semantic::Floor *p_floor : p_activeMap->getAllFloors())
            {
                if (p_floor == nullptr)
                {
                    continue;
                }
                FloorHealth floor;
                floor.id = p_floor->getId();
                for (semantic::Room *p_room : p_floor->getRooms())
                {
                    if (p_room != nullptr && !p_room->isBad() &&
                        p_room->getRoomVariant() ==
                            semantic::Room::RoomVariant::ROOM)
                    {
                        floor.roomIds.push_back(p_room->getId());
                        ++snapshot.floorRoomLinkCount;
                    }
                }
                std::sort(floor.roomIds.begin(), floor.roomIds.end());
                snapshot.floors.push_back(std::move(floor));
            }

            for (semantic::Passage *p_passage : p_activeMap->getAllPassages())
            {
                if (p_passage == nullptr)
                {
                    continue;
                }
                PassageHealth passage;
                passage.id         = p_passage->getId();
                passage.isPassable = p_passage->isPassable();
                passage.primaryRoomId =
                    p_passage->getKnownSideProvenance().p_room
                        ? p_passage->getKnownSideProvenance().p_room->getId()
                        : -1;
                passage.secondaryRoomId =
                    p_passage->getProspectiveRoom()
                        ? p_passage->getProspectiveRoom()->getId()
                        : -1;
                passage.primaryTraversalCount =
                    p_passage->getTraversalKnownToFarCount();
                passage.secondaryTraversalCount =
                    p_passage->getTraversalFarToKnownCount();
                passage.unknownCount = p_passage->getTraversalUnknownCount();
                const semantic::Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (knownSide.p_room != nullptr)
                {
                    passage.primaryRoomId = knownSide.p_room->getId();
                }
                semantic::Room *p_farSideRoom = p_passage->getProspectiveRoom();
                if (p_farSideRoom != nullptr)
                {
                    passage.secondaryRoomId = p_farSideRoom->getId();
                }
                snapshot.passages.push_back(passage);
            }
        }
    }

    if (semanticUpdateLock.owns_lock())
    {
        semanticUpdateLock.unlock();
    }
    if (p_loopCloser != nullptr)
    {
        const LoopClosing::LoopCorrectionStatus loop =
            p_loopCloser->getLoopCorrectionStatus();
        snapshot.loopSequence              = loop.sequence;
        snapshot.acceptedLoopCount         = loop.acceptedCount;
        snapshot.rejectedLoopCount         = loop.rejectedCount;
        snapshot.hasLoopEvent              = loop.hasEvent;
        snapshot.wasLastLoopAccepted       = loop.wasLastAccepted;
        snapshot.lastLoopMapId             = loop.lastMapId;
        snapshot.lastLoopCurrentKeyFrameId = loop.lastCurrentKeyFrameId;
        snapshot.lastLoopMatchedKeyFrameId = loop.lastMatchedKeyFrameId;
        snapshot.lastLoopCurrentTimestamp  = loop.lastCurrentTimestamp;
        snapshot.lastLoopMatchedTimestamp  = loop.lastMatchedTimestamp;
        snapshot.lastLoopReason            = loop.lastReason;
    }
    return snapshot;
}

} // namespace core
} // namespace vs_graphs
