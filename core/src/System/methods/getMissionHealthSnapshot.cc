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

#include "LoopClosing.h"
#include "SemanticSegmentation.h"
#include "SemanticsManager.h"
#include "System.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

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

    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_params->general.modeOfOperation ==
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
        unsigned long activeMapId{};
        if (p_activeMap->getId(activeMapId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.mapId = static_cast<std::uint64_t>(activeMapId);
        std::vector<KeyFrame *> keyFrames{};
        if (p_activeMap->getAllKeyFrames(keyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.keyFrameCount = static_cast<std::uint32_t>(keyFrames.size());
        KeyFrame *p_latestKeyFrame = nullptr;
        for (KeyFrame *p_keyFrame : keyFrames)
        {
            bool keyFrameIsBad{};
            if ((p_keyFrame != nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame != nullptr && !keyFrameIsBad &&
                (p_latestKeyFrame == nullptr ||
                 p_keyFrame->id > p_latestKeyFrame->id))
            {
                p_latestKeyFrame = p_keyFrame;
            }
        }
        if (p_latestKeyFrame != nullptr)
        {
            snapshot.latestKeyFrameTimestamp = p_latestKeyFrame->timeStamp;
            Sophus::SE3f latestKeyFramePoseInverse{};
            if (p_latestKeyFrame->getPoseInverse(latestKeyFramePoseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            snapshot.latestKeyFramePose_World = latestKeyFramePoseInverse;
            snapshot.isLatestKeyFramePoseValid =
                snapshot.latestKeyFramePose_World.translation().allFinite() &&
                snapshot.latestKeyFramePose_World.rotationMatrix().allFinite();
        }

        if (includeSemantics_in)
        {
            std::vector<semantic::Room *> activeMapAllRooms{};
            if (p_activeMap->getAllRooms(activeMapAllRooms) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Room *p_room : activeMapAllRooms)
            {
                bool roomIsBad{};
                if (!(p_room == nullptr) &&
                    p_room->isBad(roomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room == nullptr || roomIsBad)
                {
                    continue;
                }
                semantic::Room::RoomVariant roomVariant{};
                if (p_room->getRoomVariant(roomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (roomVariant != semantic::Room::RoomVariant::ROOM)
                {
                    ++snapshot.unresolvedRoomCount;
                    continue;
                }

                ++snapshot.confirmedRoomCount;
                RoomHealth room;
                int        roomId{};
                if (p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                room.id = roomId;
                std::vector<vs_graphs::core::semantic::Passage *>
                    roomPassages{};
                if (p_room->getPassages(roomPassages) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPassages returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (semantic::Passage *p_passage : roomPassages)
                {
                    if (p_passage != nullptr)
                    {
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
                        room.passageIds.push_back(passageId);
                    }
                }
                std::sort(room.passageIds.begin(), room.passageIds.end());
                snapshot.rooms.push_back(std::move(room));
            }

            std::vector<semantic::Floor *> activeMapAllFloors{};
            if (p_activeMap->getAllFloors(activeMapAllFloors) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllFloors returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Floor *p_floor : activeMapAllFloors)
            {
                if (p_floor == nullptr)
                {
                    continue;
                }
                FloorHealth floor;
                int         floorId{};
                if (p_floor->getId(floorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                floor.id = floorId;
                std::vector<vs_graphs::core::semantic::Room *> floorRooms{};
                if (p_floor->getRooms(floorRooms) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRooms returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (semantic::Room *p_room : floorRooms)
                {
                    bool roomIsBad2{};
                    if ((p_room != nullptr) &&
                        p_room->isBad(roomIsBad2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    semantic::Room::RoomVariant roomVariant2{};
                    if ((p_room != nullptr && !roomIsBad2) &&
                        p_room->getRoomVariant(roomVariant2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRoomVariant returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_room != nullptr && !roomIsBad2 &&
                        roomVariant2 == semantic::Room::RoomVariant::ROOM)
                    {
                        int roomId2{};
                        if (p_room->getId(roomId2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        floor.roomIds.push_back(roomId2);
                        ++snapshot.floorRoomLinkCount;
                    }
                }
                std::sort(floor.roomIds.begin(), floor.roomIds.end());
                snapshot.floors.push_back(std::move(floor));
            }

            std::vector<vs_graphs::core::semantic::Passage *>
                activeMapAllPassages{};
            if (p_activeMap->getAllPassages(activeMapAllPassages) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Passage *p_passage : activeMapAllPassages)
            {
                if (p_passage == nullptr)
                {
                    continue;
                }
                PassageHealth passage;
                int           passageId2{};
                if (p_passage->getId(passageId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passage.id = passageId2;
                bool passageIsPassable{};
                if (p_passage->isPassable(passageIsPassable) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isPassable returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passage.isPassable = passageIsPassable;
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance{};
                if (p_passage->getKnownSideProvenance(
                        passageKnownSideProvenance) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance2{};
                if ((passageKnownSideProvenance.p_room) &&
                    p_passage->getKnownSideProvenance(
                        passageKnownSideProvenance2) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                int id2{};
                if ((passageKnownSideProvenance.p_room) &&
                    passageKnownSideProvenance2.p_room->getId(id2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passage.primaryRoomId =
                    passageKnownSideProvenance.p_room ? id2 : -1;
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                    nullptr;
                if (p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom2 =
                    nullptr;
                if ((p_passageProspectiveRoom) &&
                    p_passage->getProspectiveRoom(p_passageProspectiveRoom2) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                int id3{};
                if ((p_passageProspectiveRoom) &&
                    p_passageProspectiveRoom2->getId(id3) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passage.secondaryRoomId = p_passageProspectiveRoom ? id3 : -1;
                std::size_t passageTraversalKnownToFarCount{};
                if (p_passage->getTraversalKnownToFarCount(
                        passageTraversalKnownToFarCount) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getTraversalKnownToFarCount cannot fail; continue as
                    // before.
                }
                passage.primaryTraversalCount = passageTraversalKnownToFarCount;
                std::size_t passageTraversalFarToKnownCount{};
                if (p_passage->getTraversalFarToKnownCount(
                        passageTraversalFarToKnownCount) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getTraversalFarToKnownCount cannot fail; continue as
                    // before.
                }
                passage.secondaryTraversalCount =
                    passageTraversalFarToKnownCount;
                std::size_t passageTraversalUnknownCount{};
                if (p_passage->getTraversalUnknownCount(
                        passageTraversalUnknownCount) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getTraversalUnknownCount returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                passage.unknownCount = passageTraversalUnknownCount;
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
                if (knownSide.p_room != nullptr)
                {
                    int id4{};
                    if (knownSide.p_room->getId(id4) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    passage.primaryRoomId = id4;
                }
                semantic::Room *p_farSideRoom = nullptr;
                if (p_passage->getProspectiveRoom(p_farSideRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_farSideRoom != nullptr)
                {
                    int farSideRoomId{};
                    if (p_farSideRoom->getId(farSideRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    passage.secondaryRoomId = farSideRoomId;
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
