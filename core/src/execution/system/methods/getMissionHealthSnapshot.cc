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

/*!
 * @file            getMissionHealthSnapshot.cc
 *
 * @brief           Implements System::getMissionHealthSnapshot(), declared in
 *                  System.h.
 */

#include "estimation/tracking/tracking.h"
#include "execution/system/objects/SystemClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationStatusEnum.h"
#include "scene_understanding/semantic_segmentation/semantic_segmentation.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace system
{

SystemStatus System::getMissionHealthSnapshot(
    System::MissionHealthSnapshot &missionHealthSnapshot_out,
    bool                           includeSemantics_in)
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
        snapshot.cameraPose_world = currentCameraPose_world;
    }

    std::unique_lock<std::mutex> semanticUpdateLock;
    if (includeSemantics_in)
    {
        if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    map::Map *p_activeMap = nullptr;
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int atlasMaps{};
    if (p_atlas->countMaps(atlasMaps) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    snapshot.mapCount = static_cast<std::uint32_t>(std::max(0, atlasMaps));
    bool atlasIsImuInitialized{};
    if ((snapshot.isInertial) &&
        p_atlas->isImuInitialized(atlasIsImuInitialized) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    snapshot.isInertialInitialized =
        snapshot.isInertial && atlasIsImuInitialized;
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

    /* Both modes run every keyframe through the segmentation thread */
    if (p_semanticSegmentation != nullptr)
    {
        semantic_segmentation::SemanticSegmentation::ProcessingStats
            processingStats{};
        if (p_semanticSegmentation->getProcessingStats(processingStats) !=
            semantic_segmentation::SemanticSegmentationStatus::
                SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProcessingStats returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
        int semanticsManagerGetCurrentRoomId{};
        if (p_semanticsManager->getCurrentRoomId(
                semanticsManagerGetCurrentRoomId) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.currentRoomId = semanticsManagerGetCurrentRoomId;
        if (snapshot.trackingState == tracking::Tracking::LOST)
        {
            if (p_semanticsManager->onTrackingLost() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: onTrackingLost returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            if (p_semanticsManager->onTrackingRecovered() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: onTrackingRecovered returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        int semanticsManagerGetLastKnownRoomId{};
        if (p_semanticsManager->getLastKnownRoomId(
                semanticsManagerGetLastKnownRoomId) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKnownRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.lastKnownRoomId = semanticsManagerGetLastKnownRoomId;
    }

    if (p_activeMap != nullptr)
    {
        unsigned long activeMapId{};
        if (p_activeMap->getId(activeMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.mapId = static_cast<std::uint64_t>(activeMapId);
        std::vector<keyframe::KeyFrame *> keyFrames{};
        if (p_activeMap->getAllKeyFrames(keyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snapshot.keyFrameCount = static_cast<std::uint32_t>(keyFrames.size());
        keyframe::KeyFrame *p_latestKeyFrame = nullptr;
        for (keyframe::KeyFrame *p_keyFrame : keyFrames)
        {
            bool keyFrameIsBad{};
            if ((p_keyFrame != nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            snapshot.latestKeyFramePose_world = latestKeyFramePoseInverse;
            snapshot.isLatestKeyFramePoseValid =
                snapshot.latestKeyFramePose_world.translation().allFinite() &&
                snapshot.latestKeyFramePose_world.rotationMatrix().allFinite();
        }

        if (includeSemantics_in)
        {
            /* One shared graph lock, taken and released inside the manager;
             * the counts below read only the copy. */
            SceneGraphSnapshot activeMapSceneGraph{};
            if (p_semanticsManager->getSceneGraphSnapshot(
                    activeMapId,
                    activeMapSceneGraph) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getSceneGraphSnapshot found no scene graph "
                             "for the active map; continuing as before.",
                             __func__);
            }
            for (const SceneGraphRoomEntry &roomEntry :
                 activeMapSceneGraph.rooms)
            {
                if (roomEntry.isBad)
                {
                    continue;
                }
                if (roomEntry.variant != semantic::Room::RoomVariant::ROOM)
                {
                    ++snapshot.unresolvedRoomCount;
                    continue;
                }
                ++snapshot.confirmedRoomCount;
                RoomHealth room;
                room.id         = roomEntry.id;
                room.passageIds = roomEntry.passageIds;
                std::sort(room.passageIds.begin(), room.passageIds.end());
                snapshot.rooms.push_back(std::move(room));
            }
            for (const SceneGraphFloorEntry &floorEntry :
                 activeMapSceneGraph.floors)
            {
                FloorHealth floor;
                floor.id = floorEntry.id;
                for (const int floorRoomId : floorEntry.roomIds)
                {
                    const SceneGraphRoomEntry *p_floorRoom =
                        activeMapSceneGraph.findRoom(floorRoomId);
                    if (p_floorRoom != nullptr && !p_floorRoom->isBad &&
                        p_floorRoom->variant ==
                            semantic::Room::RoomVariant::ROOM)
                    {
                        floor.roomIds.push_back(floorRoomId);
                        ++snapshot.floorRoomLinkCount;
                    }
                }
                std::sort(floor.roomIds.begin(), floor.roomIds.end());
                snapshot.floors.push_back(std::move(floor));
            }
            for (const SceneGraphPassageEntry &passageEntry :
                 activeMapSceneGraph.passages)
            {
                PassageHealth passage;
                passage.id         = passageEntry.id;
                passage.isPassable = passageEntry.isPassable;
                passage.primaryRoomId =
                    passageEntry.knownSideRoomId.value_or(-1);
                passage.secondaryRoomId =
                    passageEntry.prospectiveRoomId.value_or(-1);
                passage.primaryTraversalCount =
                    passageEntry.traversalKnownToFarCount;
                passage.secondaryTraversalCount =
                    passageEntry.traversalFarToKnownCount;
                passage.unknownCount = passageEntry.traversalUnknownCount;
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
        loop_closing::LoopClosing::LoopCorrectionStatus loop{};
        if (p_loopCloser->getLoopCorrectionStatus(loop) !=
            loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getLoopCorrectionStatus returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
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
    missionHealthSnapshot_out = snapshot;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */
