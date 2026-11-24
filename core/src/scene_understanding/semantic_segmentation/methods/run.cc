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
 * @file            run.cc
 *
 * @brief           Implements SemanticSegmentation::run(), declared in
 *                  SemanticSegmentation.h.
 */

#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

void SemanticSegmentation::run()
{
    /* Output message to indicate that semantic segmentation is starting */
    std::cout << "[SemSeg] Semantic Segmentation Started" << std::endl;

    /* Spin thread */
    while (true)
    {
        /* Graceful shutdown on System::Shutdown() */
        bool shouldFinish{};
        if (checkFinish(shouldFinish) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (shouldFinish)
        {
            break;
        }

        WorkItem workItem;
        bool     hasWorkItem = false;
        {
            std::lock_guard<std::mutex> lock(newKeyFramesMutex);
            if (!segmentedImageBuffer.empty())
            {
                workItem = std::move(segmentedImageBuffer.front());
                segmentedImageBuffer.pop_front();
                dequeuedCount.fetch_add(1U, std::memory_order_relaxed);
                hasWorkItem = true;
            }
        }

        /* Check if there is a new segmented image in the buffer. */
        if (!hasWorkItem)
        {
            usleep(3000);
            continue;
        }

        /*!
         * Get the point cloud from the respective keyframe via the atlas -
         * ignore it if KF doesn't exist.
         */
        keyframe::KeyFrame *p_thisKeyFrame = nullptr;
        if (p_atlas->getKeyFrameById(workItem.keyFrameId, p_thisKeyFrame) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKeyFrameById returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* If keyframe is bad continue */
        bool thisKeyFrameIsBad{};
        if (!(p_thisKeyFrame == nullptr) &&
            p_thisKeyFrame->isBad(thisKeyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_thisKeyFrame == nullptr || thisKeyFrameIsBad)
        {
            if (recordTerminalOutcome(workItem.keyFrameId,
                                      TerminalOutcome::MISSING_KEYFRAME) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: recordTerminalOutcome returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        /* With the segmenter off no segmentation result is expected */
        if (!isSegmenterOff && workItem.segmentationCloud == nullptr)
        {
            if (recordTerminalOutcome(workItem.keyFrameId,
                                      TerminalOutcome::MISSING_CLOUD) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: recordTerminalOutcome returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        map::Map *p_activeMap = nullptr;
        if (p_atlas->getCurrentMap(p_activeMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long activeMapId{};
        if (!(p_activeMap == nullptr) &&
            p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_thisKeyFrameMap = nullptr;
        if (!(p_activeMap == nullptr || activeMapId != workItem.sourceMapId) &&
            p_thisKeyFrame->getMap(p_thisKeyFrameMap) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_activeMap == nullptr || activeMapId != workItem.sourceMapId ||
            p_thisKeyFrameMap != p_activeMap)
        {
            if (recordTerminalOutcome(workItem.keyFrameId,
                                      TerminalOutcome::STALE_MAP) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: recordTerminalOutcome returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        /* Extract point cloud from keyframe */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_thisKeyFramePointCloud{};
        if (p_thisKeyFrame->getCurrentFramePointCloud(
                p_thisKeyFramePointCloud) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getCurrentFramePointCloud returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* If no point cloud in keyframe, skip to next frame */
        if (p_thisKeyFramePointCloud == nullptr)
        {
            std::cerr << "[SemSeg] Skipping keyframe " << p_thisKeyFrame->id
                      << ": the RGB-D point cloud is unavailable." << std::endl;
            if (recordTerminalOutcome(workItem.keyFrameId,
                                      TerminalOutcome::MISSING_CLOUD) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: recordTerminalOutcome returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        /*!
         * Points per class with the segmenter on; the whole keyframe cloud in
         * one slot with it off.
         */
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> p_clsCloudPtrs;
        if (separateKeyFrameCloud(p_thisKeyFrame,
                                  workItem.segmentationCloud,
                                  workItem.uncertaintyImage,
                                  p_thisKeyFramePointCloud,
                                  p_clsCloudPtrs) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: separateKeyFrameCloud returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * clear pointclouds as they are no longer needed and consume
         * significant memory. also
         */
        if (p_thisKeyFrame->clearPointCloud() !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearPointCloud returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Clear point-cloud data from older keyframes that were skipped by this
         * processing stage.
         *
         * @note            Keep the most recent few keyframes intact because
         *                  keyframes may be processed slightly out of order.
         *                  Once a keyframe is older than the buffer window, its
         *                  raw and classified point-cloud data are no longer
         *                  needed and can be released to reduce memory usage.
         */
        if (p_thisKeyFrame->id - lastProcessedKeyFrameId > 5)
        {
            /*!
             * A keyframe more than 5 ids behind the one just processed is
             * NOT necessarily "skipped" -- if segmentedImageBuffer has
             * backlogged past this window (the buffer processes strictly
             * oldest-first; see the front()/pop_front() loop above), that
             * keyframe is still legitimately queued, just not its turn
             * yet. Clearing its point cloud here races the buffer's own
             * processing: this sweep would delete data the normal
             * processing path above (p_thisKeyFrame->clearPointCloud(), a few
             * lines up) hasn't had a chance to consume, so its later
             * dequeue finds a null point cloud and permanently logs
             * "unavailable" -- turning a temporary backlog into
             * unrecoverable data loss for every keyframe caught behind
             * it. Snapshot the still-pending ids once under the buffer's
             * own lock so this sweep only clears keyframes that are
             * genuinely no longer queued.
             */
            std::unordered_set<uint64_t> pendingKeyFrameIds;
            {
                std::lock_guard<std::mutex> lock(newKeyFramesMutex);
                for (const WorkItem &bufferedItem : segmentedImageBuffer)
                {
                    pendingKeyFrameIds.insert(bufferedItem.keyFrameId);
                }
            }

            for (unsigned long int keyFrameId = lastProcessedKeyFrameId + 1;
                 keyFrameId < p_thisKeyFrame->id - 5;
                 keyFrameId++)
            {
                if (pendingKeyFrameIds.count(keyFrameId) > 0U)
                {
                    continue;
                }

                keyframe::KeyFrame *p_keyFrame = nullptr;
                if (p_atlas->getKeyFrameById(keyFrameId, p_keyFrame) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKeyFrameById returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                    keyFrameGetCurrentFramePointCloud{};
                if ((p_keyFrame != nullptr) &&
                    p_keyFrame->getCurrentFramePointCloud(
                        keyFrameGetCurrentFramePointCloud) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCurrentFramePointCloud returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_keyFrame != nullptr &&
                    keyFrameGetCurrentFramePointCloud != nullptr)
                {
                    if (p_keyFrame->clearPointCloud() !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: clearPointCloud returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame->clearClsClouds() !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: clearClsClouds returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
            lastProcessedKeyFrameId = p_thisKeyFrame->id - 5;
        }

        /* ------------------------------------------------------------------ *
         * PLANE EXTRACTION
         * ------------------------------------------------------------------ */

        /*!
         * Extract planes from segmented point cloud.
         *
         * @note            Each patch carries its class only as a guess;
         *                  applyPlaneCheckpoint() decides the label.
         */
        std::vector<PlanePatch> planePatches{};
        if (getPlanesFromClassClouds(p_clsCloudPtrs, planePatches) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getPlanesFromClassClouds returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /*!
         * Label the patches from geometry and drop failed ones (points, fit,
         * no label, wall size) before the semantic lock, so they can neither
         * match a mapped plane nor vote.
         */
        if (applyPlaneCheckpoint(p_thisKeyFrame, planePatches) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyPlaneCheckpoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Set the class specific point clouds to the keyframe */
        if (p_thisKeyFrame->setCurrentClsCloudPtrs(p_clsCloudPtrs) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCurrentClsCloudPtrs returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        {
            /*!
             * Plane association changes map-owned geometry and observation
             * edges. Serialize that short mutation with loop-closing's
             * semantic transfer; point-cloud inference remains outside the
             * transaction so it cannot unnecessarily delay a map merge.
             */
            std::unique_lock<std::mutex> semanticUpdateLock{};
            if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: acquireSemanticUpdateLock returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /*!
             * Plane extraction runs outside the semantic transaction. A map
             * merge may therefore invalidate the source keyframe or transfer
             * it away from the Atlas current map while inference is running.
             * Revalidate the source only after acquiring the transaction lock
             * so stale output cannot recreate observations in the merged map.
             */
            map::Map *p_currentMap = nullptr;
            if (p_atlas->getCurrentMap(p_currentMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            bool thisKeyFrameIsBad2{};
            if (!(p_thisKeyFrame == nullptr) &&
                p_thisKeyFrame->isBad(thisKeyFrameIsBad2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            map::Map *p_thisKeyFrameMap2 = nullptr;
            if (!(p_thisKeyFrame == nullptr || thisKeyFrameIsBad2) &&
                p_thisKeyFrame->getMap(p_thisKeyFrameMap2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_thisKeyFrame == nullptr || thisKeyFrameIsBad2 ||
                p_thisKeyFrameMap2 != p_currentMap)
            {
                std::cerr
                    << "[SemSeg] Discarding stale segmentation output for "
                       "keyframe "
                    << workItem.keyFrameId << " after a map change."
                    << std::endl;
                if (recordTerminalOutcome(workItem.keyFrameId,
                                          TerminalOutcome::STALE_MAP) !=
                    SemanticSegmentationStatus::
                        SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: recordTerminalOutcome returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                continue;
            }

            /* Add the planes to Atlas. */
            if (updatePlaneData(p_thisKeyFrame, planePatches) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updatePlaneData returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        if (recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::ACCEPTED) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: recordTerminalOutcome returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Only now has every frame taken from the buffer been handled. */
    if (setFinish() !=
        SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
