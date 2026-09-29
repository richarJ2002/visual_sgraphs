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

#include "SemanticSegmentation.h"

#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticSegmentation::run()
{
    /* Output message to indicate that semantic segmentation is starting */
    std::cout << "[SemSeg] Semantic Segmentation Started" << std::endl;

    /* Spin thread */
    while (true)
    {
        /* Graceful shutdown on System::Shutdown() */
        if (checkFinish())
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
        KeyFrame *p_thisKeyFrame =
            p_atlas->getKeyFrameById(workItem.keyFrameId);

        /* If keyframe is bad continue */
        bool thisKeyFrameIsBad{};
        if (!(p_thisKeyFrame == nullptr) &&
            p_thisKeyFrame->isBad(thisKeyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_thisKeyFrame == nullptr || thisKeyFrameIsBad)
        {
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::MISSING_KEYFRAME);
            continue;
        }

        if (workItem.segmentationCloud == nullptr)
        {
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::MISSING_CLOUD);
            continue;
        }

        Map          *p_activeMap = p_atlas->getCurrentMap();
        unsigned long activeMapId{};
        if (!(p_activeMap == nullptr) &&
            p_activeMap->getId(activeMapId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_thisKeyFrameMap = nullptr;
        if (!(p_activeMap == nullptr || activeMapId != workItem.sourceMapId) &&
            p_thisKeyFrame->getMap(p_thisKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_activeMap == nullptr || activeMapId != workItem.sourceMapId ||
            p_thisKeyFrameMap != p_activeMap)
        {
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::STALE_MAP);
            continue;
        }

        /* Extract point cloud from keyframe */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_thisKeyFramePointCloud{};
        if (p_thisKeyFrame->getCurrentFramePointCloud(
                p_thisKeyFramePointCloud) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::MISSING_CLOUD);
            continue;
        }

        /* Extract the segmentation probabilities from the image */
        pcl::PCLPointCloud2::Ptr pclPc2SegPrb = workItem.segmentationCloud;

        /* Extract the segmentation uncertainties from the image */
        cv::Mat segImageUncertainity = workItem.uncertaintyImage;

        /* Init an object of point clouds for seperated classes */
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> p_clsCloudPtrs;

        /*!
         * Seperate points into classes and filtering based on thresholds.
         *
         * clsCloudPtrs contains the points from the segmented point cloud
         * seperated into a list of point clouds with the index of the list
         * representing the semantic type of each point.
         */
        threshSeparatePointCloud(pclPc2SegPrb,
                                 segImageUncertainity,
                                 p_clsCloudPtrs,
                                 p_thisKeyFramePointCloud);

        /*!
         * Diagnostic visibility for a previously-silent failure mode: if the
         * model finds no (or very few) confident WALL-class pixels in this
         * keyframe -- e.g. a close, texture-poor, low-context wall filling
         * the frame -- getPlanesFromClassClouds() below simply `continue`s
         * past an empty/undersized class cloud with no trace anywhere. That
         * made "the segmenter found nothing this frame" indistinguishable
         * from "there was nothing to find" in every existing log. Class
         * index 1 is WALL (utils::utils::Utils::getPlaneTypeFromClassId). Gate
         * on a small count, not just empty, so this stays quiet on ordinary
         * frames.
         */
        constexpr std::size_t WALL_CLASS_INDEX               = 1U;
        constexpr std::size_t WALL_SILENT_DROP_LOG_THRESHOLD = 50U;
        if (p_clsCloudPtrs.size() > WALL_CLASS_INDEX &&
            p_clsCloudPtrs[WALL_CLASS_INDEX]->size() <
                WALL_SILENT_DROP_LOG_THRESHOLD)
        {
            Eigen::Vector3f cameraCenter_World{};
            if (p_thisKeyFrame->getCameraCenter(cameraCenter_World) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemSeg] KF#" << p_thisKeyFrame->id
                      << " wall-class points after confidence gating: "
                      << p_clsCloudPtrs[WALL_CLASS_INDEX]->size()
                      << " (camera at " << cameraCenter_World.x() << ','
                      << cameraCenter_World.y() << ',' << cameraCenter_World.z()
                      << ')' << std::endl;
        }

        /*!
         * clear pointclouds as they are no longer needed and consume
         * significant memory. also
         */
        if (p_thisKeyFrame->clearPointCloud() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
         * @note        Keep the most recent few keyframes intact because
         *              keyframes may be processed slightly out of order. Once a
         *              keyframe is older than the buffer window, its raw and
         *              classified point-cloud data are no longer needed and can
         *              be released to reduce memory usage.
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

                KeyFrame *p_keyFrame = p_atlas->getKeyFrameById(keyFrameId);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr
                    keyFrameGetCurrentFramePointCloud{};
                if ((p_keyFrame != nullptr) &&
                    p_keyFrame->getCurrentFramePointCloud(
                        keyFrameGetCurrentFramePointCloud) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: clearPointCloud returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame->clearClsClouds() !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
         * @note        Does not define the semantic type the plane is
         */
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>>
            p_clsPlanes = getPlanesFromClassClouds(p_clsCloudPtrs);

        /* Set the class specific point clouds to the keyframe */
        if (p_thisKeyFrame->setCurrentClsCloudPtrs(p_clsCloudPtrs) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            std::unique_lock<std::mutex> semanticUpdateLock =
                p_atlas->acquireSemanticUpdateLock();

            /*!
             * Plane extraction runs outside the semantic transaction. A map
             * merge may therefore invalidate the source keyframe or transfer
             * it away from the Atlas current map while inference is running.
             * Revalidate the source only after acquiring the transaction lock
             * so stale output cannot recreate observations in the merged map.
             */
            Map *p_currentMap = p_atlas->getCurrentMap();

            bool thisKeyFrameIsBad2{};
            if (!(p_thisKeyFrame == nullptr) &&
                p_thisKeyFrame->isBad(thisKeyFrameIsBad2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_thisKeyFrameMap2 = nullptr;
            if (!(p_thisKeyFrame == nullptr || thisKeyFrameIsBad2) &&
                p_thisKeyFrame->getMap(p_thisKeyFrameMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                recordTerminalOutcome(workItem.keyFrameId,
                                      TerminalOutcome::STALE_MAP);
                continue;
            }

            /* Add the planes to Atlas. */
            updatePlaneData(p_thisKeyFrame, p_clsPlanes);
        }
        recordTerminalOutcome(workItem.keyFrameId, TerminalOutcome::ACCEPTED);
    }
}

} // namespace core
} // namespace vs_graphs
