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
 * @file         SemanticSegmentation.cc
 *
 * @brief        Implements segmentation in SemanticSegmentation.h.
 */

#include "SemanticSegmentation.h"

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
            std::lock_guard<std::mutex> lock(mMutexNewKFs);
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
        if (p_thisKeyFrame == nullptr || p_thisKeyFrame->isBad())
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

        Map *p_activeMap = p_atlas->getCurrentMap();
        if (p_activeMap == nullptr ||
            p_activeMap->getId() != workItem.sourceMapId ||
            p_thisKeyFrame->getMap() != p_activeMap)
        {
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::STALE_MAP);
            continue;
        }

        /* Extract point cloud from keyframe */
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisKFPointCloud =
            p_thisKeyFrame->getCurrentFramePointCloud();

        /* If no point cloud in keyframe, skip to next frame */
        if (thisKFPointCloud == nullptr)
        {
            std::cerr << "[SemSeg] Skipping keyframe " << p_thisKeyFrame->mnId
                      << ": the RGB-D point cloud is unavailable." << std::endl;
            recordTerminalOutcome(workItem.keyFrameId,
                                  TerminalOutcome::MISSING_CLOUD);
            continue;
        }

        /* Extract the segmentation probabilities from the image */
        pcl::PCLPointCloud2::Ptr pclPc2SegPrb = workItem.segmentationCloud;

        /* Extract the segmentation uncertainties from the image */
        cv::Mat segImgUncertainity = workItem.uncertaintyImage;

        /* Init an object of point clouds for seperated classes */
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clsCloudPtrs;

        /*!
         * Seperate points into classes and filtering based on thresholds.
         *
         * clsCloudPtrs contains the points from the segmented point cloud
         * seperated into a list of point clouds with the index of the list
         * representing the semantic type of each point.
         */
        threshSeparatePointCloud(pclPc2SegPrb,
                                 segImgUncertainity,
                                 clsCloudPtrs,
                                 thisKFPointCloud);

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
        constexpr std::size_t kWallClassIndex          = 1U;
        constexpr std::size_t kWallSilentDropLogThresh = 50U;
        if (clsCloudPtrs.size() > kWallClassIndex &&
            clsCloudPtrs[kWallClassIndex]->size() < kWallSilentDropLogThresh)
        {
            const Eigen::Vector3f cameraCenter_World =
                p_thisKeyFrame->getCameraCenter();
            std::cout << "[SemSeg] KF#" << p_thisKeyFrame->mnId
                      << " wall-class points after confidence gating: "
                      << clsCloudPtrs[kWallClassIndex]->size() << " (camera at "
                      << cameraCenter_World.x() << ',' << cameraCenter_World.y()
                      << ',' << cameraCenter_World.z() << ')' << std::endl;
        }

        /*!
         * clear pointclouds as they are no longer needed and consume
         * significant memory. also
         */
        p_thisKeyFrame->clearPointCloud();

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
        if (p_thisKeyFrame->mnId - lastProcessedKeyFrameId > 5)
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
                std::lock_guard<std::mutex> lock(mMutexNewKFs);
                for (const WorkItem &bufferedItem : segmentedImageBuffer)
                {
                    pendingKeyFrameIds.insert(bufferedItem.keyFrameId);
                }
            }

            for (unsigned long int i = lastProcessedKeyFrameId + 1;
                 i < p_thisKeyFrame->mnId - 5;
                 i++)
            {
                if (pendingKeyFrameIds.count(i) > 0U)
                {
                    continue;
                }

                KeyFrame *pKF = p_atlas->getKeyFrameById(i);
                if (pKF != nullptr &&
                    pKF->getCurrentFramePointCloud() != nullptr)
                {
                    pKF->clearPointCloud();
                    pKF->clearClsClouds();
                }
            }
            lastProcessedKeyFrameId = p_thisKeyFrame->mnId - 5;
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
            clsPlanes = getPlanesFromClassClouds(clsCloudPtrs);

        /* Set the class specific point clouds to the keyframe */
        p_thisKeyFrame->setCurrentClsCloudPtrs(clsCloudPtrs);

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

            if (p_thisKeyFrame == nullptr || p_thisKeyFrame->isBad() ||
                p_thisKeyFrame->getMap() != p_currentMap)
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
            updatePlaneData(p_thisKeyFrame, clsPlanes);
        }
        recordTerminalOutcome(workItem.keyFrameId, TerminalOutcome::ACCEPTED);
    }
}

} // namespace core
} // namespace vs_graphs
