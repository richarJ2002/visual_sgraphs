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
 * @file            SemanticSegmentation.h
 *
 * @brief           Declares SemanticSegmentation, the worker thread that
 *                  applies image-segmentation labels to the mapped planes.
 */

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"
#include "SemanticSegmentationStatus.h"

#include <atomic>
#include <deque>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{
class Atlas;

/*!
 * @brief           Turns segmented camera images into wall, ground and other
 *                  planes in the atlas. Frames arrive in a bounded buffer and
 *                  are consumed by the thread running run().
 */
class SemanticSegmentation
{
  public:
    /*!
     * @brief           Lock-free counters plus a coherent queue-depth sample
     *                  for processing-aware simulation lockstep.
     */
    struct ProcessingStats
    {
        /*!
         * @brief           Number of frames added to the buffer.
         */
        std::uint64_t enqueuedCount{0U};

        /*!
         * @brief           Number of frames taken out of the buffer by run().
         */
        std::uint64_t dequeuedCount{0U};

        /*!
         * @brief           Number of frames that reached a final outcome; the
         *                  sum of the five outcome counts below.
         */
        std::uint64_t terminalCount{0U};

        /*!
         * @brief           Number of frames that were fully processed.
         */
        std::uint64_t acceptedCount{0U};

        /*!
         * @brief           Number of frames discarded because the buffer was
         *                  full.
         */
        std::uint64_t droppedCount{0U};

        /*!
         * @brief           Number of frames whose key frame was missing or bad.
         */
        std::uint64_t missingKeyFrameCount{0U};

        /*!
         * @brief           Number of frames that carried no segmentation cloud.
         */
        std::uint64_t missingCloudCount{0U};

        /*!
         * @brief           Number of frames whose key frame is not in the
         *                  active map.
         */
        std::uint64_t staleMapCount{0U};

        /*!
         * @brief           Key frame id of the frame that most recently reached
         *                  a final outcome; 0 before the first one.
         */
        std::uint64_t lastTerminalKeyFrameId{0U};

        /*!
         * @brief           Number of frames in the buffer when the snapshot was
         *                  taken.
         */
        std::uint32_t queueDepth{0U};

        /*!
         * @brief           Largest number of frames the buffer has held at
         *                  once.
         */
        std::uint32_t queueHighWatermark{0U};
    };

  private:
    /*!
     * @brief           How a buffered frame ended; recorded once per frame by
     *                  recordTerminalOutcome().
     */
    enum class TerminalOutcome
    {
        ACCEPTED,
        QUEUE_DROPPED,
        MISSING_KEYFRAME,
        MISSING_CLOUD,
        STALE_MAP
    };

    /*!
     * @brief           One segmented frame waiting in the buffer for run().
     */
    struct WorkItem
    {
        /*!
         * @brief           Id of the key frame the segmentation belongs to.
         */
        std::uint64_t keyFrameId{0U};

        /*!
         * @brief           Id of the map the key frame belonged to when the
         *                  frame was queued; the largest uint64 value when the
         *                  key frame or its map was not found.
         */
        std::uint64_t sourceMapId{0U};

        /*!
         * @brief           Per-pixel uncertainty image of the segmentation.
         */
        cv::Mat uncertaintyImage;

        /*!
         * @brief           Point cloud holding the class probabilities of every
         *                  point; may be null, which run() counts as
         *                  MISSING_CLOUD.
         */
        pcl::PCLPointCloud2::Ptr segmentationCloud;
    };

    /*!
     * @brief           Capacity of the buffer; when it is full the oldest frame
     *                  is dropped and counted as QUEUE_DROPPED.
     */
    static constexpr std::size_t MAX_BUFFERED_WORK_ITEMS = 32U;

    /*!
     * @brief           False only in the semantic-only mode of operation; run()
     *                  then decides itself whether a plane observation is large
     *                  enough to become a new map plane.
     */
    bool isGeometricSegmentationRunning;

    /*!
     * @brief           Borrowed pointer to the atlas that holds the maps and
     *                  key frames; set by the constructor.
     */
    Atlas *p_atlas;

    /*!
     * @brief           Protects segmentedImageBuffer.
     */
    std::mutex newKeyFramesMutex;

    /*!
     * @brief           Size in bytes of one class probability (a float) inside
     *                  a point of the segmentation cloud, as published by
     *                  scene_segment_ros.
     */
    const uint8_t bytesPerClassProb = 4;

    /*!
     * @brief           Key frame id up to which run() has already released the
     *                  classified clouds; key frames this far behind the one
     *                  being processed are no longer needed. 0 until the first
     *                  release.
     */
    unsigned long int lastProcessedKeyFrameId = 0;

    /*!
     * @brief           First-in first-out queue of frames waiting for run();
     *                  holds at most MAX_BUFFERED_WORK_ITEMS and is guarded by
     *                  newKeyFramesMutex.
     */
    std::deque<WorkItem> segmentedImageBuffer;

    /*!
     * @brief           Number of frames added to the buffer.
     */
    std::atomic<std::uint64_t> enqueuedCount{0U};

    /*!
     * @brief           Number of frames taken out of the buffer by run().
     */
    std::atomic<std::uint64_t> dequeuedCount{0U};

    /*!
     * @brief           Number of frames that reached a final outcome.
     */
    std::atomic<std::uint64_t> terminalCount{0U};

    /*!
     * @brief           Number of frames that were fully processed.
     */
    std::atomic<std::uint64_t> acceptedCount{0U};

    /*!
     * @brief           Number of frames discarded because the buffer was full.
     */
    std::atomic<std::uint64_t> droppedCount{0U};

    /*!
     * @brief           Number of frames whose key frame was missing or bad.
     */
    std::atomic<std::uint64_t> missingKeyFrameCount{0U};

    /*!
     * @brief           Number of frames that carried no segmentation cloud.
     */
    std::atomic<std::uint64_t> missingCloudCount{0U};

    /*!
     * @brief           Number of frames whose key frame is not in the active
     *                  map.
     */
    std::atomic<std::uint64_t> staleMapCount{0U};

    /*!
     * @brief           Key frame id of the frame that most recently reached a
     *                  final outcome; 0 before the first one.
     */
    std::atomic<std::uint64_t> lastTerminalKeyFrameId{0U};

    /*!
     * @brief           Largest number of frames the buffer has held at once.
     */
    std::atomic<std::uint32_t> queueHighWatermark{0U};

    /*!
     * @brief           Counts one final outcome for a frame and remembers its
     *                  key frame id.
     *
     * @param[in]       keyFrameId_in
     *                  Id of the key frame whose frame ended.
     *
     * @param[in]       outcome_in
     *                  How the frame ended.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus
        recordTerminalOutcome(std::uint64_t   keyFrameId_in,
                              TerminalOutcome outcome_in);

    /*!
     * @brief           Borrowed pointer to the global system parameters; set by
     *                  the constructor.
     */
    types::SystemParams *p_sysParams;

    // Shutdown control (LocalMapping-style handshake)
    /*!
     * @brief           Protects isFinishRequested and hasFinished.
     */
    std::mutex finishMutex;

    /*!
     * @brief           Set by requestFinish() to ask run() to leave its loop.
     */
    bool isFinishRequested = false;

    /*!
     * @brief           Set by setFinish() once run() has left its loop.
     */
    bool hasFinished = false;

    /*!
     * @brief           Tells whether a shutdown has been requested.
     *
     * @param[out]      isFinishRequested_out
     *                  True once requestFinish() has been called.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus
        checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief           Marks the processing thread as finished; called by run()
     *                  when it leaves its loop.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus setFinish();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the segmenter and reads the system parameters.
     *
     * @param[in]       p_atlas_in
     *                  Atlas that holds the maps and key frames; borrowed and
     *                  kept, must outlive this object.
     */
    SemanticSegmentation(Atlas *p_atlas_in);

    // Semantic segmentation frame buffer processing
    /*!
     * @brief           Queues one segmented frame for run(). When the buffer is
     *                  full the oldest queued frame is dropped and counted as
     *                  QUEUE_DROPPED.
     *
     * @param[in]       p_tuple_in
     *                  Key frame id, uncertainty image and segmentation cloud
     *                  of the frame; must not be null, only read here.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus addSegmentedFrameToBuffer(
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *p_tuple_in);

    /*!
     * @brief           Returns processing counters used by mission health and
     *                  simulation lockstep. Safe to call from any thread.
     */
    [[nodiscard]] SemanticSegmentationStatus getProcessingStats(
        SemanticSegmentation::ProcessingStats &processingStats_out);

    /*!
     * @brief           Segments the point cloud into class specific point
     *                  clouds and enriches them with the current keyframe point
     *                  cloud.
     *
     * @param[in]       p_pclPc2SegPrb_in
     *                  Contains semantic class probablilities for every pixel
     *                  or point in the segmented point cloud.
     *
     * @param[in]       segImageUncertainity_in
     *                  An image containing the uncertainty of each pixel.
     *
     * @param[out]      p_clsCloudPtrs_out
     *                  Output vector where each index contains a point cloud
     *                  for each class.
     *
     *                      x, y, z -> 3D Coordinates
     *                      r, g, b -> Original RGB value of point
     *                      a       -> Confidence value
     *
     *                  (Index indicates the semantic type of the cloud)
     *
     * @param[in]       p_thisKeyFramePointCloud_in
     *                  the current keyframe point cloud
     */
    [[nodiscard]] SemanticSegmentationStatus threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr p_pclPc2SegPrb_in,
        cv::Mat                 &segImageUncertainity_in,
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_clsCloudPtrs_out,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            &p_thisKeyFramePointCloud_in);

    /*!
     * @brief           Extracts the planes of every class point cloud with
     *                  RANSAC, after filtering each cloud by depth and thinning
     *                  it with a voxel grid. The plane semantics are not set
     *                  here.
     *
     * @param[in]       p_classCloudPtrs_in
     *                  One point cloud per semantic class. The vector itself is
     *                  not changed, but each cloud that keeps points after
     *                  filtering is overwritten with its filtered copy.
     *
     * @param[out]      planesFromClassClouds_out
     *                  For each class whose filtered cloud is not empty, the
     *                  extracted planes as pairs of the plane's points and its
     *                  equation (a, b, c, d); classes with an empty cloud get
     *                  no entry, so the index need not match
     *                  p_classCloudPtrs_in. Planes are searched only in clouds
     *                  with more than seg.pointcloudsThresh points.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_classCloudPtrs_in,
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>>
            &planesFromClassClouds_out);

    /*!
     * @brief           Adds the planes to the Atlas
     *
     * @param[in]       p_keyFrame_in
     *                  the key frame the planes were observed in
     *
     * @param[in]       p_clsPlanes_in
     *                  the planes to be added
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS
     */
    [[nodiscard]] SemanticSegmentationStatus updatePlaneData(
        KeyFrame *p_keyFrame_in,
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>> &p_clsPlanes_in);

    /*!
     * @brief           Updates the map plane
     *
     * @param[in]       planeId_in
     *                  the plane id
     *
     * @param[in]       clsId_in
     *                  the class id
     *
     * @param[in]       confidence_in
     *                  the confidence of the class predictions
     */
    [[nodiscard]] SemanticSegmentationStatus
        updatePlaneSemantics(int    planeId_in,
                             int    clsId_in,
                             double confidence_in);

    // Shutdown control
    /*!
     * @brief           Asks run() to leave its loop.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus requestFinish();

    /*!
     * @brief           Tells whether run() has left its loop.
     *
     * @param[out]      isFinished_out
     *                  True once setFinish() has been called.
     *
     * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS always.
     */
    [[nodiscard]] SemanticSegmentationStatus isFinished(bool &isFinished_out);

    // Running the thread
    void run();
};
} // namespace core
} // namespace vs_graphs

#endif // SEMANTICSEG_H
