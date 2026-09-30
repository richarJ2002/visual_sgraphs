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

class SemanticSegmentation
{
  public:
    /*!
     * @brief       Lock-free counters plus a coherent queue-depth sample for
     *              processing-aware simulation lockstep.
     */
    struct ProcessingStats
    {
        std::uint64_t enqueuedCount{0U};
        std::uint64_t dequeuedCount{0U};
        std::uint64_t terminalCount{0U};
        std::uint64_t acceptedCount{0U};
        std::uint64_t droppedCount{0U};
        std::uint64_t missingKeyFrameCount{0U};
        std::uint64_t missingCloudCount{0U};
        std::uint64_t staleMapCount{0U};
        std::uint64_t lastTerminalKeyFrameId{0U};
        std::uint32_t queueDepth{0U};
        std::uint32_t queueHighWatermark{0U};
    };

  private:
    enum class TerminalOutcome
    {
        ACCEPTED,
        QUEUE_DROPPED,
        MISSING_KEYFRAME,
        MISSING_CLOUD,
        STALE_MAP
    };

    struct WorkItem
    {
        std::uint64_t            keyFrameId{0U};
        std::uint64_t            sourceMapId{0U};
        cv::Mat                  uncertaintyImage;
        pcl::PCLPointCloud2::Ptr segmentationCloud;
    };

    static constexpr std::size_t MAX_BUFFERED_WORK_ITEMS = 32U;

    bool isGeometricSegmentationRunning;

    Atlas *p_atlas;

    std::mutex newKeyFramesMutex;

    // Four bytes per class probability - refer to scene_segment_ros
    const uint8_t bytesPerClassProb = 4;

    unsigned long int lastProcessedKeyFrameId = 0;

    std::deque<WorkItem> segmentedImageBuffer;

    std::atomic<std::uint64_t> enqueuedCount{0U};
    std::atomic<std::uint64_t> dequeuedCount{0U};
    std::atomic<std::uint64_t> terminalCount{0U};
    std::atomic<std::uint64_t> acceptedCount{0U};
    std::atomic<std::uint64_t> droppedCount{0U};
    std::atomic<std::uint64_t> missingKeyFrameCount{0U};
    std::atomic<std::uint64_t> missingCloudCount{0U};
    std::atomic<std::uint64_t> staleMapCount{0U};
    std::atomic<std::uint64_t> lastTerminalKeyFrameId{0U};
    std::atomic<std::uint32_t> queueHighWatermark{0U};

    [[nodiscard]] SemanticSegmentationStatus
        recordTerminalOutcome(std::uint64_t   keyFrameId_in,
                              TerminalOutcome outcome_in);

    // System parameters
    types::SystemParams *p_sysParams;

    // Shutdown control (LocalMapping-style handshake)
    std::mutex finishMutex;
    bool       isFinishRequested = false;
    bool       hasFinished       = false;
    [[nodiscard]] SemanticSegmentationStatus
        checkFinish(bool &isFinishRequested_out);
    [[nodiscard]] SemanticSegmentationStatus setFinish();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticSegmentation(Atlas *p_atlas_in);

    // Semantic segmentation frame buffer processing
    [[nodiscard]] SemanticSegmentationStatus addSegmentedFrameToBuffer(
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *p_tuple_in);

    /*!
     * @brief       Returns processing counters used by mission health and
     *              simulation lockstep. Safe to call from any thread.
     */
    [[nodiscard]] SemanticSegmentationStatus getProcessingStats(
        SemanticSegmentation::ProcessingStats &processingStats_out);

    /*!
     * @brief       Segments the point cloud into class specific point clouds
     *              and enriches them with the current keyframe point cloud.
     *
     * @param[in]   p_pclPc2SegPrb_in
     *              Contains semantic class probablilities for every pixel
     *              or point in the segmented point cloud.
     *
     * @param[in]   segImageUncertainity_in
     *              An image containing the uncertainty of each pixel.
     *
     * @param[out]  p_clsCloudPtrs_out
     *              Output vector where each index contains a point cloud for
     *              each class.
     *
     *                  x, y, z -> 3D Coordinates
     *                  r, g, b -> Original RGB value of point
     *                  a       -> Confidence value
     *
     *              (Index indicates the semantic type of the cloud)
     *
     * @param       p_thisKeyFramePointCloud_in
     *              the current keyframe point cloud
     */
    [[nodiscard]] SemanticSegmentationStatus threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr p_pclPc2SegPrb_in,
        cv::Mat                 &segImageUncertainity_in,
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_clsCloudPtrs_out,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            &p_thisKeyFramePointCloud_in);

    /*!
     * @brief       Gets all planes for each class specific point cloud using
     *              RANSAC. Will perform filtering of point clouds, then
     *              extract the planes using RANSAC. Important to note that the
     *              plane semantics are not set in this method.
     *
     * @param[in]   p_clsCloudPtrs_in
     *              the class specific point clouds
     *
     * @param[out] planesFromClassClouds_out A vector of extracted planes
     * @return SEMANTIC_SEGMENTATION_STATUS_SUCCESS.
     */
    [[nodiscard]] SemanticSegmentationStatus getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &p_clsCloudPtrs_in,
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>>
            &planesFromClassClouds_out);

    /*!
     * @brief       Adds the planes to the Atlas
     *
     * @param       p_keyFrame_in
     *              the key frame the planes were observed in
     *
     * @param       p_clsPlanes_in
     *              the planes to be added
     *
     * @return      SEMANTIC_SEGMENTATION_STATUS_SUCCESS
     */
    [[nodiscard]] SemanticSegmentationStatus updatePlaneData(
        KeyFrame *p_keyFrame_in,
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>> &p_clsPlanes_in);

    /*!
     * @brief       Updates the map plane
     *
     * @param       planeId_in
     *              the plane id
     *
     * @param       clsId_in
     *              the class id
     *
     * @param       confidence_in
     *              the confidence of the class predictions
     */
    [[nodiscard]] SemanticSegmentationStatus
        updatePlaneSemantics(int    planeId_in,
                             int    clsId_in,
                             double confidence_in);

    // Shutdown control
    [[nodiscard]] SemanticSegmentationStatus requestFinish();
    [[nodiscard]] SemanticSegmentationStatus isFinished(bool &isFinished_out);

    // Running the thread
    void run();
};
} // namespace core
} // namespace vs_graphs

#endif // SEMANTICSEG_H
