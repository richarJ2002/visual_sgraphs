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

#ifndef SEMANTICSEG_H
#define SEMANTICSEG_H

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Utils.h"

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

    bool geoRuns;

    Atlas *p_atlas;

    std::mutex mMutexNewKFs;

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

    void recordTerminalOutcome(std::uint64_t   keyFrameId,
                               TerminalOutcome outcome);

    // System parameters
    types::SystemParams *p_sysParams;

    // Shutdown control (LocalMapping-style handshake)
    std::mutex mMutexFinish;
    bool       finishRequested = false;
    bool       finished        = false;
    bool       checkFinish();
    void       setFinish();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticSegmentation(Atlas *pAtlas);

    // Semantic segmentation frame buffer processing
    void addSegmentedFrameToBuffer(
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

    /*!
     * @brief       Returns processing counters used by mission health and
     *              simulation lockstep. Safe to call from any thread.
     */
    ProcessingStats getProcessingStats();

    /*!
     * @brief       Segments the point cloud into class specific point clouds
     *              and enriches them with the current keyframe point cloud.
     *
     * @param[in]   pclPc2SegPrb
     *              Contains semantic class probablilities for every pixel
     *              or point in the segmented point cloud.
     *
     * @param[in]   segImgUncertainity
     *              An image containing the uncertainty of each pixel.
     *
     * @param[out]  clsCloudPtrs
     *              Output vector where each index contains a point cloud for
     *              each class.
     *
     *                  x, y, z -> 3D Coordinates
     *                  r, g, b -> Original RGB value of point
     *                  a       -> Confidence value
     *
     *              (Index indicates the semantic type of the cloud)
     *
     * @param       thisKFPointCloud
     *              the current keyframe point cloud
     */
    void threshSeparatePointCloud(
        pcl::PCLPointCloud2::Ptr pclPc2SegPrb,
        cv::Mat                 &segImgUncertainity,
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr         &thisKFPointCloud);

    /*!
     * @brief       Gets all planes for each class specific point cloud using
     *              RANSAC. Will perform filtering of point clouds, then
     *              extract the planes using RANSAC. Important to note that the
     *              plane semantics are not set in this method.
     *
     * @param[in]   clsCloudPtrs
     *              the class specific point clouds
     *
     * @param[in]   minCloudSize
     *              the minimum size of the point cloud to be segmented
     *
     * @return      A vector of extracted planes
     */
    std::vector<std::vector<
        std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
        getPlanesFromClassClouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs);

    /*!
     * @brief       Adds the planes to the Atlas
     *
     * @param       clsPlanes
     *              the planes to be added
     *
     * @param       clsConfs
     *              the confidence of the class predictions
     */
    void updatePlaneData(
        KeyFrame *pKF,
        std::vector<
            std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                  Eigen::Vector4d>>> &clsPlanes);

    /*!
     * @brief       Updates the map plane
     *
     * @param       planeId
     *              the plane id
     *
     * @param       clsId
     *              the class id
     *
     * @param       confidence
     *              the confidence of the class predictions
     */
    void updatePlaneSemantics(int planeId, int clsId, double confidence);

    // Shutdown control
    void requestFinish();
    bool isFinished();

    // Running the thread
    void run();
};
} // namespace core
} // namespace vs_graphs

#endif // SEMANTICSEG_H
