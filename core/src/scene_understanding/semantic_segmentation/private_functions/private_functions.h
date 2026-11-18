/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  SemanticSegmentation translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of SemanticSegmentation.cc;
 *                  external linkage here is module-internal only. Names
 *                  are kept verbatim (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H

#include "configuration/types/types.h"
#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationStatusEnum.h"
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

/*!
 * @brief           Describes the strongest spatially connected part of a wall
 *                  cloud.
 */
struct WallComponentSupport
{
    /*!
     * @brief           Source-cloud indices forming the largest component.
     */
    std::vector<int> pointIndices;
    /*!
     * @brief           Number of finite points considered by clustering.
     */
    std::size_t      finitePointCount = 0U;
    /*!
     * @brief           Fraction of finite points in the largest component.
     */
    double           componentRatio = 0.0;
};

[[nodiscard]] SemanticSegmentationStatus findLargestWallComponent(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const double                                        clusterTolerance_m_in,
    WallComponentSupport &largestWallComponent_out);

/*!
 * @brief           Computes how well a plane patch fits its plane equation.
 *
 * @param[in]       p_planeCloud_in
 *                  Patch points, in the same frame as the equation.
 *
 * @param[in]       planeEquation_in
 *                  Plane (a, b, c, d) with a x + b y + c z + d = 0.
 *
 * @param[out]      fitRms_m_out
 *                  Root-mean-square point-to-plane distance in metres;
 *                  infinity for a null or empty cloud, a non-finite equation
 *                  or a zero normal.
 *
 * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticSegmentationStatus computePlaneFitRms(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector4d                              &planeEquation_in,
    double                                             &fitRms_m_out);

/*!
 * @brief           Checks that a wall patch is connected and large enough to
 *                  be a wall, and keeps only its connected support.
 *
 * @param[in]       wallSize_in
 *                  Size and connectivity thresholds.
 *
 * @param[in,out]   p_wallCloud_inout
 *                  Wall patch points (any frame: the check uses distances
 *                  only). On success, replaced by its largest connected
 *                  part when connectivity is enabled; unchanged otherwise.
 *
 * @param[out]      isLargeEnough_out
 *                  True when the patch passes every size and connectivity
 *                  threshold.
 *
 * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticSegmentationStatus checkWallPatchSize(
    const types::SystemParams::PlaneCheckpoint::WallSize &wallSize_in,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr              &p_wallCloud_inout,
    bool                                                 &isLargeEnough_out);

/*!
 * @brief           Weights a point by its depth: 255 nearer than the near
 *                  threshold, 45 beyond the far one, and in between falling
 *                  with the square root of the depth fraction.
 *
 * @param[in]       depth_m_in
 *                  Point depth along the camera axis, metres.
 *
 * @param[in]       nearDepth_m_in
 *                  Near threshold (pointcloud.distance_thresh first), metres.
 *
 * @param[in]       farDepth_m_in
 *                  Far threshold (pointcloud.distance_thresh second), metres;
 *                  greater than nearDepth_m_in.
 *
 * @param[out]      depthWeight_out
 *                  Weight in [45, 255].
 *
 * @return          SEMANTIC_SEGMENTATION_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticSegmentationStatus
    computeDepthWeight(const float   depth_m_in,
                       const float   nearDepth_m_in,
                       const float   farDepth_m_in,
                       std::uint8_t &depthWeight_out);

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H \
        */
