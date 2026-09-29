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

#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

/*!
 * @file         Utils.h
 *
 * @brief        Declares geometric and point-cloud helpers for the estimator.
 */

#include "Atlas.h"
#include "Thirdparty/pcl_custom/WeightedSACSegmentation.hpp"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{
/*!
 * @brief        Static geometric and point-cloud helpers used across the
 *               estimator.
 */
class Utils
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using KeyFramePoseMap = std::map<
        KeyFrame *,
        g2o::Sim3,
        std::less<KeyFrame *>,
        Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>;

    // Variables
    /*!
     * @brief        Conversion factor from degrees to radians.
     */
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    /*!
     * @brief        Calculate the Euclidean distance between two points.
     *
     * @param[in]    point1_in
     *               First point.
     * @param[in]    point2_in
     *               Second point.
     *
     * @param[out] euclideanDistance_out Euclidean distance in the units of the
     * input points.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        calculateEuclideanDistance(const Eigen::Vector3f &point1_in,
                                   const Eigen::Vector3f &point2_in,
                                   double &euclideanDistance_out);

    /*!
     * @brief        Calculate the distance between a point and a plane.
     *
     * @param[in]    plane_in
     *               Plane equation.
     * @param[in]    point_in
     *               Given point.
     *
     * @param[out] distance_out Absolute distance in the input length units.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        calculateDistancePointToPlane(const Eigen::Vector4d &plane_in,
                                      const Eigen::Vector3d &point_in,
                                      double                &distance_out);

    /*!
     * @brief        Calculates the intersection point of a line and a
     *               plane. The line shall not be parallel to the plane.
     *
     * @param[in]    plane_in
     *               Plane equation.
     * @param[in]    lineStart_in
     *               Start point of the line.
     * @param[in]    lineEnd_in
     *               End point of the line.
     *
     * @param[out] intersection_out Point where the line meets the plane.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        lineIntersectsPlane(const Eigen::Vector4d &plane_in,
                            const Eigen::Vector3d &lineStart_in,
                            const Eigen::Vector3d &lineEnd_in,
                            Eigen::Vector3d       &intersection_out);

    /*!
     * @brief        Checks to see if two planes are apart enough from
     *               each other, given a threshold.
     *
     * @param[in]    p_plane1_in
     *               First plane; may be null.
     * @param[in]    p_plane2_in
     *               Second plane; may be null.
     * @param[in]    threshold_in
     *               Threshold value for perpendicularity, in metres.
     *
     * @param[out] arePlanesApartEnough_out True when both planes are valid and
     * their perpendicular separation exceeds the threshold.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        arePlanesApartEnough(const geometric::Plane *p_plane1_in,
                             const geometric::Plane *p_plane2_in,
                             const double           &threshold_in,
                             bool                   &arePlanesApartEnough_out);

    /*!
     * @brief        Checks to see if two planes are perpendicular to each
     *               other or not.
     *
     * @param[in]    p_plane1_in
     *               First plane; shall be non-null.
     * @param[in]    p_plane2_in
     *               Second plane; shall be non-null.
     *
     * @param[out] arePlanesPerpendicular_out True when the inter-plane angle is
     * within the configured threshold of 90 degrees.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus arePlanesPerpendicular(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in,
        bool                                    &arePlanesPerpendicular_out);

    /*!
     * @brief        Checks to see if two planes are parallel to each other
     *               or not.
     *
     * @param[in]    p_plane1_in
     *               First plane; shall be non-null.
     * @param[in]    p_plane2_in
     *               Second plane; shall be non-null.
     *
     * @param[out] arePlanesParallel_out True when the plane normals align
     * within the configured threshold.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        arePlanesParallel(const vs_graphs::core::geometric::Plane *p_plane1_in,
                          const vs_graphs::core::geometric::Plane *p_plane2_in,
                          bool &arePlanesParallel_out);

    /*!
     * @brief        Checks to see if two planes are facing each other or
     *               not.
     *
     * @param[in]    p_plane1_in
     *               First plane (small plane, e.g., door); may be null.
     * @param[in]    p_plane2_in
     *               Second plane (big plane, e.g., wall); may be null.
     *
     * @param[out] arePlanesFacingEachOther_out True when the two valid planes
     * face each other across a gap.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus arePlanesFacingEachOther(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in,
        bool                                    &arePlanesFacingEachOther_out);

    /*!
     * @brief        Returns the planes that are facing each other from
     *               the given list.
     *
     * @param[in]    planes_in
     *               List of planes to be checked.
     *
     * @param[out] facingPlanes_out Facing plane pairs whose separation passes
     * the configured minimum. Pointers are borrowed from the input list.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus getFacingPlanes(
        const std::vector<geometric::Plane *> &planes_in,
        std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                              vs_graphs::core::geometric::Plane *>>
            &facingPlanes_out);

    /*!
     * @brief        Corrects the given plane equations to apply
     *               calculations.
     *
     * @param[in]    plane_in
     *               Input plane.
     *
     * @param[out] planeDirection_out Plane with negated coefficients when the
     * offset is positive; unchanged otherwise.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        correctPlaneDirection(const Eigen::Vector4d &plane_in,
                              Eigen::Vector4d       &planeDirection_out);

    /*!
     * @brief        Converts the plane equation from local to global.
     *
     * @param[in]    keyframePose_in
     *               Pose of the current keyframe.
     * @param[in]    plane_in
     *               Plane equation in the local map.
     *
     * @param[out] transformedPlane_out Transformed plane coefficients.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        applyPoseToPlane(const Eigen::Matrix4d &keyframePose_in,
                         const g2o::Plane3D    &plane_in,
                         g2o::Plane3D          &transformedPlane_out);

    /*!
     * @brief        Gets the centeroid of a set or cluster of points.
     *
     * @param[in]    points_in
     *               Given cluster of points.
     *
     * @param[out] centroid_out Mean of the points; zero when the input is
     * empty.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        computeCentroidFromPoints(const std::vector<Eigen::Vector3d> &points_in,
                                  Eigen::Vector3d &centroid_out);

    /*!
     * @brief        Downsamples the pointclouds based on the given leaf
     *               size. PointT shall be a PCL point type.
     *
     * @param[in]    p_cloud_in
     *               Pointcloud to be downsampled.
     * @param[in]    leafSize_in
     *               Leaf size for downsampling, in the cloud's length
     *               units.
     * @param[in]    minPointsPerVoxel_in
     *               Minimum points required to keep a voxel.
     * @param[out]   p_downsampledCloud_out
     *               Owning pointer to the new downsampled cloud.
     *
     * @return       UTILS_STATUS_SUCCESS.
     */
    template <typename PointT>
    [[nodiscard]] static UtilsStatus pointcloudDownsample(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        const float                                  leafSize_in,
        const unsigned int                           minPointsPerVoxel_in,
        typename pcl::PointCloud<PointT>::Ptr       &p_downsampledCloud_out);

    /*!
     * @brief        Filters the pointclouds based on the given min/max
     *               distance acceptable. PointT shall be a PCL point type.
     *
     *               Keeps the points whose sensor-forward distance lies
     *               within the configured range.
     *
     * @param[in]    p_cloud_in
     *               Pointcloud to be filtered.
     * @param[out]   p_filteredCloud_out
     *               Owning pointer to the new filtered cloud.
     *
     * @return       UTILS_STATUS_SUCCESS.
     */
    template <typename PointT>
    [[nodiscard]] static UtilsStatus pointcloudDistanceFilter(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        typename pcl::PointCloud<PointT>::Ptr       &p_filteredCloud_out);

    /*!
     * @brief        Removes the points that are farther away from their
     *               neighbors. PointT shall be a PCL point type.
     *
     * @param[in]    p_cloud_in
     *               Pointcloud to be filtered.
     * @param[in]    meanThreshold_in
     *               Mean threshold for neighbor points.
     * @param[in]    stdDevThreshold_in
     *               Standard deviation threshold for neighbor points.
     * @param[out]   p_filteredCloud_out
     *               Owning pointer to the new filtered cloud.
     *
     * @return       UTILS_STATUS_SUCCESS.
     */
    template <typename PointT>
    [[nodiscard]] static UtilsStatus pointcloudOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        const int                                    meanThreshold_in,
        const float                                  stdDevThreshold_in,
        typename pcl::PointCloud<PointT>::Ptr       &p_filteredCloud_out);

    /*!
     * @brief        Computes the width and height of a plane given its
     *               point cloud.
     *
     * @param[in]    p_cloud_in
     *               Borrowed point cloud of the plane.
     *
     * @param[out] planeWidthHeight_out Plane width and height in the cloud's
     * length units.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus computePlaneWidthHeight(
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud_in,
        std::pair<double, double>                   &planeWidthHeight_out);

    /*!
     * @brief        Performs PCL ransac to get the plane equations from
     *               a given point cloud. SegmentationType names the PCL
     *               segmentation applied to PointT.
     *
     * @param[in,out] cloud_inout
     *               Input cloud; planes are extracted from it.
     * @param[out]   planes_out
     *               Extracted plane clouds paired with their plane
     *               equations.
     *
     * @return       UTILS_STATUS_SUCCESS.
     */
    template <typename PointT, template <typename> class SegmentationType>
    [[nodiscard]] static UtilsStatus ransacPlaneFitting(
        typename pcl::PointCloud<PointT>::Ptr   &cloud_inout,
        std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr,
                              Eigen::Vector4d>> &planes_out);

    /*!
     * @brief        Checks to see if the given point is on the plane or
     *               not.
     *
     * @param[in]    planeEquation_in
     *               Plane equation.
     * @param[in]    p_mapPoint_in
     *               Borrowed point to be checked; shall be non-null.
     *
     * @param[out] isOnPlane_out True when the point lies within the configured
     * distance of the plane.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        pointOnPlane(Eigen::Vector4d planeEquation_in,
                     MapPoint       *p_mapPoint_in,
                     bool           &isOnPlane_out);

    /*!
     * @brief        Associates given planes with the mapped planes.
     *
     * @param[in]    mappedPlanes_in
     *               Mapped planes.
     * @param[in]    observedPlane_in
     *               Given plane (in the same frame as the
     *               keyframePose_in, global if keyframePose_in is
     *               identity).
     * @param[in]    p_observedCloud_in
     *               Given plane's support cloud.
     * @param[in]    keyframePose_in
     *               Pose of the current keyframe.
     * @param[in]    observedPlaneType_in
     *               Observed plane variant.
     * @param[in]    threshold_in
     *               Threshold value for association.
     * @param[in]    maximumFiniteCloudDistance_m_in
     *               Optional finite-cloud gap override, in metres.
     * @param[in]    observationOrigin_World_m_in
     *               Optional observing camera origin in the world frame,
     *               in metres, used to keep opposite wall faces separate.
     *
     * @param[out] matchedPlaneId_out Plane id of the mapped plane.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus associatePlanes(
        const vector<geometric::Plane *>            &mappedPlanes_in,
        g2o::Plane3D                                 observedPlane_in,
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_observedCloud_in,
        const Eigen::Matrix4d                       &keyframePose_in,
        const geometric::Plane::PlaneVariant         observedPlaneType_in,
        const float                                  threshold_in,
        int                                         &matchedPlaneId_out,
        const float maximumFiniteCloudDistance_m_in = -1.0F,
        const std::optional<Eigen::Vector3d> &observationOrigin_World_m_in =
            std::nullopt);

    /*!
     * @brief        Clusters the point cloud into separate clouds based
     *               on the plane detection.
     *
     * @param[in]    p_cloud_in
     *               Point cloud to be clustered.
     * @param[in,out] clusterIndices_inout
     *               Vector of point indices for each cluster.
     */
    [[nodiscard]] static UtilsStatus clusterPlaneClouds(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &p_cloud_in,
        std::vector<pcl::PointIndices>                &clusterIndices_inout);

    /*!
     * @brief        Re-associates semantically classified planes if they get
     *               closer after optimization
     *
     * @param[in]    p_atlas_in
     *               A pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus
        reAssociateSemanticPlanes(Atlas *p_atlas_in);

    /*!
     * @brief        Re-associates semantically classified planes if they get
     *               closer after optimization
     *
     * @param[in]    p_atlas_in
     *               a pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus reAssociateRooms(Atlas *p_atlas_in);

    /*!
     * @brief        Fuses duplicate confirmed rooms introduced by a
     *               map merge.
     *
     *              Only rooms imported from the obsolete submap are
     *              considered for retirement. A candidate must agree
     *              with a pre-existing room in type, centroid
     *              proximity, shared-wall side, and unobstructed
     *              free-space geometry. This prevents adjacent rooms
     *              on opposite sides of a wall from being collapsed.
     *
     * @param[in,out] p_map_inout
     *                Surviving map whose room graph is reconciled.
     * @param[in]    importedRooms_in
     *               Rooms transferred from the obsolete submap.
     */
    [[nodiscard]] static UtilsStatus fuseDuplicateRoomsAfterMerge(
        Map                                 *p_map_inout,
        const std::vector<semantic::Room *> &importedRooms_in);

    /*!
     * @brief        Fuses duplicate passage entities after optimization or map
     *               merge and rewires all room/keyframe associations.
     *
     * @param[in]     p_atlas_in
     *                 Atlas whose active semantic graph is reconciled.
     */
    [[nodiscard]] static UtilsStatus reAssociatePassages(Atlas *p_atlas_in);

    /*!
     * @brief        Propagates keyframe pose corrections to the
     *               semantic graph.
     *
     *              A local or global bundle adjustment changes
     *              keyframe poses and mapped points, but semantic
     *              entities are stored independently in the world
     *              frame. This method applies the corresponding
     *              old-world to corrected-world deformation exactly
     *              once to planes, markers, passages, rooms, floors,
     *              and free-space skeleton geometry.
     *
     * @param[in,out] p_map_inout
     *                Map whose semantic geometry is updated.
     * @param[in]    keyFramePosesBefore_WorldToCamera_in
     *               World-to-camera poses immediately before
     *               optimization.
     * @param[in]    keyFramePosesAfter_WorldToCamera_in
     *               World-to-camera poses immediately after
     *               optimization.
     * @param[in]    fallbackTransform_oldWorldToNewWorld_in
     *               Transform used only when no valid keyframe
     *               deformation node can anchor an entity. Pass
     *               identity for an in-place BA.
     */
    [[nodiscard]] static UtilsStatus propagateSemanticPoseCorrections(
        Map                   *p_map_inout,
        const KeyFramePoseMap &keyFramePosesBefore_WorldToCamera_in,
        const KeyFramePoseMap &keyFramePosesAfter_WorldToCamera_in,
        const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in);

    /*!
     * @brief        Consolidates redundant single-wall provisional structural
     *               elements into a room supported by a free-space cluster.
     *
     * @param[in,out] p_selectedRoom_inout
     *               The cluster-backed room which has absorbed the walls
     *
     * @param[in]    p_atlas_in
     *               a pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus consolidateProvisionalRooms(
        vs_graphs::core::semantic::Room *p_selectedRoom_inout,
        Atlas                           *p_atlas_in);

    /*!
     * @brief        Gets the PlaneVariant type from the class id.
     *
     * @param[in]    classId_in
     *               Class id.
     *
     * @param[out] planeTypeFromClassId_out Ground, wall, door or window type
     * for ids 0 to 3; UNDEFINED otherwise.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        getPlaneTypeFromClassId(int classId_in,
                                vs_graphs::core::geometric::Plane::PlaneVariant
                                    &planeTypeFromClassId_out);

    /*!
     * @brief        Gets the class id from the PlaneVariant type.
     *
     * @param[in]    planeType_in
     *               PlaneVariant type.
     *
     * @param[out] classIdFromPlaneType_out Class id 0 to 3 for known types; -1
     * otherwise.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus getClassIdFromPlaneType(
        vs_graphs::core::geometric::Plane::PlaneVariant planeType_in,
        int &classIdFromPlaneType_out);

    /*!
     * @brief        Computes the rigid transform mapping map A into map B
     *               using Horn's closed-form solution on oriented
     *               wall-normal correspondences.
     *
     *               The transform maps points from map A's frame into map
     *               B's frame such that p_B = T * p_A.
     *
     * @param[in]    normalsA_in
     *               Unit wall normals of map A.
     * @param[in]    centroidsA_in
     *               Wall centroids of map A.
     * @param[in]    normalsB_in
     *               Unit wall normals of map B.
     * @param[in]    centroidsB_in
     *               Wall centroids of map B.
     *
     * @param[out] mapTransform_Horn_out Rigid transform from map A to map B, or
     * identity when the input data is invalid.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus computeMapTransform_Horn(
        const std::vector<Eigen::Vector3d> &normalsA_in,
        const std::vector<Eigen::Vector3d> &centroidsA_in,
        const std::vector<Eigen::Vector3d> &normalsB_in,
        const std::vector<Eigen::Vector3d> &centroidsB_in,
        Eigen::Isometry3d                  &mapTransform_Horn_out);

    /*!
     * @brief        Pairs the walls of two matched rooms by greedy
     *               best-first normal alignment.
     *
     * @param[in]    p_roomA_in
     *               Room in the surviving map; may be null.
     * @param[in]    p_roomB_in
     *               Same physical room in the map to be absorbed; may
     *               be null.
     * @param[in,out] normalsA_inout
     *               Matched wall normals of room A.
     * @param[in,out] centroidsA_inout
     *               Matched wall centroids of room A.
     * @param[in,out] normalsB_inout
     *               Matched wall normals of room B.
     * @param[in,out] centroidsB_inout
     *               Matched wall centroids of room B.
     *
     * @param[out] matchedWallCount_out Number of accepted wall pairs; zero when
     * either room is null.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        matchWallsBetweenRooms(const semantic::Room         *p_roomA_in,
                               const semantic::Room         *p_roomB_in,
                               std::vector<Eigen::Vector3d> &normalsA_inout,
                               std::vector<Eigen::Vector3d> &centroidsA_inout,
                               std::vector<Eigen::Vector3d> &normalsB_inout,
                               std::vector<Eigen::Vector3d> &centroidsB_inout,
                               std::size_t &matchedWallCount_out);

    /*!
     * @brief        Gathers wall correspondences across two maps using
     *               their shared room identity tags.
     *
     * @param[in]    p_mapA_in
     *               Surviving map; may be null.
     * @param[in]    p_mapB_in
     *               Map to be absorbed; may be null.
     * @param[in,out] normalsA_inout
     *               Matched wall normals of map A.
     * @param[in,out] centroidsA_inout
     *               Matched wall centroids of map A.
     * @param[in,out] normalsB_inout
     *               Matched wall normals of map B.
     * @param[in,out] centroidsB_inout
     *               Matched wall centroids of map B.
     *
     * @param[out] hasEnoughCorrespondences_out True when at least three valid
     * correspondences were collected.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus collectCorrespondingWalls(
        Map                          *p_mapA_in,
        Map                          *p_mapB_in,
        std::vector<Eigen::Vector3d> &normalsA_inout,
        std::vector<Eigen::Vector3d> &centroidsA_inout,
        std::vector<Eigen::Vector3d> &normalsB_inout,
        std::vector<Eigen::Vector3d> &centroidsB_inout,
        bool                         &hasEnoughCorrespondences_out);

    /*!
     * @brief        Calculates the soft-min approximation of the given
     *               values. Soft-min estimates segment quality from
     *               per-pixel uncertainties.
     *
     * @param[in]    values_in
     *               Input values; shall hold at least one value.
     *
     * @param[out] softMin_out Soft-min value.
     * @return UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus calcSoftMin(vector<double> &values_in,
                                                 double         &softMin_out);
};
} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs

#endif // UTILS_H
