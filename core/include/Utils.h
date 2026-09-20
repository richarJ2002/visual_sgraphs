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

#ifndef UTILS_H
#define UTILS_H

/*!
 * @file         Utils.h
 *
 * @brief        Declares geometric and point-cloud helpers for the estimator.
 */

#include "Atlas.h"
#include "Thirdparty/pcl_custom/WeightedSACSegmentation.hpp"
#include "Tracking.h"

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
     * @return       Euclidean distance in the units of the input points.
     */
    static double calculateEuclideanDistance(const Eigen::Vector3f &point1_in,
                                             const Eigen::Vector3f &point2_in);

    /*!
     * @brief        Calculate the distance between a point and a plane.
     *
     * @param[in]    plane_in
     *               Plane equation.
     * @param[in]    point_in
     *               Given point.
     *
     * @return       Absolute distance in the input length units.
     */
    static double
        calculateDistancePointToPlane(const Eigen::Vector4d &plane_in,
                                      const Eigen::Vector3d &point_in);

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
     * @return       Point where the line meets the plane.
     */
    static Eigen::Vector3d
        lineIntersectsPlane(const Eigen::Vector4d &plane_in,
                            const Eigen::Vector3d &lineStart_in,
                            const Eigen::Vector3d &lineEnd_in);

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
     * @return       True when both planes are valid and their
     *               perpendicular separation exceeds the threshold.
     */
    static bool arePlanesApartEnough(const geometric::Plane *p_plane1_in,
                                     const geometric::Plane *p_plane2_in,
                                     const double           &threshold_in);

    /*!
     * @brief        Checks to see if two planes are perpendicular to each
     *               other or not.
     *
     * @param[in]    p_plane1_in
     *               First plane; shall be non-null.
     * @param[in]    p_plane2_in
     *               Second plane; shall be non-null.
     *
     * @return       True when the inter-plane angle is within the
     *               configured threshold of 90 degrees.
     */
    static bool arePlanesPerpendicular(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in);

    /*!
     * @brief        Checks to see if two planes are parallel to each other
     *               or not.
     *
     * @param[in]    p_plane1_in
     *               First plane; shall be non-null.
     * @param[in]    p_plane2_in
     *               Second plane; shall be non-null.
     *
     * @return       True when the plane normals align within the
     *               configured threshold.
     */
    static bool
        arePlanesParallel(const vs_graphs::core::geometric::Plane *p_plane1_in,
                          const vs_graphs::core::geometric::Plane *p_plane2_in);

    /*!
     * @brief        Checks to see if two planes are facing each other or
     *               not.
     *
     * @param[in]    p_plane1_in
     *               First plane (small plane, e.g., door); may be null.
     * @param[in]    p_plane2_in
     *               Second plane (big plane, e.g., wall); may be null.
     *
     * @return       True when the two valid planes face each other
     *               across a gap.
     */
    static bool arePlanesFacingEachOther(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in);

    /*!
     * @brief        Returns the planes that are facing each other from
     *               the given list.
     *
     * @param[in]    planes_in
     *               List of planes to be checked.
     *
     * @return       Facing plane pairs whose separation passes the
     *               configured minimum. Pointers are borrowed from the
     *               input list.
     */
    static std::vector<std::pair<geometric::Plane *, geometric::Plane *>>
        getFacingPlanes(const std::vector<geometric::Plane *> &planes_in);

    /*!
     * @brief        Corrects the given plane equations to apply
     *               calculations.
     *
     * @param[in]    plane_in
     *               Input plane.
     *
     * @return       Plane with negated coefficients when the offset is
     *               positive; unchanged otherwise.
     */
    static Eigen::Vector4d
        correctPlaneDirection(const Eigen::Vector4d &plane_in);

    /*!
     * @brief        Converts the plane equation from local to global.
     *
     * @param[in]    keyframePose_in
     *               Pose of the current keyframe.
     * @param[in]    plane_in
     *               Plane equation in the local map.
     *
     * @return       Transformed plane coefficients.
     */
    static g2o::Plane3D applyPoseToPlane(const Eigen::Matrix4d &keyframePose_in,
                                         const g2o::Plane3D    &plane_in);

    /*!
     * @brief        Gets the centeroid of a set or cluster of points.
     *
     * @param[in]    points_in
     *               Given cluster of points.
     *
     * @return       Mean of the points; zero when the input is empty.
     */
    static Eigen::Vector3d computeCentroidFromPoints(
        const std::vector<Eigen::Vector3d> &points_in);

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
     *
     * @return       Owning pointer to the new downsampled cloud.
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr pointcloudDownsample(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        const float                                  leafSize_in,
        const unsigned int                           minPointsPerVoxel_in);

    /*!
     * @brief        Filters the pointclouds based on the given min/max
     *               distance acceptable. PointT shall be a PCL point type.
     *
     *               Keeps the points whose sensor-forward distance lies
     *               within the configured range.
     *
     * @param[in]    p_cloud_in
     *               Pointcloud to be filtered.
     *
     * @return       Owning pointer to the new filtered cloud.
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr pointcloudDistanceFilter(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in);

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
     *
     * @return       Owning pointer to the new filtered cloud.
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr pointcloudOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        const int                                    meanThreshold_in,
        const float                                  stdDevThreshold_in);

    /*!
     * @brief        Computes the width and height of a plane given its
     *               point cloud.
     *
     * @param[in]    p_cloud_in
     *               Borrowed point cloud of the plane.
     *
     * @return       Plane width and height in the cloud's length units.
     */
    static std::pair<double, double> computePlaneWidthHeight(
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud_in);

    /*!
     * @brief        Performs PCL ransac to get the plane equations from
     *               a given point cloud. SegmentationType names the PCL
     *               segmentation applied to PointT.
     *
     * @param[in,out] cloud_inout
     *               Input cloud; planes are extracted from it.
     *
     * @return       Extracted plane clouds paired with their plane
     *               equations.
     */
    template <typename PointT, template <typename> class SegmentationType>
    static std::vector<
        std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
        ransacPlaneFitting(typename pcl::PointCloud<PointT>::Ptr &cloud_inout);

    /*!
     * @brief        Checks to see if the given point is on the plane or
     *               not.
     *
     * @param[in]    planeEquation_in
     *               Plane equation.
     * @param[in]    p_mapPoint_in
     *               Borrowed point to be checked; shall be non-null.
     *
     * @return       True when the point lies within the configured
     *               distance of the plane.
     */
    static bool pointOnPlane(Eigen::Vector4d planeEquation_in,
                             MapPoint       *p_mapPoint_in);

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
     * @return       Plane id of the mapped plane.
     */
    static int associatePlanes(
        const vector<geometric::Plane *>            &mappedPlanes_in,
        g2o::Plane3D                                 observedPlane_in,
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_observedCloud_in,
        const Eigen::Matrix4d                       &keyframePose_in,
        const geometric::Plane::PlaneVariant         observedPlaneType_in,
        const float                                  threshold_in,
        const float maximumFiniteCloudDistance_m_in = -1.0F,
        const std::optional<Eigen::Vector3d> &observationOrigin_World_m_in =
            std::nullopt);

    /*!
     * @brief        Clusters the point cloud into separate clouds based
     *               on the plane detection.
     *
     * @param[in]    p_cloud_in
     *               Point cloud to be clustered.
     * @param[out]   clusterIndices_out
     *               Vector of point indices for each cluster.
     */
    static void clusterPlaneClouds(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &p_cloud_in,
        std::vector<pcl::PointIndices>                &clusterIndices_out);

    /*!
     * @brief        Re-associates semantically classified planes if they get
     *               closer after optimization
     *
     * @param[in]    p_atlas_inout
     *               A pointer to the Atlas
     */
    static void reAssociateSemanticPlanes(Atlas *p_atlas_inout);

    /*!
     * @brief        Re-associates semantically classified planes if they get
     *               closer after optimization
     *
     * @param[in]    p_atlas_inout
     *               a pointer to the Atlas
     */
    static void reAssociateRooms(Atlas *p_atlas_inout);

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
    static void fuseDuplicateRoomsAfterMerge(
        Map                                 *p_map_inout,
        const std::vector<semantic::Room *> &importedRooms_in);

    /*!
     * @brief        Fuses duplicate passage entities after optimization or map
     *               merge and rewires all room/keyframe associations.
     *
     * @param[in,out] p_atlas_inout
     *                 Atlas whose active semantic graph is reconciled.
     */
    static void reAssociatePassages(Atlas *p_atlas_inout);

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
    static void propagateSemanticPoseCorrections(
        Map                   *p_map_inout,
        const KeyFramePoseMap &keyFramePosesBefore_WorldToCamera_in,
        const KeyFramePoseMap &keyFramePosesAfter_WorldToCamera_in,
        const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in);

    /*!
     * @brief        Consolidates redundant single-wall provisional structural
     *               elements into a room supported by a free-space cluster.
     *
     * @param[in]    p_selectedRoom_inout
     *               The cluster-backed room which has absorbed the walls
     *
     * @param[in]    p_atlas_inout
     *               a pointer to the Atlas
     */
    static void consolidateProvisionalRooms(
        vs_graphs::core::semantic::Room *p_selectedRoom_inout,
        Atlas                           *p_atlas_inout);

    /*!
     * @brief        Gets the PlaneVariant type from the class id.
     *
     * @param[in]    classId_in
     *               Class id.
     *
     * @return       Ground, wall, door or window type for ids 0 to 3;
     *               UNDEFINED otherwise.
     */
    static vs_graphs::core::geometric::Plane::PlaneVariant
        getPlaneTypeFromClassId(int classId_in);

    /*!
     * @brief        Gets the class id from the PlaneVariant type.
     *
     * @param[in]    planeType_in
     *               PlaneVariant type.
     *
     * @return       Class id 0 to 3 for known types; -1 otherwise.
     */
    static int getClassIdFromPlaneType(
        vs_graphs::core::geometric::Plane::PlaneVariant planeType_in);

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
     * @return       Rigid transform from map A to map B, or identity
     *               when the input data is invalid.
     */
    static Eigen::Isometry3d computeMapTransform_Horn(
        const std::vector<Eigen::Vector3d> &normalsA_in,
        const std::vector<Eigen::Vector3d> &centroidsA_in,
        const std::vector<Eigen::Vector3d> &normalsB_in,
        const std::vector<Eigen::Vector3d> &centroidsB_in);

    /*!
     * @brief        Pairs the walls of two matched rooms by greedy
     *               best-first normal alignment.
     *
     * @param[in]    p_roomA_in
     *               Room in the surviving map; may be null.
     * @param[in]    p_roomB_in
     *               Same physical room in the map to be absorbed; may
     *               be null.
     * @param[out]   normalsA_out
     *               Matched wall normals of room A.
     * @param[out]   centroidsA_out
     *               Matched wall centroids of room A.
     * @param[out]   normalsB_out
     *               Matched wall normals of room B.
     * @param[out]   centroidsB_out
     *               Matched wall centroids of room B.
     *
     * @return       Number of accepted wall pairs; zero when either
     *               room is null.
     */
    static std::size_t
        matchWallsBetweenRooms(const semantic::Room         *p_roomA_in,
                               const semantic::Room         *p_roomB_in,
                               std::vector<Eigen::Vector3d> &normalsA_out,
                               std::vector<Eigen::Vector3d> &centroidsA_out,
                               std::vector<Eigen::Vector3d> &normalsB_out,
                               std::vector<Eigen::Vector3d> &centroidsB_out);

    /*!
     * @brief        Gathers wall correspondences across two maps using
     *               their shared room identity tags.
     *
     * @param[in]    p_mapA_in
     *               Surviving map; may be null.
     * @param[in]    p_mapB_in
     *               Map to be absorbed; may be null.
     * @param[out]   normalsA_out
     *               Matched wall normals of map A.
     * @param[out]   centroidsA_out
     *               Matched wall centroids of map A.
     * @param[out]   normalsB_out
     *               Matched wall normals of map B.
     * @param[out]   centroidsB_out
     *               Matched wall centroids of map B.
     *
     * @return       True when at least three valid correspondences were
     *               collected.
     */
    static bool
        collectCorrespondingWalls(Map                          *p_mapA_in,
                                  Map                          *p_mapB_in,
                                  std::vector<Eigen::Vector3d> &normalsA_out,
                                  std::vector<Eigen::Vector3d> &centroidsA_out,
                                  std::vector<Eigen::Vector3d> &normalsB_out,
                                  std::vector<Eigen::Vector3d> &centroidsB_out);

    /*!
     * @brief        Calculates the soft-min approximation of the given
     *               values. Soft-min estimates segment quality from
     *               per-pixel uncertainties.
     *
     * @param[in]    values_in
     *               Input values; shall hold at least one value.
     *
     * @return       Soft-min value.
     */
    static double calcSoftMin(vector<double> &values_in);
};
} // namespace core
} // namespace vs_graphs

#endif // UTILS_H
