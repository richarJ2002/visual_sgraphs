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

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_UTILSCLASS_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_UTILSCLASS_H

#include "mapping/geometry_helpers/objects/PlaneEvidenceStruct.h"
#include "mapping/geometry_helpers/objects/PlaneMatchSettingsStruct.h"
#include "mapping/geometry_helpers/objects/PlaneMatchVerdictStruct.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"

/*!
 * @file            UtilsClass.h
 *
 * @brief           Declares geometric and point-cloud helpers for the
 *                  estimator.
 */

#include "map_state/atlas/atlas.h"
#include "dependency_lineage/pcl_custom/pcl_custom.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
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
 * @brief           Static geometric and point-cloud helpers used across the
 *                  estimator.
 */
class Utils
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Maps each key frame to its world-to-camera pose as a
     *                  Sim(3); the keys are borrowed key frame pointers.
     */
    using KeyFramePoseMap = std::map<
        keyframe::KeyFrame *,
        g2o::Sim3,
        std::less<keyframe::KeyFrame *>,
        Eigen::aligned_allocator<std::pair<keyframe::KeyFrame *const, g2o::Sim3>>>;

    /* Variables */
    /*!
     * @brief           Conversion factor from degrees to radians.
     */
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    /*!
     * @brief           Calculate the Euclidean distance between two points.
     *
     * @param[in]       point1_in
     *                  First point.
     *
     * @param[in]       point2_in
     *                  Second point.
     *
     * @param[out]      euclideanDistance_out
     *                  Euclidean distance in the units of the input points.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        calculateEuclideanDistance(const Eigen::Vector3f &point1_in,
                                   const Eigen::Vector3f &point2_in,
                                   double &euclideanDistance_out);

    /*!
     * @brief           Calculate the distance between a point and a plane.
     *
     * @param[in]       plane_in
     *                  Plane equation.
     *
     * @param[in]       point_in
     *                  Given point.
     *
     * @param[out]      distance_out
     *                  Absolute distance in the input length units.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        calculateDistancePointToPlane(const Eigen::Vector4d &plane_in,
                                      const Eigen::Vector3d &point_in,
                                      double                &distance_out);

    /*!
     * @brief           Calculates the intersection point of a line and a
     *                  plane. The line shall not be parallel to the plane.
     *
     * @param[in]       plane_in
     *                  Plane equation.
     *
     * @param[in]       lineStart_in
     *                  Start point of the line.
     *
     * @param[in]       lineEnd_in
     *                  End point of the line.
     *
     * @param[out]      intersection_out
     *                  Point where the line meets the plane.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        lineIntersectsPlane(const Eigen::Vector4d &plane_in,
                            const Eigen::Vector3d &lineStart_in,
                            const Eigen::Vector3d &lineEnd_in,
                            Eigen::Vector3d       &intersection_out);

    /*!
     * @brief           Checks to see if two planes are apart enough from
     *                  each other, given a threshold.
     *
     * @param[in]       p_plane1_in
     *                  First plane; may be null.
     *
     * @param[in]       p_plane2_in
     *                  Second plane; may be null.
     *
     * @param[in]       threshold_in
     *                  Threshold value for perpendicularity, in metres.
     *
     * @param[out]      arePlanesApartEnough_out
     *                  True when both planes are valid and their perpendicular
     *                  separation exceeds the threshold.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        arePlanesApartEnough(const geometric::Plane *p_plane1_in,
                             const geometric::Plane *p_plane2_in,
                             const double           &threshold_in,
                             bool                   &arePlanesApartEnough_out);

    /*!
     * @brief           Checks to see if two planes are perpendicular to each
     *                  other or not.
     *
     * @param[in]       p_plane1_in
     *                  First plane; shall be non-null.
     *
     * @param[in]       p_plane2_in
     *                  Second plane; shall be non-null.
     *
     * @param[out]      arePlanesPerpendicular_out
     *                  True when the inter-plane angle is within the configured
     *                  threshold of 90 degrees.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus arePlanesPerpendicular(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in,
        bool                                    &arePlanesPerpendicular_out);

    /*!
     * @brief           Checks to see if two planes are parallel to each other
     *                  or not.
     *
     * @param[in]       p_plane1_in
     *                  First plane; shall be non-null.
     *
     * @param[in]       p_plane2_in
     *                  Second plane; shall be non-null.
     *
     * @param[out]      arePlanesParallel_out
     *                  True when the plane normals align within the configured
     *                  threshold.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        arePlanesParallel(const vs_graphs::core::geometric::Plane *p_plane1_in,
                          const vs_graphs::core::geometric::Plane *p_plane2_in,
                          bool &arePlanesParallel_out);

    /*!
     * @brief           Checks to see if two planes are facing each other or
     *                  not.
     *
     * @param[in]       p_plane1_in
     *                  First plane (small plane, e.g., door); may be null.
     *
     * @param[in]       p_plane2_in
     *                  Second plane (big plane, e.g., wall); may be null.
     *
     * @param[out]      arePlanesFacingEachOther_out
     *                  True when the two valid planes face each other across a
     *                  gap.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus arePlanesFacingEachOther(
        const vs_graphs::core::geometric::Plane *p_plane1_in,
        const vs_graphs::core::geometric::Plane *p_plane2_in,
        bool                                    &arePlanesFacingEachOther_out);

    /*!
     * @brief           Returns the planes that are facing each other from
     *                  the given list.
     *
     * @param[in]       planes_in
     *                  List of planes to be checked.
     *
     * @param[out]      facingPlanes_out
     *                  Facing plane pairs whose separation passes the
     *                  configured minimum. Pointers are borrowed from the input
     *                  list.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus getFacingPlanes(
        const std::vector<geometric::Plane *> &planes_in,
        std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                              vs_graphs::core::geometric::Plane *>>
            &facingPlanes_out);

    /*!
     * @brief           Corrects the given plane equations to apply
     *                  calculations.
     *
     * @param[in]       plane_in
     *                  Input plane.
     *
     * @param[out]      planeDirection_out
     *                  Plane with negated coefficients when the offset is
     *                  positive; unchanged otherwise.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        correctPlaneDirection(const Eigen::Vector4d &plane_in,
                              Eigen::Vector4d       &planeDirection_out);

    /*!
     * @brief           Converts the plane equation from local to global.
     *
     * @param[in]       keyframePose_in
     *                  Pose of the current keyframe.
     *
     * @param[in]       plane_in
     *                  Plane equation in the local map.
     *
     * @param[out]      transformedPlane_out
     *                  Transformed plane coefficients.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        applyPoseToPlane(const Eigen::Matrix4d &keyframePose_in,
                         const g2o::Plane3D    &plane_in,
                         g2o::Plane3D          &transformedPlane_out);

    /*!
     * @brief           Gets the centeroid of a set or cluster of points.
     *
     * @param[in]       points_in
     *                  Given cluster of points.
     *
     * @param[out]      centroid_out
     *                  Mean of the points; zero when the input is empty.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        computeCentroidFromPoints(const std::vector<Eigen::Vector3d> &points_in,
                                  Eigen::Vector3d &centroid_out);

    /*!
     * @brief           Downsamples the pointclouds based on the given leaf
     *                  size. PointT shall be a PCL point type.
     *
     * @param[in]       p_cloud_in
     *                  Pointcloud to be downsampled.
     *
     * @param[in]       leafSize_in
     *                  Leaf size for downsampling, in the cloud's length
     *                  units.
     *
     * @param[in]       minPointsPerVoxel_in
     *                  Minimum points required to keep a voxel.
     *
     * @param[out]      p_downsampledCloud_out
     *                  Owning pointer to the new downsampled cloud.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    template <typename PointT>
    [[nodiscard]] static UtilsStatus pointcloudDownsample(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        const float                                  leafSize_in,
        const unsigned int                           minPointsPerVoxel_in,
        typename pcl::PointCloud<PointT>::Ptr       &p_downsampledCloud_out);

    /*!
     * @brief           Filters the pointclouds based on the given min/max
     *                  distance acceptable. PointT shall be a PCL point type.
     *
     *                  Keeps the points whose sensor-forward distance lies
     *                  within the configured range.
     *
     * @param[in]       p_cloud_in
     *                  Pointcloud to be filtered.
     *
     * @param[out]      p_filteredCloud_out
     *                  Owning pointer to the new filtered cloud.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    template <typename PointT>
    [[nodiscard]] static UtilsStatus pointcloudDistanceFilter(
        const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
        typename pcl::PointCloud<PointT>::Ptr       &p_filteredCloud_out);

    /*!
     * @brief           Computes the width and height of a plane given its
     *                  point cloud.
     *
     * @param[in]       p_cloud_in
     *                  Borrowed point cloud of the plane.
     *
     * @param[out]      planeWidthHeight_out
     *                  Plane width and height in the cloud's length units.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus computePlaneWidthHeight(
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud_in,
        std::pair<double, double>                   &planeWidthHeight_out);

    /*!
     * @brief           Performs PCL ransac to get the plane equations from
     *                  a given point cloud. SegmentationType names the PCL
     *                  segmentation applied to PointT.
     *
     * @param[in,out]   cloud_inout
     *                  Input cloud; planes are extracted from it.
     *
     * @param[out]      planes_out
     *                  Extracted plane clouds paired with their plane
     *                  equations.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    template <typename PointT, template <typename> class SegmentationType>
    [[nodiscard]] static UtilsStatus ransacPlaneFitting(
        typename pcl::PointCloud<PointT>::Ptr   &cloud_inout,
        std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr,
                              Eigen::Vector4d>> &planes_out);

    /*!
     * @brief           Finds the mapped plane that best matches an observed
     *                  plane under one settings bundle. Compare planes of two
     *                  maps by transforming one into the other's world frame
     *                  first.
     *
     *                  SAME_FACE rejects a candidate whose type fails the
     *                  type rule, whose normal or perpendicular offset is too
     *                  far off, whose support cloud neither lies near nor
     *                  extends the observed one, or, for a wall, that was
     *                  observed from the other side. The best remaining
     *                  candidate has the lowest weighted sum of angle,
     *                  offset, centroid and cloud distances.
     *
     *                  OPPOSITE_FACE accepts a candidate that is parallel, no
     *                  more than the maximum thickness away, observed from
     *                  the opposite side (stamped observation origins) and
     *                  overlapping in footprint by the minimum ratio. The
     *                  accepted candidate with the most observations wins;
     *                  the earliest wins a tie.
     *
     * @param[in]       mappedPlanes_in
     *                  Candidate planes, non-owning; null and bad planes are
     *                  skipped.
     *
     * @param[in]       observedPlane_in
     *                  Observed plane evidence, in the candidates' world
     *                  frame.
     *
     * @param[in]       matchSettings_in
     *                  Match mode, type rule and thresholds.
     *
     * @param[out]      matchedPlaneId_out
     *                  Id of the best matching mapped plane, or -1 when none
     *                  matches.
     *
     * @param[out]      p_candidateVerdicts_out
     *                  Optional, may be null. OPPOSITE_FACE fills one verdict
     *                  per mapped plane, in input order; SAME_FACE clears it.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus findMatchingPlane(
        const std::vector<geometric::Plane *> &mappedPlanes_in,
        const PlaneEvidence                   &observedPlane_in,
        const PlaneMatchSettings              &matchSettings_in,
        int                                   &matchedPlaneId_out,
        std::vector<PlaneMatchVerdict>        *p_candidateVerdicts_out);

    /*!
     * @brief           Clusters the point cloud into separate clouds based
     *                  on the plane detection.
     *
     * @param[in]       p_cloud_in
     *                  Point cloud to be clustered.
     *
     * @param[in,out]   clusterIndices_inout
     *                  Vector of point indices for each cluster.
     */
    [[nodiscard]] static UtilsStatus clusterPlaneClouds(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &p_cloud_in,
        std::vector<pcl::PointIndices>                &clusterIndices_inout);

    /*!
     * @brief           Re-associates semantically classified planes if they get
     *                  closer after optimization
     *
     * @param[in]       p_atlas_in
     *                  A pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus
        reAssociateSemanticPlanes(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Re-associates semantically classified planes if they get
     *                  closer after optimization
     *
     * @param[in]       p_atlas_in
     *                  a pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus reAssociateRooms(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Fuses each duplicate room that a map merge brought in
     *                  into the room it duplicates. The pairs come from the
     *                  geometric room pairing (walls coplanar, same facing and
     *                  overlapping; the anchor rooms by their tag); this
     *                  function pairs nothing itself. The imported room's walls
     *                  and passages move to the retained room, a wall behind a
     *                  passable passage stays with the far-side room, passage
     *                  and floor references are re-pointed, and the imported
     *                  room is retired. A pair whose rooms lie on opposite
     *                  sides of a passable passage opening is left alone.
     *
     * @param[in,out]   p_map_inout
     *                  Surviving map whose room graph is reconciled.
     *
     * @param[in]       duplicateRoomPairs_in
     *                  Pairs (imported room, retained room). Borrowed; both
     *                  rooms belong to p_map_inout.
     */
    [[nodiscard]] static UtilsStatus fuseDuplicateRoomsAfterMerge(
        map::Map *p_map_inout,
        const std::vector<std::pair<semantic::Room *, semantic::Room *>>
            &duplicateRoomPairs_in);

    /*!
     * @brief           Fuses duplicate passage entities after optimization or
     *                  map merge and rewires all room/keyframe associations.
     *
     * @param[in]       p_atlas_in
     *                  Atlas whose active semantic graph is reconciled.
     */
    [[nodiscard]] static UtilsStatus reAssociatePassages(atlas::Atlas *p_atlas_in);

    /*!
     * @brief           Propagates keyframe pose corrections to the
     *                   semantic graph.
     *
     *                  A local or global bundle adjustment changes
     *                  keyframe poses and mapped points, but semantic
     *                  entities are stored independently in the world
     *                  frame. This method applies the corresponding
     *                  old-world to corrected-world deformation exactly
     *                  once to planes, markers, passages, rooms, floors,
     *                  and free-space skeleton geometry.
     *
     * @param[in,out]   p_map_inout
     *                  Map whose semantic geometry is updated.
     *
     * @param[in]       keyFramePosesBefore_worldToCamera_in
     *                  World-to-camera poses immediately before
     *                  optimization.
     *
     * @param[in]       keyFramePosesAfter_worldToCamera_in
     *                  World-to-camera poses immediately after
     *                  optimization.
     *
     * @param[in]       fallbackTransform_oldWorldToNewWorld_in
     *                  Transform used only when no valid keyframe
     *                  deformation node can anchor an entity. Pass
     *                  identity for an in-place BA.
     */
    [[nodiscard]] static UtilsStatus propagateSemanticPoseCorrections(
        map::Map                   *p_map_inout,
        const KeyFramePoseMap &keyFramePosesBefore_worldToCamera_in,
        const KeyFramePoseMap &keyFramePosesAfter_worldToCamera_in,
        const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in);

    /*!
     * @brief           Consolidates redundant single-wall provisional
     *                  structural elements into a room supported by a
     *                  free-space cluster.
     *
     * @param[in,out]   p_selectedRoom_inout
     *                  The cluster-backed room which has absorbed the walls
     *
     * @param[in]       p_atlas_in
     *                  a pointer to the Atlas
     */
    [[nodiscard]] static UtilsStatus consolidateProvisionalRooms(
        vs_graphs::core::semantic::Room *p_selectedRoom_inout,
        atlas::Atlas                           *p_atlas_in);

    /*!
     * @brief           Gets the PlaneVariant type from the class id.
     *
     * @param[in]       classId_in
     *                  Class id.
     *
     * @param[out]      planeTypeFromClassId_out
     *                  Ground, wall, door or window type for ids 0 to 3;
     *                  UNDEFINED otherwise.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus
        getPlaneTypeFromClassId(int classId_in,
                                vs_graphs::core::geometric::Plane::PlaneVariant
                                    &planeTypeFromClassId_out);

    /*!
     * @brief           Gets the class id from the PlaneVariant type.
     *
     * @param[in]       planeType_in
     *                  PlaneVariant type.
     *
     * @param[out]      classIdFromPlaneType_out
     *                  Class id 0 to 3 for known types; -1 otherwise.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus getClassIdFromPlaneType(
        vs_graphs::core::geometric::Plane::PlaneVariant planeType_in,
        int &classIdFromPlaneType_out);

    /*!
     * @brief           Calculates the soft-min approximation of the given
     *                  values. Soft-min estimates segment quality from
     *                  per-pixel uncertainties.
     *
     * @param[in]       values_in
     *                  Input values; shall hold at least one value.
     *
     * @param[out]      softMin_out
     *                  Soft-min value.
     *
     * @return          UTILS_STATUS_SUCCESS.
     */
    [[nodiscard]] static UtilsStatus calcSoftMin(std::vector<double> &values_in,
                                                 double &softMin_out);
};
} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_UTILSCLASS_H */
