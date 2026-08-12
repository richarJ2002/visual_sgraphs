/**
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

namespace ORB_SLAM3
{
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
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    /**
     * @brief Calculate the Euclidean distance between two points
     * @param point1 first point
     * @param point2 second point
     */
    static double calculateEuclideanDistance(const Eigen::Vector3f &p1,
                                             const Eigen::Vector3f &p2);

    /**
     * @brief Calculate the distance between a point and a plane
     * @param plane the plane equation
     * @param point the given point
     */
    static double calculateDistancePointToPlane(const Eigen::Vector4d &plane,
                                                const Eigen::Vector3d &point);

    /**
     * @brief Calculates the intersection point of a line and a plane
     * @param plane the plane equation
     * @param lineStart the start point of the line
     * @param lineEnd the end point of the line
     */
    static Eigen::Vector3d lineIntersectsPlane(const Eigen::Vector4d &plane,
                                               const Eigen::Vector3d &lineStart,
                                               const Eigen::Vector3d &lineEnd);

    /**
     * @brief Checks to see if two planes are apart enough from each other,
     * given a threshold
     * @param plane1 first plane
     * @param plane2 second plane
     * @param threshold the threshold value for perpendicularity
     */
    static bool arePlanesApartEnough(const Plane  *plane1,
                                     const Plane  *plane2,
                                     const double &threshold);

    /**
     * @brief Checks to see if two planes are perpendicular to each other or not
     * @param plane1 first plane
     * @param plane2 second plane
     */
    static bool arePlanesPerpendicular(const ORB_SLAM3::Plane *plane1,
                                       const ORB_SLAM3::Plane *plane2);

    /**
     * @brief Checks to see if two planes are parallel to each other or not
     * @param plane1 first plane
     * @param plane2 second plane
     */
    static bool arePlanesParallel(const ORB_SLAM3::Plane *plane1,
                                  const ORB_SLAM3::Plane *plane2);

    /**
     * @brief Checks to see if two planes are facing each other or not
     * @param plane1 first plane (small plane, e.g., door)
     * @param plane2 second plane (big plane, e.g., wall)
     */
    static bool arePlanesFacingEachOther(const ORB_SLAM3::Plane *plane1,
                                         const ORB_SLAM3::Plane *plane2);

    /**
     * @brief Returns the planes that are facing each other from the given list
     * @param planes list of planes to be checked
     */
    static std::vector<std::pair<Plane *, Plane *>>
        getFacingPlanes(const std::vector<Plane *> &planes);

    /**
     * @brief Corrects the given plane equations to apply calculations
     * @param plane the input plane
     */
    static Eigen::Vector4d correctPlaneDirection(const Eigen::Vector4d &plane);

    /**
     * @brief Converts the plane equation from local to global
     * @param kfPose the pose of the current keyframe
     * @param plane the plane equation in the local map
     */
    static g2o::Plane3D applyPoseToPlane(const Eigen::Matrix4d &kfPose,
                                         const g2o::Plane3D    &plane);

    /**
     * @brief Gets the centeroid of a set or cluster of points
     * @param points the given cluster of points
     */
    static Eigen::Vector3d
        computeCentroidFromPoints(const std::vector<Eigen::Vector3d> &points);

    /**
     * @brief Downsamples the pointclouds based on the given leaf size
     *
     * @param cloud the pointcloud to be downsampled
     * @param leafSize the leaf size for downsampling
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr
        pointcloudDownsample(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                             const float        leafSize,
                             const unsigned int minPointsPerVoxel);

    /**
     * @brief Filters the pointclouds based on the given min/max distance
     * acceptable
     *
     * @param cloud the pointcloud to be filtered
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr pointcloudDistanceFilter(
        const typename pcl::PointCloud<PointT>::Ptr &cloud);

    /**
     * @brief Removes the points that are farther away from their neighbors
     *
     * @param cloud the pointcloud to be filtered
     * @param meanThresh the mean threshold for neighbor points
     * @param stdDevThresh the standard deviation threshold for neighbor points
     */
    template <typename PointT>
    static typename pcl::PointCloud<PointT>::Ptr pointcloudOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        const int                                    meanThresh,
        const float                                  stdDevThresh);

    /**
     * @brief Computes the width and height of a plane given its point cloud
     * @param cloud the point cloud of the plane
     */
    static std::pair<double, double>
        computePlaneWidthHeight(
            pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);

    /**
     * @brief Performs PCL ransac to get the plane equations from the a given
     * point cloud
     *
     * @param cloud the input point cloud
     * @param minSegmentationPoints the minimum number of points
     */
    template <typename PointT, template <typename> class SegmentationType>
    static std::vector<
        std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
        ransacPlaneFitting(typename pcl::PointCloud<PointT>::Ptr &cloud);

    /**
     * @brief Checks to see if the given point is on the plane or not
     * @param planeEquation the plane equation
     * @param mapPoint the point to be checked
     */
    static bool pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint);

    /**
     * @brief associates given planes with the mapped planes
     * @param mappedPlanes the mapped planes
     * @param givenPlane the given plane (in the same frame as the kfPose,
     * global if kfPose is identity)
     * @param kfPose the pose of the current keyframe
     * @param threshold the threshold value for association
     * @param maximumFiniteCloudDistance_m_in optional finite-cloud gap override
     * @param observationOrigin_World_m_in optional observing camera origin used
     *        to keep opposite wall faces separate
     * @return the plane id of the mapped plane
     */
    static int
        associatePlanes(const vector<Plane *>                  &mappedPlanes,
                        g2o::Plane3D                            givenPlane,
                        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr givenCloud,
                        const Eigen::Matrix4d                   &kfPose,
                        const Plane::planeVariant                obsPlaneType,
                        const float                              threshold,
                        const float maximumFiniteCloudDistance_m_in = -1.0F,
                        const std::optional<Eigen::Vector3d>
                            &observationOrigin_World_m_in = std::nullopt);

    /**
     * @brief Clusters the point cloud into separate clouds based on the plane
     * detection
     * @param cloud the point cloud to be clustered
     * @param clusterIndices the vector of point indices for each cluster
     */
    static void
        clusterPlaneClouds(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                           std::vector<pcl::PointIndices> &clusterIndices);

    /*!
     * @brief       Re-associates semantically classified planes if they get
     *              closer after optimization
     *
     * @param[in]   mpAtlas
     *              A pointer to the Atlas
     */
    static void reAssociateSemanticPlanes(Atlas *mpAtlas);

    /*!
     * @brief       Re-associates semantically classified planes if they get
     *              closer after optimization
     *
     * @param[in]   mpAtlas
     *              a pointer to the Atlas
     */
    static void reAssociateRooms(Atlas *mpAtlas);

    /*!
     * @brief Fuses duplicate confirmed rooms introduced by a map merge.
     *
     *        Only rooms imported from the obsolete submap are considered for
     *        retirement. A candidate must agree with a pre-existing room in
     *        type, centroid proximity, shared-wall side, and unobstructed
     *        free-space geometry. This prevents adjacent rooms on opposite
     *        sides of a wall from being collapsed.
     *
     * @param[in,out] p_map_inout
     *                Surviving map whose room graph is reconciled.
     * @param[in] importedRooms_in
     *            Rooms transferred from the obsolete submap.
     */
    static void fuseDuplicateRoomsAfterMerge(
        Map                       *p_map_inout,
        const std::vector<Room *> &importedRooms_in);

    /*!
     * @brief       Fuses duplicate passage entities after optimization or map
     *              merge and rewires all room/keyframe associations.
     *
     * @param[in,out] p_atlas_inout
     *                Atlas whose active semantic graph is reconciled.
     */
    static void reAssociatePassages(Atlas *p_atlas_inout);

    /**
     * @brief Propagates keyframe pose corrections to the semantic graph.
     *
     * A local or global bundle adjustment changes keyframe poses and mapped
     * points, but semantic entities are stored independently in the world
     * frame. This method applies the corresponding old-world to corrected-
     * world deformation exactly once to planes, markers, passages, rooms,
     * floors, and free-space skeleton geometry.
     *
     * @param[in,out] p_map_inout
     *                Map whose semantic geometry is updated.
     * @param[in] keyFramePosesBefore_WorldToCamera_in
     *            World-to-camera poses immediately before optimization.
     * @param[in] keyFramePosesAfter_WorldToCamera_in
     *            World-to-camera poses immediately after optimization.
     * @param[in] fallbackTransform_oldWorldToNewWorld_in
     *            Transform used only when no valid keyframe deformation node
     *            can anchor an entity. Pass identity for an in-place BA.
     */
    static void propagateSemanticPoseCorrections(
        Map                   *p_map_inout,
        const KeyFramePoseMap &keyFramePosesBefore_WorldToCamera_in,
        const KeyFramePoseMap &keyFramePosesAfter_WorldToCamera_in,
        const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in);

    /*!
     * @brief       Consolidates redundant single-wall provisional structural
     *              elements into a room supported by a free-space cluster.
     *
     * @param[in]   selectedRoom
     *              The cluster-backed room which has absorbed the walls
     *
     * @param[in]   mpAtlas
     *              a pointer to the Atlas
     */
    static void consolidateProvisionalRooms(ORB_SLAM3::Room *selectedRoom,
                                            Atlas           *mpAtlas);

    /**
     * @brief Gets the planeVariant type from the class id
     * @param clsId the class id
     * @return the planeVariant type
     */
    static ORB_SLAM3::Plane::planeVariant getPlaneTypeFromClassId(int clsId);

    /**
     * @brief Gets the class id from the planeVariant type
     * @param planeType the planeVariant type
     * @return the class id
     */
    static int
        getClassIdFromPlaneType(ORB_SLAM3::Plane::planeVariant planeType);

    /**
     * @brief Computes the rigid transform mapping map A into map B using
     * Horn's closed-form solution on oriented wall-normal correspondences.
     *
     * The transform maps points from map A's frame into map B's frame such
     * that p_B = T * p_A.
     *
     * @param normalsA_in unit wall normals of map A
     * @param centroidsA_in wall centroids of map A
     * @param normalsB_in unit wall normals of map B
     * @param centroidsB_in wall centroids of map B
     * @return the rigid transform from map A to map B, or identity when the
     *         input data is invalid
     */
    static Eigen::Isometry3d computeMapTransform_Horn(
        const std::vector<Eigen::Vector3d> &normalsA_in,
        const std::vector<Eigen::Vector3d> &centroidsA_in,
        const std::vector<Eigen::Vector3d> &normalsB_in,
        const std::vector<Eigen::Vector3d> &centroidsB_in);

    /**
     * @brief Pairs the walls of two matched rooms by greedy best-first normal
     * alignment.
     *
     * @param p_roomA_in room in the surviving map
     * @param p_roomB_in the same physical room in the map to be absorbed
     * @param normalsA_out matched wall normals of room A
     * @param centroidsA_out matched wall centroids of room A
     * @param normalsB_out matched wall normals of room B
     * @param centroidsB_out matched wall centroids of room B
     * @return the number of accepted wall pairs
     */
    static std::size_t
        matchWallsBetweenRooms(const Room                   *p_roomA_in,
                               const Room                   *p_roomB_in,
                               std::vector<Eigen::Vector3d> &normalsA_out,
                               std::vector<Eigen::Vector3d> &centroidsA_out,
                               std::vector<Eigen::Vector3d> &normalsB_out,
                               std::vector<Eigen::Vector3d> &centroidsB_out);

    /**
     * @brief Gathers wall correspondences across two maps using their shared
     * room identity tags.
     *
     * @param p_mapA_in the surviving map
     * @param p_mapB_in the map to be absorbed
     * @param normalsA_out matched wall normals of map A
     * @param centroidsA_out matched wall centroids of map A
     * @param normalsB_out matched wall normals of map B
     * @param centroidsB_out matched wall centroids of map B
     * @return true when at least three valid correspondences were collected
     */
    static bool
        collectCorrespondingWalls(Map                          *p_mapA_in,
                                  Map                          *p_mapB_in,
                                  std::vector<Eigen::Vector3d> &normalsA_out,
                                  std::vector<Eigen::Vector3d> &centroidsA_out,
                                  std::vector<Eigen::Vector3d> &normalsB_out,
                                  std::vector<Eigen::Vector3d> &centroidsB_out);

    /**
     * @brief Calculates the soft-min approximation of the given values
     * soft-min is an estimate of the quality of the segment based on per-pixel
     * uncertainities
     * @param values the input values
     * @return the soft-min value
     */
    static double calcSoftMin(vector<double> &values);
};
} // namespace ORB_SLAM3

#endif // UTILS_H
