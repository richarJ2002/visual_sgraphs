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

#include "Utils.h"
#include "GeoSemHelpers.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_set>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace ORB_SLAM3
{
namespace
{
/*!
 * @brief       Estimates which side of a mapped plane observed it.
 *
 *              Plane coefficients have an arbitrary sign, so the caller
 *              supplies an already normalized and consistently oriented
 *              equation. The median camera-to-plane distance rejects isolated
 *              poses produced during relocalization. Observations too close to
 *              the surface do not provide reliable side evidence.
 *
 * @param[in]   p_plane_in
 *              Plane whose observing keyframes provide the camera positions.
 * @param[in]   planeEquation_World_in
 *              Normalized plane equation expressed in the active map frame.
 *
 * @return      Median signed camera distance in metres, or no value when the
 *              available observations do not establish a side.
 */
struct ObservationSideEvidence
{
    std::optional<double> medianSignedDistance_m;
    bool ambiguous{false};
};

ObservationSideEvidence getMedianObservationSide_World_m(
    Plane                 *p_plane_in,
    const Eigen::Vector4d &planeEquation_World_in)
{
    if (p_plane_in == nullptr)
    {
        return {};
    }
    const Plane::ObservationSideSnapshot snapshot =
        p_plane_in->getObservationSideSnapshot(planeEquation_World_in);
    return {snapshot.medianSignedDistance_m,
            snapshot.face ==
                Plane::ObservationSideSnapshot::Face::AMBIGUOUS};
}

/*!
 * @brief Finite ranges of a cloud projected onto two plane-tangent axes.
 */
struct ProjectedPlaneBounds
{
    double minimumU_m = std::numeric_limits<double>::max();
    double maximumU_m = std::numeric_limits<double>::lowest();
    double minimumV_m = std::numeric_limits<double>::max();
    double maximumV_m = std::numeric_limits<double>::lowest();
    bool   valid      = false;
};

/*!
 * @brief Projects a finite plane cloud onto a shared in-plane coordinate
 *        system.
 *
 * @param[in] p_planeCloud_in
 *            Plane support cloud expressed in the active map frame.
 * @param[in] tangentU_World_in
 *            First unit tangent of the common plane.
 * @param[in] tangentV_World_in
 *            Second unit tangent of the common plane.
 *
 * @return Finite projected bounds, or invalid bounds for an empty cloud.
 */
ProjectedPlaneBounds projectPlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector3d                              &tangentU_World_in,
    const Eigen::Vector3d                              &tangentV_World_in)
{
    ProjectedPlaneBounds bounds;

    if (p_planeCloud_in == nullptr)
    {
        return bounds;
    }

    for (const pcl::PointXYZRGBA &point_World_m : p_planeCloud_in->points)
    {
        if (!pcl::isFinite(point_World_m))
        {
            continue;
        }

        const Eigen::Vector3d position_World_m(point_World_m.x,
                                               point_World_m.y,
                                               point_World_m.z);

        const double coordinateU_m = tangentU_World_in.dot(position_World_m);
        const double coordinateV_m = tangentV_World_in.dot(position_World_m);

        bounds.minimumU_m = std::min(bounds.minimumU_m, coordinateU_m);
        bounds.maximumU_m = std::max(bounds.maximumU_m, coordinateU_m);
        bounds.minimumV_m = std::min(bounds.minimumV_m, coordinateV_m);
        bounds.maximumV_m = std::max(bounds.maximumV_m, coordinateV_m);
        bounds.valid      = true;
    }

    return bounds;
}

/*!
 * @brief Tests whether a segment traverses a passable passage aperture.
 *
 *        Mirrors the far-side wall-routing crossing test used by the semantic
 *        manager so that the merge path can apply the same rule when imported
 *        walls are copied into a retained room.
 *
 * @param[in] segmentStart_World_m_in First endpoint in the active map frame.
 * @param[in] segmentEnd_World_m_in Second endpoint in the active map frame.
 * @param[in] p_passage_in Passable passage defining the finite aperture.
 * @param[in] groundNormal_World_in Unit ground normal in the active map frame.
 * @param[in] openingMargin_m_in Aperture expansion used for noisy geometry.
 * @param[in] minimumSideDistance_m_in Required endpoint distance from plane.
 * @return True only when the segment crosses inside the finite opening.
 */
bool crossesPassablePassageOpening(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    ORB_SLAM3::Passage    *p_passage_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in)
{
    if (p_passage_in == nullptr ||
        !p_passage_in->isPassable() ||
        !segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite())
    {
        return false;
    }

    Eigen::Vector4d passageEquation_World =
        p_passage_in->getGlobalEquation().coeffs();
    const double passageNormalNorm = passageEquation_World.head<3>().norm();

    if (!passageEquation_World.allFinite() || passageNormalNorm < 1e-8)
    {
        return false;
    }

    passageEquation_World /= passageNormalNorm;
    const Eigen::Vector3d passageNormal_World = passageEquation_World.head<3>();
    const double          startSide_m =
        passageNormal_World.dot(segmentStart_World_m_in) +
        passageEquation_World(3);
    const double endSide_m = passageNormal_World.dot(segmentEnd_World_m_in) +
                             passageEquation_World(3);

    if (startSide_m * endSide_m >= 0.0 ||
        std::abs(startSide_m) < minimumSideDistance_m_in ||
        std::abs(endSide_m) < minimumSideDistance_m_in)
    {
        return false;
    }

    const double interpolation = startSide_m / (startSide_m - endSide_m);

    if (!std::isfinite(interpolation) || interpolation < 0.0 ||
        interpolation > 1.0)
    {
        return false;
    }

    const Eigen::Vector3d intersection_World_m =
        segmentStart_World_m_in +
        interpolation * (segmentEnd_World_m_in - segmentStart_World_m_in);
    const Eigen::Vector3d passageCentroid_World_m = p_passage_in->getCentroid();

    if (!passageCentroid_World_m.allFinite())
    {
        return false;
    }

    Eigen::Vector3d apertureOffset_World_m =
        intersection_World_m - passageCentroid_World_m;
    apertureOffset_World_m -=
        apertureOffset_World_m.dot(passageNormal_World) * passageNormal_World;

    const double verticalOffset_m =
        std::abs(apertureOffset_World_m.dot(groundNormal_World_in));
    const Eigen::Vector3d horizontalOffset_World_m =
        apertureOffset_World_m -
        apertureOffset_World_m.dot(groundNormal_World_in) *
            groundNormal_World_in;
    const double horizontalOffset_m = horizontalOffset_World_m.norm();

    return horizontalOffset_m <=
               0.5 * p_passage_in->getWidth() + openingMargin_m_in &&
           verticalOffset_m <=
               0.5 * p_passage_in->getHeight() + openingMargin_m_in;
}

/*!
 * @brief Tests whether two finite clouds overlap or extend one another along
 *        the same plane.
 *
 * @param[in] p_firstCloud_in
 *            First finite plane support cloud.
 * @param[in] p_secondCloud_in
 *            Second finite plane support cloud.
 * @param[in] commonNormal_World_in
 *            Unit normal shared by the already equation-compatible planes.
 * @param[in] maximumInPlaneGap_m_in
 *            Maximum permitted extension gap along either tangent.
 * @param[in] minimumOrthogonalOverlap_m_in
 *            Required overlap along the other tangent.
 *
 * @return True when the clouds overlap or form adjacent finite extensions.
 */
bool finiteWallExtentsAreCompatible(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_firstCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_secondCloud_in,
    const Eigen::Vector3d                              &commonNormal_World_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in)
{
    if (!commonNormal_World_in.allFinite() ||
        commonNormal_World_in.norm() < 1e-8)
    {
        return false;
    }

    const Eigen::Vector3d normal_World = commonNormal_World_in.normalized();

    Eigen::Vector3d referenceAxis_World = Eigen::Vector3d::UnitX();

    if (std::abs(normal_World.dot(referenceAxis_World)) > 0.90)
    {
        referenceAxis_World = Eigen::Vector3d::UnitY();
    }

    const Eigen::Vector3d tangentU_World =
        normal_World.cross(referenceAxis_World).normalized();
    const Eigen::Vector3d tangentV_World =
        normal_World.cross(tangentU_World).normalized();

    const ProjectedPlaneBounds firstBounds =
        projectPlaneBounds(p_firstCloud_in, tangentU_World, tangentV_World);
    const ProjectedPlaneBounds secondBounds =
        projectPlaneBounds(p_secondCloud_in, tangentU_World, tangentV_World);

    if (!firstBounds.valid || !secondBounds.valid)
    {
        return false;
    }

    const double overlapU_m =
        std::min(firstBounds.maximumU_m, secondBounds.maximumU_m) -
        std::max(firstBounds.minimumU_m, secondBounds.minimumU_m);
    const double overlapV_m =
        std::min(firstBounds.maximumV_m, secondBounds.maximumV_m) -
        std::max(firstBounds.minimumV_m, secondBounds.minimumV_m);

    if ((overlapU_m >= minimumOrthogonalOverlap_m_in && overlapV_m >= 0.0) ||
        (overlapV_m >= minimumOrthogonalOverlap_m_in && overlapU_m >= 0.0))
    {
        return true;
    }

    const double gapU_m = std::max(0.0, -overlapU_m);
    const double gapV_m = std::max(0.0, -overlapV_m);

    return (gapU_m <= maximumInPlaneGap_m_in &&
            overlapV_m >= minimumOrthogonalOverlap_m_in) ||
           (gapV_m <= maximumInPlaneGap_m_in &&
            overlapU_m >= minimumOrthogonalOverlap_m_in);
}
} // namespace

double Utils::calculateEuclideanDistance(const Eigen::Vector3f &p1,
                                         const Eigen::Vector3f &p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    double dz = p1.z() - p2.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Utils::calculateDistancePointToPlane(const Eigen::Vector4d &plane,
                                            const Eigen::Vector3d &point)
{
    // Find the distance of the point from a given plane
    return fabs(plane.head<3>().dot(point) + plane(3));
}

Eigen::Vector3d Utils::lineIntersectsPlane(const Eigen::Vector4d &plane,
                                           const Eigen::Vector3d &lineStart,
                                           const Eigen::Vector3d &lineEnd)
{
    // Calculate the direction vector of the line
    Eigen::Vector3d lineDirection = lineEnd - lineStart;

    // [TODO] - check if the line is parallel to the plane

    // Calculate the intersection point
    double t = -(plane.head<3>().dot(lineStart) + plane(3)) /
               plane.head<3>().dot(lineDirection);
    return lineStart + t * lineDirection;
}

bool Utils::arePlanesFacingEachOther(const ORB_SLAM3::Plane *plane1,
                                     const ORB_SLAM3::Plane *plane2)
{
    if (plane1 == nullptr || plane2 == nullptr)
    {
        return false;
    }

    Eigen::Vector4d equation1 = plane1->getGlobalEquation().coeffs();
    Eigen::Vector4d equation2 = plane2->getGlobalEquation().coeffs();

    const double normalNorm1 = equation1.head<3>().norm();
    const double normalNorm2 = equation2.head<3>().norm();

    if (!std::isfinite(normalNorm1) || !std::isfinite(normalNorm2) ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        return false;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    const double normalAlignment =
        std::abs(equation1.head<3>().dot(equation2.head<3>()));

    const double minimumParallelAlignment =
        std::abs(SystemParams::GetParams()->room_seg.plane_facing_dot_thresh);

    if (normalAlignment < minimumParallelAlignment)
    {
        return false;
    }

    if (equation1.head<3>().dot(equation2.head<3>()) < 0.0)
    {
        equation2 *= -1.0;
    }

    const double perpendicularSeparation_m =
        std::abs(equation1(3) - equation2(3));

    /*
     * Plane-equation sign is arbitrary. Two distinct parallel boundary planes
     * can always be oriented toward the space between them, so facing is a
     * relationship between their geometry rather than their stored signs.
     */
    return perpendicularSeparation_m > 1e-3;
}

bool Utils::arePlanesApartEnough(const ORB_SLAM3::Plane *plane1,
                                 const ORB_SLAM3::Plane *plane2,
                                 const double           &threshold)
{
    if (plane1 == nullptr || plane2 == nullptr)
    {
        return false;
    }

    Eigen::Vector4d equation1 = plane1->getGlobalEquation().coeffs();
    Eigen::Vector4d equation2 = plane2->getGlobalEquation().coeffs();

    const double normalNorm1 = equation1.head<3>().norm();
    const double normalNorm2 = equation2.head<3>().norm();

    if (!std::isfinite(normalNorm1) || !std::isfinite(normalNorm2) ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        return false;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    const double normalAlignment = equation1.head<3>().dot(equation2.head<3>());

    if (std::abs(normalAlignment) <= 0.99)
    {
        return false;
    }

    if (normalAlignment < 0.0)
    {
        equation2 *= -1.0;
    }

    const double perpendicularSeparation_m =
        std::abs(equation1(3) - equation2(3));

    return perpendicularSeparation_m > threshold;
}

bool Utils::arePlanesPerpendicular(const ORB_SLAM3::Plane *plane1,
                                   const ORB_SLAM3::Plane *plane2)
{
    // Get the threshold value
    double threshold =
        SystemParams::GetParams()->room_seg.walls_perpendicularity_thresh *
        Utils::DEG_TO_RAD;

    // Extract and normalize plane normals
    Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal().normalized();
    Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal().normalized();

    // Compute the absolute dot product (clamped for safety)
    double dotProduct = std::clamp(std::abs(normal1.dot(normal2)), -1.0, 1.0);

    // Calculate the angle between the planes
    double angle = std::acos(dotProduct);

    // Check if the angle is within the threshold
    return std::abs(angle - M_PI_2) < threshold;
}

bool Utils::arePlanesParallel(const ORB_SLAM3::Plane *plane1,
                              const ORB_SLAM3::Plane *plane2)
{
    // Get the threshold value
    double threshold =
        SystemParams::GetParams()->room_seg.walls_parallelism_thresh *
        Utils::DEG_TO_RAD;

    // Extract and normalize plane normals
    Eigen::Vector3d normal1 = plane1->getGlobalEquation().normal().normalized();
    Eigen::Vector3d normal2 = plane2->getGlobalEquation().normal().normalized();

    // Compute the dot product (clamped to avoid floating-point domain errors)
    double dotProduct = std::clamp(std::abs(normal1.dot(normal2)), -1.0, 1.0);

    // Compute the angle between normals in radians
    double angle = std::acos(dotProduct);

    // Planes are parallel if their normals are within threshold (0° or 180°)
    return (angle < threshold) || (std::abs(angle - M_PI) < threshold);
}

std::vector<std::pair<ORB_SLAM3::Plane *, ORB_SLAM3::Plane *>>
    Utils::getFacingPlanes(const std::vector<ORB_SLAM3::Plane *> &planes)
{
    // Variables
    ORB_SLAM3::SystemParams *sysParams = ORB_SLAM3::SystemParams::GetParams();
    std::vector<std::pair<ORB_SLAM3::Plane *, ORB_SLAM3::Plane *>> facingPlanes;
    double minValidSpace = sysParams->room_seg.min_wall_distance_thresh;

    // Loop through all the planes
    for (size_t idx1 = 0; idx1 < planes.size(); ++idx1)
    {
        ORB_SLAM3::Plane *plane1 = planes[idx1];
        for (size_t idx2 = idx1 + 1; idx2 < planes.size(); ++idx2)
        {
            // Variables
            ORB_SLAM3::Plane *plane2 = planes[idx2];
            // Check if the planes are facing each other
            bool isFacing = Utils::arePlanesFacingEachOther(plane1, plane2);
            if (isFacing)
            {
                if (Utils::arePlanesApartEnough(plane1, plane2, minValidSpace))
                    facingPlanes.push_back(std::make_pair(plane1, plane2));
            }
        }
    }
    return facingPlanes;
}

Eigen::Vector4d Utils::correctPlaneDirection(const Eigen::Vector4d &plane)
{
    // Check if the transformation is needed
    if (plane(3) > 0)
        return -plane;
    else
        return plane;
}

g2o::Plane3D Utils::applyPoseToPlane(const Eigen::Matrix4d &kfPose,
                                     const g2o::Plane3D    &plane)
{
    Eigen::Vector4d v = plane.coeffs();
    Eigen::Vector4d v2;
    Eigen::Matrix3d R = kfPose.block<3, 3>(0, 0);
    v2.head<3>()      = R * v.head<3>();
    v2(3)             = v(3) - kfPose.block<3, 1>(0, 3).dot(v2.head<3>());
    return g2o::Plane3D(v2);
}

Eigen::Vector3d
    Utils::computeCentroidFromPoints(const std::vector<Eigen::Vector3d> &points)
{
    // Check if there are points in the vector
    if (points.empty())
        return Eigen::Vector3d(0.0, 0.0, 0.0);

    // Variables
    Eigen::Vector3d sum(0.0, 0.0, 0.0);

    // Calculate the sum of the points
    for (const auto &point : points)
        sum += point;

    // Return the centroid of the cluster
    return sum / points.size();
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDownsample(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const float                                  leafSize,
    const unsigned int                           minPointsPerVoxel)
{
    // The filtered point cloud object
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(
        new pcl::PointCloud<PointT>());

    // Define the downsampling filter
    typename pcl::VoxelGrid<PointT>::Ptr downsampleFilter(
        new pcl::VoxelGrid<PointT>());

    // Set the parameters of the downsampling filter
    downsampleFilter->setLeafSize(leafSize, leafSize, leafSize);
    downsampleFilter->setMinimumPointsNumberPerVoxel(minPointsPerVoxel);
    downsampleFilter->setInputCloud(cloud);

    // Apply the downsampling filter
    downsampleFilter->filter(*filteredCloud);
    filteredCloud->header = cloud->header;
    filteredCloud->width  = filteredCloud->size();
    filteredCloud->height = 1;

    return filteredCloud;
}
template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
        const float,
        const unsigned int);

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDistanceFilter(
    const typename pcl::PointCloud<PointT>::Ptr &cloud)
{
    // Variables
    double                        distance;
    const std::pair<float, float> thresholds =
        SystemParams::GetParams()->pointcloud.distance_thresh;
    const float thresholdNear = thresholds.first;
    const float thresholdFar  = thresholds.second;

    // Define the filtered point cloud object
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(
        new pcl::PointCloud<PointT>());
    filteredCloud->reserve(cloud->size());

    // Filter the point cloud
    std::copy_if(cloud->begin(),
                 cloud->end(),
                 std::back_inserter(filteredCloud->points),
                 [&](const PointT &p)
                 {
                     /*!
                      * Filter points based on distance along z axis from
                      * sensor.
                      *
                      * @note:      The z axis is pointing away from the
                      *             sensor.
                      */
                     distance = p.z;
                     return distance > thresholdNear && distance < thresholdFar;
                 });

    filteredCloud->height   = 1;
    filteredCloud->is_dense = false;
    filteredCloud->header   = cloud->header;
    filteredCloud->width    = filteredCloud->size();

    return filteredCloud;
}
template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudOutlierRemoval(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const int                                    meanThresh,
    const float                                  stdDevThresh)
{
    // Check if the input cloud is empty
    if (cloud->points.size() == 0)
        return cloud;

    // Create a container for the filtered cloud
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(
        new pcl::PointCloud<PointT>);

    // Create the filtering object: StatisticalOutlierRemoval
    pcl::StatisticalOutlierRemoval<PointT> outlierRemoval;
    outlierRemoval.setInputCloud(cloud);
    outlierRemoval.setMeanK(meanThresh);
    outlierRemoval.setStddevMulThresh(stdDevThresh);
    outlierRemoval.filter(*filteredCloud);

    filteredCloud->header = cloud->header;
    filteredCloud->width  = filteredCloud->size();
    filteredCloud->height = 1;

    // Return the filtered cloud
    return filteredCloud;
}
template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
        const int,
        const float);

std::pair<double, double> Utils::computePlaneWidthHeight(
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
    if (cloud->points.empty())
        return std::make_pair(0.0, 0.0);

    // Apply PCA to find the principal axes of the point cloud
    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(cloud);

    // Project all points onto the PCA space
    pcl::PointCloud<pcl::PointXYZRGBA> projected;
    pca.project(*cloud, projected);

    // Find min/max along each principal axis
    pcl::PointXYZRGBA minPt, maxPt;
    pcl::getMinMax3D(projected, minPt, maxPt);

    // Axis 0 = largest variance (width), Axis 1 = second (height)
    double width  = static_cast<double>(maxPt.y - minPt.y);
    double height = static_cast<double>(maxPt.x - minPt.x);

    return std::make_pair(width, height);
}

template <typename PointT, template <typename> class SegmentationType>
std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting(typename pcl::PointCloud<PointT>::Ptr &cloud)
{
    /* Initialize Variables */
    std::vector<
        std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
                  extractedPlanes;
    SystemParams *sysParams = SystemParams::GetParams();

    /* Extract planes from point clouds */
    for (unsigned int i = 0;
         i < sysParams->seg.ransac.max_planes &&
         cloud->points.size() > sysParams->seg.pointclouds_thresh;
         i++)
    {
        try
        {
            /* Create objects for RANSAC plane segmentation */
            typename pcl::ExtractIndices<PointT> extract;
            pcl::PointIndices::Ptr               inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

            /* Create the SAC segmentation object */
            SegmentationType<PointT> seg;

            /* Fill the values of the segmentation object */
            seg.setInputCloud(cloud);
            seg.setNumberOfThreads(8);
            seg.setMaxIterations(sysParams->seg.ransac.max_iterations);
            seg.setDistanceThreshold(sysParams->seg.ransac.distance_thresh);
            seg.setOptimizeCoefficients(true);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setModelType(pcl::SACMODEL_PLANE);

            /*!
             * Apply RANSAC segmentation.
             * Inliers contains the index of points used to find coefficients
             */
            seg.segment(*inliers, *coeffs);

            if (inliers->indices.empty() || coeffs->values.size() < 4)
            {
                break;
            }

            /* Calculate normal on the plane */
            Eigen::Vector4d planeEquation(coeffs->values[0],
                                          coeffs->values[1],
                                          coeffs->values[2],
                                          coeffs->values[3]);

            /* Calculate the closest points */
            Eigen::Vector4d plane;
            Eigen::Vector3d closestPoint =
                planeEquation.head(3) * planeEquation(3);
            plane.head(3) = closestPoint / closestPoint.norm();
            plane(3)      = closestPoint.norm();

            /* Init an object to contain a pointcloud within the plane */
            typename pcl::PointCloud<PointT>::Ptr extractedCloud(
                new pcl::PointCloud<PointT>);

            /* Create a point cloud containing the points within the plane */
            for (const auto &idx : inliers->indices)
            {
                /* Fill the point cloud with indices */
                PointT inPoint;
                inPoint.r = cloud->points[idx].r;
                inPoint.g = cloud->points[idx].g;
                inPoint.b = cloud->points[idx].b;
                inPoint.x = cloud->points[idx].x;
                inPoint.y = cloud->points[idx].y;
                inPoint.z = cloud->points[idx].z;
                inPoint.a = cloud->points[idx].a;

                /* Add the point to the extrcated cloud of the plane */
                extractedCloud->points.push_back(inPoint);
            }

            /* Set the width of the cloud */
            extractedCloud->width =
                static_cast<std::uint32_t>(extractedCloud->points.size());

            /* Set the height of the cloud */
            extractedCloud->height = 1;

            /* Extract flag to indicate that the cloud is dense */
            extractedCloud->is_dense = cloud->is_dense;

            /* Add the extracted cloud to the vector */
            extractedPlanes.push_back(std::make_pair(extractedCloud, plane));

            /* Remove the inliers into a distinct output cloud. PCL filters do
             * not preserve organized-cloud metadata reliably when their input
             * and output alias; that produced width/size mismatches during
             * repeated plane extraction. */
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            typename pcl::PointCloud<PointT>::Ptr p_remainingCloud(
                new pcl::PointCloud<PointT>);
            extract.filter(*p_remainingCloud);
            p_remainingCloud->header   = cloud->header;
            p_remainingCloud->width    = p_remainingCloud->size();
            p_remainingCloud->height   = 1;
            p_remainingCloud->is_dense = cloud->is_dense;
            cloud                      = std::move(p_remainingCloud);
        }
        catch (const std::exception &e)
        {
            std::cout << "RANSAC model error!" << std::endl;
        }
    }
    return extractedPlanes;
}
template std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
template std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

ORB_SLAM3::Plane::planeVariant Utils::getPlaneTypeFromClassId(int clsId)
{
    switch (clsId)
    {
    case 0:
        return ORB_SLAM3::Plane::planeVariant::GROUND;
    case 1:
        return ORB_SLAM3::Plane::planeVariant::WALL;
    case 2:
        return ORB_SLAM3::Plane::planeVariant::DOOR;
    case 3:
        return ORB_SLAM3::Plane::planeVariant::WINDOW;
    default:
        return ORB_SLAM3::Plane::planeVariant::UNDEFINED;
    }
}

int Utils::getClassIdFromPlaneType(ORB_SLAM3::Plane::planeVariant planeType)
{
    switch (planeType)
    {
    case ORB_SLAM3::Plane::planeVariant::GROUND:
        return 0;
    case ORB_SLAM3::Plane::planeVariant::WALL:
        return 1;
    case ORB_SLAM3::Plane::planeVariant::DOOR:
        return 2;
    case ORB_SLAM3::Plane::planeVariant::WINDOW:
        return 3;
    default:
        return -1;
    }
}

bool Utils::pointOnPlane(Eigen::Vector4d planeEquation, MapPoint *mapPoint)
{
    if (mapPoint->isBad())
        return false;

    // Find the distance of the point from a given plane
    double pointPlaneDist =
        calculateDistancePointToPlane(planeEquation,
                                      mapPoint->GetWorldPos().cast<double>());

    // Apply a threshold
    if (pointPlaneDist < SystemParams::GetParams()->seg.plane_point_dist_thresh)
        return true;

    return false;
}

int Utils::associatePlanes(const vector<Plane *>                  &mappedPlanes,
                           g2o::Plane3D                            givenPlane,
                           pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr givenCloud,
                           const Eigen::Matrix4d                  &kfPose,
                           const Plane::planeVariant               obsPlaneType,
                           const float                             threshold,
                           const float maximumFiniteCloudDistance_m_in,
                           const std::optional<Eigen::Vector3d>
                               &observationOrigin_World_m_in)
{
    /* Return no association when no mapped planes are available */
    if (mappedPlanes.empty())
    {
        return -1;
    }

    /* Confirm the observed plane point cloud is valid */
    if (givenCloud == nullptr || givenCloud->empty())
    {
        return -1;
    }

    /* Extract the system parameters */
    SystemParams *sysParams = SystemParams::GetParams();

    /* Extract and normalize the observed plane equation */
    Eigen::Vector4d givenEquation   = givenPlane.coeffs();
    const double    givenNormalNorm = givenEquation.head<3>().norm();

    /* Confirm norm is valid */
    if (!std::isfinite(givenNormalNorm) || givenNormalNorm < 1e-8)
    {
        return -1;
    }

    /* Find the unit vector */
    givenEquation /= givenNormalNorm;

    /* Calculate the centroid of the observed global point cloud */
    Eigen::Vector3d givenCentroid = Eigen::Vector3d::Zero();

    std::size_t validGivenPointCount = 0;

    for (const pcl::PointXYZRGBA &point : givenCloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        givenCentroid += Eigen::Vector3d(static_cast<double>(point.x),
                                         static_cast<double>(point.y),
                                         static_cast<double>(point.z));

        validGivenPointCount++;
    }

    /* Return when the point cloud has no valid points */
    if (validGivenPointCount == 0)
    {
        return -1;
    }

    givenCentroid /= static_cast<double>(validGivenPointCount);

    /*!
     * Association thresholds.
     *
     * @note        The ominus threshold is used here as the maximum angular
     *              difference in radians.
     */
    const double maximumAngularDifference =
        std::max(0.01, static_cast<double>(threshold));

    const double maximumPlaneDistance = std::max(
        0.01,
        static_cast<double>(sysParams->seg.plane_association.distance_thresh));

    const double maximumCentroidDistance = std::max(
        0.10,
        static_cast<double>(sysParams->seg.plane_association.centroid_thresh));

    const bool useWallExtension =
        obsPlaneType == Plane::planeVariant::WALL &&
        sysParams->sem_seg.reassociate.wallExtension.enabled;

    const double configuredFiniteCloudDistance_m =
        maximumFiniteCloudDistance_m_in > 0.0F
            ? static_cast<double>(maximumFiniteCloudDistance_m_in)
        : useWallExtension
            ? static_cast<double>(sysParams->sem_seg.reassociate.wallExtension
                                      .maximumInPlaneGap_m)
            : static_cast<double>(sysParams->seg.plane_association
                                      .cluster_separation.tolerance);

    const double maximumFiniteCloudDistance =
        std::max(0.05, configuredFiniteCloudDistance_m);

    /*!
     * Minimum fraction of sampled observation points which must be close to
     * the mapped finite plane cloud when the centroids are far apart.
     */
    constexpr double minimumFiniteOverlapRatio = 0.10;

    /*!
     * Limit the number of nearest-neighbour searches for each candidate.
     */
    constexpr std::size_t maximumSampleCount = 300;

    /* Track the best mapped-plane candidate */
    int bestPlaneId = -1;

    double bestAssociationScore = std::numeric_limits<double>::max();

    /* Iterate through every mapped plane */
    for (Plane *mappedPlane : mappedPlanes)
    {
        /* Skip invalid mapped planes */
        if (mappedPlane == nullptr || mappedPlane->isBad())
        {
            continue;
        }

        /* Extract the mapped plane point cloud */
        const Plane::GeometrySnapshot mappedGeometry =
            mappedPlane->getGeometrySnapshot();
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr mappedCloud =
            mappedGeometry.supportCloud;

        /* Skip mapped planes without finite geometry */
        if (mappedCloud == nullptr || mappedCloud->empty())
        {
            continue;
        }

        /*
         * Check semantic compatibility.
         *
         * A mapped UNDEFINED plane is allowed to match a semantically labelled
         * observation so that it can accumulate enough votes for confirmation.
         */
        const Plane::planeVariant mappedPlaneType =
            mappedPlane->getExpectedPlaneType();

        const bool semanticTypesCompatible =
            obsPlaneType == Plane::planeVariant::UNDEFINED ||
            mappedPlaneType == Plane::planeVariant::UNDEFINED ||
            mappedPlaneType == obsPlaneType;

        if (!semanticTypesCompatible)
        {
            continue;
        }

        /*!
         * Transform the mapped equation into the frame used by the supplied
         * observation.
         *
         * @note        SemanticSegmentation now supplies both planes in the
         *              global frame, therefore kfPose is normally identity.
         */
        const g2o::Plane3D mappedPlaneInGivenFrame =
            Utils::applyPoseToPlane(kfPose,
                                    g2o::Plane3D(mappedGeometry.equation_World));

        Eigen::Vector4d mappedEquation = mappedPlaneInGivenFrame.coeffs();

        const double mappedNormalNorm = mappedEquation.head<3>().norm();

        if (!std::isfinite(mappedNormalNorm) || mappedNormalNorm < 1e-8)
        {
            continue;
        }

        mappedEquation /= mappedNormalNorm;

        /*!
         * Ensure both equations use the same normal direction before comparing
         * their distance coefficients.
         */
        if (givenEquation.head<3>().dot(mappedEquation.head<3>()) < 0.0)
        {
            mappedEquation *= -1.0;
        }

        /* Keep observations from opposite sides as distinct wall faces. */
        if (obsPlaneType == Plane::planeVariant::WALL &&
            observationOrigin_World_m_in.has_value() &&
            observationOrigin_World_m_in->allFinite())
        {
            const ObservationSideEvidence mappedObservationSide =
                getMedianObservationSide_World_m(mappedPlane, mappedEquation);

            const double givenObservationSide_m =
                mappedEquation.head<3>().dot(
                    observationOrigin_World_m_in.value()) +
                mappedEquation(3);

            constexpr double minimumReliableSideDistance_m = 0.10;

            if (mappedObservationSide.ambiguous)
            {
                continue;
            }

            if (mappedObservationSide.medianSignedDistance_m.has_value() &&
                std::abs(givenObservationSide_m) >=
                    minimumReliableSideDistance_m &&
                mappedObservationSide.medianSignedDistance_m.value() *
                        givenObservationSide_m <
                    0.0)
            {
                continue;
            }
        }

        /* Calculate the angular difference between the plane normals */
        const double normalAlignment =
            std::clamp(givenEquation.head<3>().dot(mappedEquation.head<3>()),
                       -1.0,
                       1.0);

        const double angularDifference = std::acos(normalAlignment);

        /* Reject planes whose normals are not sufficiently aligned */
        if (angularDifference > maximumAngularDifference)
        {
            continue;
        }

        /* Calculate perpendicular separation between the planes */
        const double planeDistance =
            std::abs(givenEquation(3) - mappedEquation(3));

        /* Reject parallel planes which are physically separated */
        if (planeDistance > maximumPlaneDistance)
        {
            continue;
        }

        const bool finiteWallExtentsCompatible =
            useWallExtension && finiteWallExtentsAreCompatible(
                                    mappedCloud,
                                    givenCloud,
                                    mappedEquation.head<3>(),
                                    sysParams->sem_seg.reassociate.wallExtension
                                        .maximumInPlaneGap_m,
                                    sysParams->sem_seg.reassociate.wallExtension
                                        .minimumOrthogonalOverlap_m);

        /* Extract the global mapped-plane centroid */
        const Eigen::Vector3d mappedCentroid = mappedGeometry.centroid_World_m;

        /* Calculate the global centroid distance */
        const double centroidDistance = (givenCentroid - mappedCentroid).norm();

        /*!
         * Measure finite-cloud compatibility using nearest-neighbour distance.
         *
         * This prevents distant coplanar surfaces from being merged while
         * allowing neighbouring fragments of the same physical wall to join.
         */
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> mappedCloudSearch;

        mappedCloudSearch.setInputCloud(mappedCloud);

        const std::size_t samplingStride =
            std::max<std::size_t>(1, givenCloud->size() / maximumSampleCount);

        std::size_t sampledPointCount     = 0;
        std::size_t overlappingPointCount = 0;

        double minimumCloudDistance = std::numeric_limits<double>::max();

        std::vector<int> nearestPointIndex(1);

        std::vector<float> nearestSquaredDistance(1);

        for (std::size_t pointIndex = 0; pointIndex < givenCloud->size();
             pointIndex += samplingStride)
        {
            const pcl::PointXYZRGBA &queryPoint =
                givenCloud->points[pointIndex];

            if (!pcl::isFinite(queryPoint))
            {
                continue;
            }

            sampledPointCount++;

            const int neighbourCount =
                mappedCloudSearch.nearestKSearch(queryPoint,
                                                 1,
                                                 nearestPointIndex,
                                                 nearestSquaredDistance);

            if (neighbourCount <= 0)
            {
                continue;
            }

            const double cloudDistance =
                std::sqrt(static_cast<double>(nearestSquaredDistance.front()));

            minimumCloudDistance =
                std::min(minimumCloudDistance, cloudDistance);

            if (cloudDistance <= maximumFiniteCloudDistance)
            {
                overlappingPointCount++;
            }
        }

        const double finiteOverlapRatio =
            sampledPointCount > 0 ? static_cast<double>(overlappingPointCount) /
                                        static_cast<double>(sampledPointCount)
                                  : 0.0;

        if (useWallExtension && !finiteWallExtentsCompatible &&
            finiteOverlapRatio < minimumFiniteOverlapRatio)
        {
            continue;
        }

        /*!
         * Planes with very close centroids and compatible normals are likely
         * duplicate estimates of the same physical surface.
         */
        const bool centroidsAreClose =
            centroidDistance <= maximumCentroidDistance;

        /*!
         * Plane fragments with separated centroids may still belong to the same
         * physical wall when their finite point clouds overlap or are adjacent.
         */
        const std::size_t minimumAdjacentPointCount = std::max<std::size_t>(
            3,
            static_cast<std::size_t>(
                std::ceil(0.02 * static_cast<double>(sampledPointCount))));

        const bool finiteCloudsCompatible =
            finiteWallExtentsCompatible ||
            finiteOverlapRatio >= minimumFiniteOverlapRatio ||
            (minimumCloudDistance <= maximumFiniteCloudDistance &&
             overlappingPointCount >= minimumAdjacentPointCount);

        /*!
         * Require either a direct centroid match or finite point-cloud
         * compatibility.
         *
         * @note        The angular and perpendicular plane-distance checks have
         *              already been applied above. Therefore, planes with the
         * same centroid but significantly different normals are not merged.
         */
        if (!centroidsAreClose && !finiteCloudsCompatible)
        {
            continue;
        }

        /* Normalize each component used by the association score */
        const double normalizedAngularDifference =
            angularDifference / maximumAngularDifference;

        const double normalizedPlaneDistance =
            planeDistance / maximumPlaneDistance;

        const double normalizedCentroidDistance =
            std::min(centroidDistance / maximumCentroidDistance, 2.0);

        const double normalizedCloudDistance =
            std::isfinite(minimumCloudDistance)
                ? std::min(minimumCloudDistance / maximumFiniteCloudDistance,
                           2.0)
                : 2.0;

        /*!
         * Calculate a combined score for the mapped-plane candidate.
         *
         * Lower scores represent stronger associations.
         */
        const double associationScore =
            3.0 * normalizedAngularDifference + 4.0 * normalizedPlaneDistance +
            0.25 * normalizedCentroidDistance + 0.50 * normalizedCloudDistance -
            2.0 * finiteOverlapRatio;

        /* Keep the strongest valid mapped-plane candidate */
        if (associationScore < bestAssociationScore)
        {
            bestAssociationScore = associationScore;

            bestPlaneId = mappedPlane->getId();
        }
    }

    return bestPlaneId;
}

void Utils::clusterPlaneClouds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
    std::vector<pcl::PointIndices>                &clusterIndices)
{
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(
        SystemParams::GetParams()
            ->seg.plane_association.cluster_separation.tolerance);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
}

void Utils::reAssociateSemanticPlanes(Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return;
    }

    SystemParams *p_systemParams = SystemParams::GetParams();

    bool mergedPlaneInPass = true;

    while (mergedPlaneInPass)
    {
        mergedPlaneInPass = false;

        const std::vector<Plane *> mappedPlanes = p_atlas_in->GetAllPlanes();

        for (Plane *p_candidatePlane : mappedPlanes)
        {
            if (p_candidatePlane == nullptr || p_candidatePlane->isBad() ||
                p_candidatePlane->getPlaneType() ==
                    Plane::planeVariant::UNDEFINED)
            {
                continue;
            }

            std::vector<Plane *> compatiblePlanes;
            compatiblePlanes.reserve(mappedPlanes.size());

            for (Plane *p_otherPlane : mappedPlanes)
            {
                if (p_otherPlane == nullptr ||
                    p_otherPlane == p_candidatePlane || p_otherPlane->isBad() ||
                    p_otherPlane->getPlaneType() !=
                        p_candidatePlane->getPlaneType())
                {
                    continue;
                }

                /*
                 * Never merge wall faces observed from opposite sides. Their
                 * equations can be nearly identical when wall thickness is
                 * below the association threshold, but they bound different
                 * rooms and require independent ownership.
                 */
                if (p_candidatePlane->getPlaneType() ==
                    Plane::planeVariant::WALL)
                {
                    const Plane::GeometrySnapshot candidateGeometry =
                        p_candidatePlane->getGeometrySnapshot();
                    const Plane::GeometrySnapshot otherGeometry =
                        p_otherPlane->getGeometrySnapshot();
                    Eigen::Vector4d candidateEquation_World =
                        candidateGeometry.equation_World;
                    Eigen::Vector4d otherEquation_World =
                        otherGeometry.equation_World;

                    const double candidateNormalNorm =
                        candidateEquation_World.head<3>().norm();
                    const double otherNormalNorm =
                        otherEquation_World.head<3>().norm();

                    if (candidateEquation_World.allFinite() &&
                        otherEquation_World.allFinite() &&
                        candidateNormalNorm >= 1e-8 && otherNormalNorm >= 1e-8)
                    {
                        candidateEquation_World /= candidateNormalNorm;
                        otherEquation_World /= otherNormalNorm;

                        if (candidateEquation_World.head<3>().dot(
                                otherEquation_World.head<3>()) < 0.0)
                        {
                            otherEquation_World *= -1.0;
                        }

                        const ObservationSideEvidence candidateObservationSide =
                            getMedianObservationSide_World_m(
                                p_candidatePlane,
                                candidateEquation_World);
                        const ObservationSideEvidence otherObservationSide =
                            getMedianObservationSide_World_m(
                                p_otherPlane,
                                otherEquation_World);

                        if (candidateObservationSide.ambiguous ||
                            otherObservationSide.ambiguous)
                        {
                            continue;
                        }

                        if (candidateObservationSide.medianSignedDistance_m
                                .has_value() &&
                            otherObservationSide.medianSignedDistance_m
                                .has_value() &&
                            candidateObservationSide.medianSignedDistance_m
                                    .value() *
                                    otherObservationSide
                                        .medianSignedDistance_m.value() <
                                0.0)
                        {
                            continue;
                        }

                        /* Finite support is checked once by associatePlanes(),
                         * including nearest-neighbour partial overlap. */
                    }
                }

                compatiblePlanes.push_back(p_otherPlane);
            }

            if (compatiblePlanes.empty())
            {
                continue;
            }

            const bool useWallExtensionDistance =
                p_candidatePlane->getPlaneType() == Plane::planeVariant::WALL &&
                p_systemParams->sem_seg.reassociate.wallExtension.enabled;

            const float maximumFiniteCloudDistance_m =
                useWallExtensionDistance
                    ? p_systemParams->sem_seg.reassociate.wallExtension
                          .maximumInPlaneGap_m
                    : -1.0F;

            const Plane::GeometrySnapshot candidateAssociationGeometry =
                p_candidatePlane->getGeometrySnapshot();
            const int matchedPlaneId = associatePlanes(
                compatiblePlanes,
                g2o::Plane3D(candidateAssociationGeometry.equation_World),
                candidateAssociationGeometry.supportCloud,
                Eigen::Matrix4d::Identity(),
                p_candidatePlane->getPlaneType(),
                p_systemParams->sem_seg.reassociate.association_thresh,
                maximumFiniteCloudDistance_m);

            if (matchedPlaneId < 0)
            {
                continue;
            }

            const auto matchedPlaneIterator =
                std::find_if(compatiblePlanes.begin(),
                             compatiblePlanes.end(),
                             [matchedPlaneId](const Plane *p_plane) {
                                 return p_plane != nullptr &&
                                        p_plane->getId() == matchedPlaneId;
                             });

            if (matchedPlaneIterator == compatiblePlanes.end())
            {
                continue;
            }

            Plane *p_matchedPlane = *matchedPlaneIterator;

            const Plane::GeometrySnapshot candidateGeometry =
                p_candidatePlane->getGeometrySnapshot();
            const Plane::GeometrySnapshot matchedGeometry =
                p_matchedPlane->getGeometrySnapshot();
            const auto candidateEvidence = std::make_tuple(
                p_candidatePlane->getObservationCount(),
                candidateGeometry.supportCloud != nullptr
                    ? candidateGeometry.supportCloud->size()
                    : 0U,
                -p_candidatePlane->getId());

            const auto matchedEvidence = std::make_tuple(
                p_matchedPlane->getObservationCount(),
                matchedGeometry.supportCloud != nullptr
                    ? matchedGeometry.supportCloud->size()
                    : 0U,
                -p_matchedPlane->getId());

            Plane *p_retainedPlane = candidateEvidence >= matchedEvidence
                                         ? p_candidatePlane
                                         : p_matchedPlane;

            Plane *p_retiredPlane = p_retainedPlane == p_candidatePlane
                                        ? p_matchedPlane
                                        : p_candidatePlane;

            const Plane::GeometrySnapshot retiredGeometry =
                p_retiredPlane->getGeometrySnapshot();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr retiredCloudCopy(
                new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (retiredGeometry.supportCloud != nullptr)
            {
                *retiredCloudCopy = *retiredGeometry.supportCloud;
                p_retainedPlane->setMapClouds(retiredCloudCopy);
            }

            for (MapPoint *p_mapPoint : p_retiredPlane->getMapPoints())
            {
                if (p_mapPoint != nullptr && !p_mapPoint->isBad())
                {
                    p_retainedPlane->setMapPoints(p_mapPoint);
                }
            }

            const std::map<KeyFrame *, Plane::Observation> retiredObservations =
                p_retiredPlane->getObservations();

            for (const auto &[p_keyFrame, observation] : retiredObservations)
            {
                if (p_keyFrame == nullptr || p_keyFrame->isBad())
                {
                    continue;
                }

                p_retainedPlane->mergeObservation(p_keyFrame, observation);
            }

            GeoSemHelpers::refitMappedPlaneFromCloud(p_retainedPlane);
            const Plane::planeVariant retainedPlaneType =
                p_retainedPlane->getPlaneType();

            for (Room *p_room : p_atlas_in->GetAllRooms())
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    continue;
                }

                p_room->replaceWall(p_retiredPlane, p_retainedPlane);
                p_room->replaceGroundPlane(p_retiredPlane, p_retainedPlane);
            }

            for (ORB_SLAM3::Passage *p_passage : p_atlas_in->GetAllPassages())
            {
                if (p_passage != nullptr)
                {
                    p_passage->replacePlaneAssociation(p_retiredPlane,
                                                       p_retainedPlane);
                }
            }

            for (KeyFrame *p_keyFrame : p_atlas_in->GetAllKeyFrames())
            {
                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    p_keyFrame->ReplaceMapPlane(p_retiredPlane,
                                                p_retainedPlane);
                }
            }

            Map *p_currentMap = p_atlas_in->GetCurrentMap();

            if (p_currentMap != nullptr)
            {
                p_currentMap->EraseRoomWallPlane(p_retiredPlane);

                if (retainedPlaneType == Plane::planeVariant::WALL)
                {
                    p_currentMap->AddRoomWallPlane(p_retainedPlane);
                }
            }

            /*
             * Invalidate only after every graph edge points at the survivor,
             * then remove the retired hypothesis from the map container and
             * ID index so later merge passes cannot rediscover stale state.
             */
            p_retiredPlane->setBad();

            if (p_currentMap != nullptr)
            {
                p_currentMap->EraseMapPlane(p_retiredPlane);
            }

            p_retiredPlane->SetMap(nullptr);

            std::cout << "[SemanticMerge] Fused Plane#"
                      << p_retiredPlane->getId() << " into Plane#"
                      << p_retainedPlane->getId() << '.' << std::endl;

            mergedPlaneInPass = true;
            break;
        }
    }
}

void Utils::reAssociateRooms(Atlas *mpAtlas)
{
    /*!
     * Re-run the targeted provisional-room consolidation pass.
     *
     * @note        This function does not merge confirmed rooms. It only allows
     *              a confirmed room or corridor to absorb redundant,
     *              single-wall provisional structural elements.
     */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    for (ORB_SLAM3::Room *room : allRooms)
    {
        /* Skip invalid structural elements */
        if (room == nullptr || room->isBad())
        {
            continue;
        }

        /*!
         * Only confirmed rooms may absorb provisional
         * structural elements.
         */
        const bool isConfirmedRoom =
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::ROOM;

        if (!isConfirmedRoom)
        {
            continue;
        }

        /* Require more than one valid wall before allowing consolidation */
        std::size_t validWallCount = 0;

        const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        for (ORB_SLAM3::Plane *wall : roomWalls)
        {
            if (wall != nullptr && !wall->isBad())
            {
                validWallCount++;
            }
        }

        if (validWallCount < 2)
        {
            continue;
        }

        /*!
         * Consolidate only redundant single-wall provisional structural
         * elements whose wall already belongs to this confirmed room.
         */
        Utils::consolidateProvisionalRooms(room, mpAtlas);
    }
}

void Utils::fuseDuplicateRoomsAfterMerge(
    Map                       *p_map_inout,
    const std::vector<Room *> &importedRooms_in)
{
    /* Reject an invalid lifecycle request. */
    if (p_map_inout == nullptr || importedRooms_in.empty())
    {
        return;
    }

    const SystemParams *p_systemParameters = SystemParams::GetParams();

    const double maximumRoomCentroidDistance_m =
        p_systemParameters != nullptr
            ? static_cast<double>(
                  p_systemParameters->room_seg.center_distance_thresh)
            : 2.0;

    constexpr double minimumRoomSideDistance_m = 0.20;
    constexpr double finiteWallBoundsMargin_m  = 0.30;

    const std::unordered_set<Room *> importedRoomSet(importedRooms_in.begin(),
                                                     importedRooms_in.end());

    /*
     * Report whether a finite mapped wall separates two room centres. An
     * infinite plane alone is insufficient because unrelated coplanar wall
     * segments are common in office environments.
     */
    const auto hasSeparatingFiniteWall =
        [p_map_inout](const Eigen::Vector3d &firstCentroid_World_m_in,
                      const Eigen::Vector3d &secondCentroid_World_m_in)
    {
        for (Plane *p_wall : p_map_inout->GetAllPlanes())
        {
            if (p_wall == nullptr || p_wall->isBad() ||
                p_wall->getPlaneType() != Plane::planeVariant::WALL)
            {
                continue;
            }

            const Plane::GeometrySnapshot wallGeometry =
                p_wall->getGeometrySnapshot();
            Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;

            const double wallNormalNorm = wallEquation_World.head<3>().norm();

            if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
            {
                continue;
            }

            wallEquation_World /= wallNormalNorm;

            const double firstSignedDistance_m =
                wallEquation_World.head<3>().dot(firstCentroid_World_m_in) +
                wallEquation_World(3);

            const double secondSignedDistance_m =
                wallEquation_World.head<3>().dot(secondCentroid_World_m_in) +
                wallEquation_World(3);

            if (firstSignedDistance_m * secondSignedDistance_m >= 0.0 ||
                std::abs(firstSignedDistance_m) < minimumRoomSideDistance_m ||
                std::abs(secondSignedDistance_m) < minimumRoomSideDistance_m)
            {
                continue;
            }

            const double interpolation =
                firstSignedDistance_m /
                (firstSignedDistance_m - secondSignedDistance_m);

            if (interpolation <= 0.0 || interpolation >= 1.0)
            {
                continue;
            }

            const Eigen::Vector3d intersection_World_m =
                firstCentroid_World_m_in +
                interpolation *
                    (secondCentroid_World_m_in - firstCentroid_World_m_in);

            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
                wallGeometry.supportCloud;

            if (p_wallCloud == nullptr || p_wallCloud->empty())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                wallGeometry.centroid_World_m;

            const Eigen::Vector3d wallAxisU_World =
                wallEquation_World.head<3>().unitOrthogonal().normalized();

            const Eigen::Vector3d wallAxisV_World = wallEquation_World.head<3>()
                                                        .cross(wallAxisU_World)
                                                        .normalized();

            double      minimumWallU_m = std::numeric_limits<double>::max();
            double      maximumWallU_m = std::numeric_limits<double>::lowest();
            double      minimumWallV_m = std::numeric_limits<double>::max();
            double      maximumWallV_m = std::numeric_limits<double>::lowest();
            std::size_t validWallPointCount = 0U;

            for (const pcl::PointXYZRGBA &wallPoint : p_wallCloud->points)
            {
                if (!pcl::isFinite(wallPoint))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint_World_m(
                    static_cast<double>(wallPoint.x),
                    static_cast<double>(wallPoint.y),
                    static_cast<double>(wallPoint.z));

                const Eigen::Vector3d wallPointRelToCentroid_World_m =
                    wallPoint_World_m - wallCentroid_World_m;

                const double wallPointU_m =
                    wallPointRelToCentroid_World_m.dot(wallAxisU_World);

                const double wallPointV_m =
                    wallPointRelToCentroid_World_m.dot(wallAxisV_World);

                minimumWallU_m = std::min(minimumWallU_m, wallPointU_m);
                maximumWallU_m = std::max(maximumWallU_m, wallPointU_m);
                minimumWallV_m = std::min(minimumWallV_m, wallPointV_m);
                maximumWallV_m = std::max(maximumWallV_m, wallPointV_m);
                validWallPointCount++;
            }

            if (validWallPointCount == 0U)
            {
                continue;
            }

            const Eigen::Vector3d intersectionRelToWallCentroid_World_m =
                intersection_World_m - wallCentroid_World_m;

            const double intersectionU_m =
                intersectionRelToWallCentroid_World_m.dot(wallAxisU_World);

            const double intersectionV_m =
                intersectionRelToWallCentroid_World_m.dot(wallAxisV_World);

            const bool intersectionInsideFiniteWall =
                intersectionU_m >= minimumWallU_m - finiteWallBoundsMargin_m &&
                intersectionU_m <= maximumWallU_m + finiteWallBoundsMargin_m &&
                intersectionV_m >= minimumWallV_m - finiteWallBoundsMargin_m &&
                intersectionV_m <= maximumWallV_m + finiteWallBoundsMargin_m;

            if (intersectionInsideFiniteWall)
            {
                return true;
            }
        }

        return false;
    };

    /* Evaluate each imported hypothesis against rooms that already existed. */
    for (Room *p_importedRoom : importedRooms_in)
    {
        if (p_importedRoom == nullptr || p_importedRoom->isBad())
        {
            continue;
        }

        const Room::roomVariant importedRoomType =
            p_importedRoom->getRoomVariant();

        if (importedRoomType == Room::roomVariant::UNDEFINED)
        {
            continue;
        }

        const Eigen::Vector3d importedCentroid_World_m =
            p_importedRoom->getCentroid();

        if (!importedCentroid_World_m.allFinite())
        {
            continue;
        }

        const std::vector<Plane *> importedWalls = p_importedRoom->getWalls();

        Room  *p_bestRetainedRoom     = nullptr;
        double bestCentroidDistance_m = std::numeric_limits<double>::infinity();

        for (Room *p_candidateRoom : p_map_inout->GetAllRooms())
        {
            if (p_candidateRoom == nullptr ||
                p_candidateRoom == p_importedRoom || p_candidateRoom->isBad() ||
                importedRoomSet.count(p_candidateRoom) > 0U ||
                p_candidateRoom->getRoomVariant() != importedRoomType)
            {
                continue;
            }

            if (p_importedRoom->getHasKnownLabel() &&
                p_candidateRoom->getHasKnownLabel() &&
                p_importedRoom->getMetaMarkerId() !=
                    p_candidateRoom->getMetaMarkerId())
            {
                continue;
            }

            const Eigen::Vector3d candidateCentroid_World_m =
                p_candidateRoom->getCentroid();

            const double centroidDistance_m =
                (candidateCentroid_World_m - importedCentroid_World_m).norm();

            if (!candidateCentroid_World_m.allFinite() ||
                !std::isfinite(centroidDistance_m) ||
                centroidDistance_m > maximumRoomCentroidDistance_m ||
                centroidDistance_m >= bestCentroidDistance_m)
            {
                continue;
            }

            const std::vector<Plane *> candidateWalls =
                p_candidateRoom->getWalls();

            std::size_t sameSideSharedWallCount   = 0U;
            bool        hasOppositeSideSharedWall = false;

            for (Plane *p_importedWall : importedWalls)
            {
                if (p_importedWall == nullptr || p_importedWall->isBad())
                {
                    continue;
                }

                const bool wallIsShared =
                    std::find(candidateWalls.begin(),
                              candidateWalls.end(),
                              p_importedWall) != candidateWalls.end();

                if (!wallIsShared)
                {
                    continue;
                }

                Eigen::Vector4d wallEquation_World =
                    p_importedWall->getGlobalEquation().coeffs();

                const double wallNormalNorm =
                    wallEquation_World.head<3>().norm();

                if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
                {
                    continue;
                }

                wallEquation_World /= wallNormalNorm;

                const double importedSide_m =
                    wallEquation_World.head<3>().dot(importedCentroid_World_m) +
                    wallEquation_World(3);

                const double candidateSide_m = wallEquation_World.head<3>().dot(
                                                   candidateCentroid_World_m) +
                                               wallEquation_World(3);

                if (importedSide_m * candidateSide_m < 0.0 &&
                    std::abs(importedSide_m) >= minimumRoomSideDistance_m &&
                    std::abs(candidateSide_m) >= minimumRoomSideDistance_m)
                {
                    hasOppositeSideSharedWall = true;
                    break;
                }

                sameSideSharedWallCount++;
            }

            if (hasOppositeSideSharedWall || sameSideSharedWallCount == 0U ||
                hasSeparatingFiniteWall(importedCentroid_World_m,
                                        candidateCentroid_World_m))
            {
                continue;
            }

            p_bestRetainedRoom     = p_candidateRoom;
            bestCentroidDistance_m = centroidDistance_m;
        }

        if (p_bestRetainedRoom == nullptr)
        {
            continue;
        }

        const std::vector<Plane *> retainedWalls =
            p_bestRetainedRoom->getWalls();

        /*! A wall can bound the retained (near) room only when no passable
         * passage aperture separates its centroid from the retained room
         * centre. Otherwise it belongs to the far-side room (the passage's
         * prospective, or a confirmed room that already resolved that
         * prospective). Copying such a wall into the retained room would both
         * corrupt the near boundary and hand the far room's evidence to the
         * near room, so the far-side consultation happens here. */
        Plane *p_mergeGroundPlane = nullptr;

        for (Plane *p_plane : p_map_inout->GetAllPlanes())
        {
            if (p_plane != nullptr && !p_plane->isBad() &&
                p_plane->getPlaneType() == Plane::planeVariant::GROUND)
            {
                p_mergeGroundPlane = p_plane;
                break;
            }
        }

        Eigen::Vector3d mergeGroundNormal_World = Eigen::Vector3d::Zero();
        if (p_mergeGroundPlane != nullptr)
        {
            const Eigen::Vector4d groundEq =
                p_mergeGroundPlane->getGlobalEquation().coeffs();
            const double groundNormalNorm = groundEq.head<3>().norm();
            if (groundEq.allFinite() && groundNormalNorm > 1e-8)
            {
                mergeGroundNormal_World =
                    groundEq.head<3>() / groundNormalNorm;
            }
        }

        const double mergeOpeningMargin_m =
            p_systemParameters != nullptr
                ? static_cast<double>(
                      p_systemParameters->room_seg.passagePartition
                          .openingMargin_m)
                : 0.20;

        const double mergeMinimumSideDistance_m =
            p_systemParameters != nullptr
                ? static_cast<double>(
                      p_systemParameters->room_seg.passagePartition
                          .minimumSideDistance_m)
                : 0.30;

        const Eigen::Vector3d retainedCentroid_World_m =
            p_bestRetainedRoom->getCentroid();

        const std::vector<Passage *> mergePassages =
            p_map_inout->GetAllPassages();
        const bool roomsAreSeparatedByPassage = std::any_of(
            mergePassages.begin(),
            mergePassages.end(),
            [&retainedCentroid_World_m,
             &importedCentroid_World_m,
             &mergeGroundNormal_World,
             mergeOpeningMargin_m,
             mergeMinimumSideDistance_m](Passage *p_passage)
            {
                return crossesPassablePassageOpening(
                    retainedCentroid_World_m,
                    importedCentroid_World_m,
                    p_passage,
                    mergeGroundNormal_World,
                    mergeOpeningMargin_m,
                    mergeMinimumSideDistance_m);
            });

        if (roomsAreSeparatedByPassage)
        {
            std::cout << "[SemanticMerge] Preserved Room#"
                      << p_importedRoom->getId() << " and Room#"
                      << p_bestRetainedRoom->getId()
                      << "; a passable passage separates their centroids."
                      << std::endl;
            continue;
        }

        struct WallTransfer
        {
            Plane   *p_wall;
            Room    *p_targetRoom;
            Passage *p_separatingPassage;
        };

        std::vector<WallTransfer> wallTransfers;
        wallTransfers.reserve(importedWalls.size());
        std::vector<Room *> mapRooms = p_map_inout->GetAllRooms();
        std::sort(mapRooms.begin(),
                  mapRooms.end(),
                  [](const Room *p_first, const Room *p_second)
                  {
                      if (p_first == nullptr)
                      {
                          return false;
                      }
                      if (p_second == nullptr)
                      {
                          return true;
                      }
                      return p_first->getId() < p_second->getId();
                  });

        for (Plane *p_importedWall : importedWalls)
        {
            if (p_importedWall == nullptr || p_importedWall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d importedWallCentroid_World_m =
                p_importedWall->getCentroid().cast<double>();

            ORB_SLAM3::Passage *p_separatingPassage = nullptr;

            for (ORB_SLAM3::Passage *p_passage : mergePassages)
            {
                if (p_passage != nullptr &&
                    crossesPassablePassageOpening(
                        retainedCentroid_World_m,
                        importedWallCentroid_World_m,
                        p_passage,
                        mergeGroundNormal_World,
                        mergeOpeningMargin_m,
                        mergeMinimumSideDistance_m))
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            Room *p_targetRoom = p_bestRetainedRoom;

            /* Never introduce another owner when a distinct confirmed room
             * already owns this wall. The imported duplicate is retired below,
             * leaving that confirmed owner unchanged. */
            for (Room *p_existingOwner : mapRooms)
            {
                if (p_existingOwner == nullptr || p_existingOwner->isBad() ||
                    p_existingOwner == p_importedRoom ||
                    p_existingOwner == p_bestRetainedRoom ||
                    p_existingOwner->getRoomVariant() !=
                        Room::roomVariant::ROOM)
                {
                    continue;
                }

                const std::vector<Plane *> ownerWalls =
                    p_existingOwner->getWalls();
                if (std::find(ownerWalls.begin(),
                              ownerWalls.end(),
                              p_importedWall) != ownerWalls.end())
                {
                    p_targetRoom = p_existingOwner;
                    break;
                }
            }

            if (p_separatingPassage != nullptr &&
                p_targetRoom == p_bestRetainedRoom)
            {
                ORB_SLAM3::Room *p_farSideRoom =
                    p_separatingPassage->getProspectiveRoom();

                if (p_farSideRoom == nullptr || p_farSideRoom->isBad() ||
                    p_farSideRoom == p_importedRoom)
                {
                    p_farSideRoom = nullptr;
                }

                p_targetRoom = p_farSideRoom;
            }

            wallTransfers.push_back(
                {p_importedWall, p_targetRoom, p_separatingPassage});
        }

        /* Commit the precomputed ownership transaction. Removing the duplicate
         * edge before adding its destination prevents transient double
         * ownership inside the authorized room fusion. */
        for (const WallTransfer &transfer : wallTransfers)
        {
            p_importedRoom->removeWall(transfer.p_wall);

            if (transfer.p_targetRoom == nullptr)
            {
                std::cout << "[SemanticMerge] Far-side Wall#"
                          << transfer.p_wall->getId() << " at Passage#"
                          << transfer.p_separatingPassage->getId()
                          << " has no prospective; left unbound." << std::endl;
                continue;
            }

            transfer.p_targetRoom->setWalls(transfer.p_wall);

            if (transfer.p_separatingPassage != nullptr &&
                transfer.p_targetRoom != p_bestRetainedRoom)
            {
                std::cout << "[SemanticMerge] Redirected far-side Wall#"
                          << transfer.p_wall->getId() << " to stable Room#"
                          << transfer.p_targetRoom->getId() << "." << std::endl;
            }
        }

        for (ORB_SLAM3::Passage *p_importedPassage :
             p_importedRoom->getPassages())
        {
            p_bestRetainedRoom->setDoorways(p_importedPassage);
        }

        const double retainedWeight = static_cast<double>(
            std::max<std::size_t>(retainedWalls.size(), 1U));

        const double importedWeight = static_cast<double>(
            std::max<std::size_t>(importedWalls.size(), 1U));

        const Eigen::Vector3d fusedCentroid_World_m =
            (retainedWeight * p_bestRetainedRoom->getCentroid() +
             importedWeight * importedCentroid_World_m) /
            (retainedWeight + importedWeight);

        p_bestRetainedRoom->setCentroid(fusedCentroid_World_m);

        if (p_bestRetainedRoom->getGroundPlane() == nullptr)
        {
            p_bestRetainedRoom->setGroundPlane(
                p_importedRoom->getGroundPlane());
        }

        if (!p_bestRetainedRoom->getHasKnownLabel() &&
            p_importedRoom->getHasKnownLabel())
        {
            p_bestRetainedRoom->setHasKnownLabel(true);
            p_bestRetainedRoom->setMetaMarker(p_importedRoom->getMetaMarker());
            p_bestRetainedRoom->setMetaMarkerId(
                p_importedRoom->getMetaMarkerId());
            p_bestRetainedRoom->setName(p_importedRoom->getName());
        }

        for (Floor *p_floor : p_map_inout->GetAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(p_importedRoom, p_bestRetainedRoom);
            }
        }

        p_map_inout->EraseDetectedMapRoom(p_importedRoom);
        p_map_inout->EraseMarkerBasedMapRoom(p_importedRoom);
        p_importedRoom->clearWalls();
        p_importedRoom->clearPassages();
        p_importedRoom->setBad();

        std::cout << "[SemanticMerge] Fused duplicate Room#"
                  << p_importedRoom->getId() << " into Room#"
                  << p_bestRetainedRoom->getId() << " (centroid distance "
                  << bestCentroidDistance_m << " m)." << std::endl;
    }
}

void Utils::reAssociatePassages(Atlas *p_atlas_inout)
{
    if (p_atlas_inout == nullptr)
    {
        return;
    }

    Map *p_activeMap = p_atlas_inout->GetCurrentMap();

    if (p_activeMap == nullptr)
    {
        return;
    }

    std::vector<Passage *> passages = p_activeMap->GetAllPassages();
    const std::vector<Room *> activeRooms = p_activeMap->GetAllRooms();
    const std::unordered_set<Room *> activeRoomSet(activeRooms.begin(),
                                                   activeRooms.end());

    const auto liveRoomHandle = [&activeRoomSet](Room *p_room) -> Room *
    {
        return p_room != nullptr && !p_room->isBad() &&
                       activeRoomSet.count(p_room) > 0U
                   ? p_room
                   : nullptr;
    };

    std::sort(passages.begin(),
              passages.end(),
              [](const Passage *p_firstPassage, const Passage *p_secondPassage)
              {
                  if (p_firstPassage == nullptr)
                  {
                      return false;
                  }

                  if (p_secondPassage == nullptr)
                  {
                      return true;
                  }

                  return p_firstPassage->getId() < p_secondPassage->getId();
              });

    const SystemParams::sem_seg::PassageDetection &passageParameters =
        SystemParams::GetParams()->sem_seg.passageDetection;

    /*
     * Use the same geometrically constrained identity gate as online passage
     * tracking. Coplanarity and normal checks below prevent this distance from
     * collapsing openings on unrelated walls after a map merge.
     */
    const double maximumCentroidDistance_m =
        static_cast<double>(passageParameters.duplicatePassageDistance_m);
    const double minimumNormalAlignment =
        static_cast<double>(passageParameters.duplicateNormalAlignment);
    constexpr double maximumSupportingPlaneSeparation_m = 0.30;

    std::unordered_set<Passage *> retiredPassages;

    for (std::size_t retainedIndex = 0U; retainedIndex < passages.size();
         retainedIndex++)
    {
        Passage *p_retainedPassage = passages[retainedIndex];

        if (p_retainedPassage == nullptr ||
            retiredPassages.count(p_retainedPassage) > 0U)
        {
            continue;
        }

        Eigen::Vector4d retainedEquation =
            p_retainedPassage->getGlobalEquation().coeffs();
        const double retainedNormalNorm = retainedEquation.head<3>().norm();

        if (!retainedEquation.allFinite() || retainedNormalNorm < 1e-8)
        {
            continue;
        }

        retainedEquation /= retainedNormalNorm;

        for (std::size_t candidateIndex = retainedIndex + 1U;
             candidateIndex < passages.size();
             candidateIndex++)
        {
            Passage *p_candidatePassage = passages[candidateIndex];

            if (p_candidatePassage == nullptr ||
                retiredPassages.count(p_candidatePassage) > 0U)
            {
                continue;
            }

            const Eigen::Vector3d retainedCentroid_World_m =
                p_retainedPassage->getCentroid();
            const Eigen::Vector3d candidateCentroid_World_m =
                p_candidatePassage->getCentroid();

            if (!retainedCentroid_World_m.allFinite() ||
                !candidateCentroid_World_m.allFinite() ||
                (candidateCentroid_World_m - retainedCentroid_World_m).norm() >
                    maximumCentroidDistance_m)
            {
                continue;
            }

            Eigen::Vector4d candidateEquation =
                p_candidatePassage->getGlobalEquation().coeffs();
            const double candidateNormalNorm =
                candidateEquation.head<3>().norm();

            if (!candidateEquation.allFinite() || candidateNormalNorm < 1e-8)
            {
                continue;
            }

            candidateEquation /= candidateNormalNorm;

            if (std::abs(retainedEquation.head<3>().dot(
                    candidateEquation.head<3>())) < minimumNormalAlignment)
            {
                continue;
            }

            const double retainedPlaneResidual_m = std::abs(
                retainedEquation.head<3>().dot(candidateCentroid_World_m) +
                retainedEquation(3));
            const double candidatePlaneResidual_m = std::abs(
                candidateEquation.head<3>().dot(retainedCentroid_World_m) +
                candidateEquation(3));

            if (retainedPlaneResidual_m > maximumSupportingPlaneSeparation_m ||
                candidatePlaneResidual_m > maximumSupportingPlaneSeparation_m)
            {
                continue;
            }

            Room *p_retainedHandle =
                liveRoomHandle(p_retainedPassage->getProspectiveRoom());
            Room *p_candidateHandle =
                liveRoomHandle(p_candidatePassage->getProspectiveRoom());

            p_retainedPassage->mergeKnownSideProvenance(
                p_candidatePassage->getKnownSideProvenance());
            const Passage::KnownSideProvenance knownSide =
                p_retainedPassage->getKnownSideProvenance();

            const auto isProvenFarSide =
                [&knownSide, &retainedCentroid_World_m](Room *p_room)
            {
                return p_room != nullptr && knownSide.hasDirection() &&
                       knownSide.direction_World.dot(
                           p_room->getCentroid() -
                           retainedCentroid_World_m) < -0.20;
            };

            Room *p_survivingHandle = p_retainedHandle;

            if (p_survivingHandle == nullptr)
            {
                p_survivingHandle = p_candidateHandle;
            }
            else if (p_candidateHandle != nullptr &&
                     p_candidateHandle != p_survivingHandle)
            {
                const bool retainedIsFarSide =
                    isProvenFarSide(p_retainedHandle);
                const bool candidateIsFarSide =
                    isProvenFarSide(p_candidateHandle);
                if (candidateIsFarSide && !retainedIsFarSide)
                {
                    p_survivingHandle = p_candidateHandle;
                }
            }

            p_retainedPassage->setProspectiveRoom(p_survivingHandle);
            if (p_survivingHandle != nullptr)
            {
                p_survivingHandle->setDoorways(p_retainedPassage);
            }

            Eigen::Vector3d fusedCentroid_World_m =
                0.5 * (retainedCentroid_World_m + candidateCentroid_World_m);
            const double fusedCentroidResidual_m =
                retainedEquation.head<3>().dot(fusedCentroid_World_m) +
                retainedEquation(3);
            fusedCentroid_World_m -=
                fusedCentroidResidual_m * retainedEquation.head<3>();

            p_retainedPassage->setCentroid(fusedCentroid_World_m);
            p_retainedPassage->setWidth(
                std::max(p_retainedPassage->getWidth(),
                         p_candidatePassage->getWidth()));
            p_retainedPassage->setHeight(
                std::max(p_retainedPassage->getHeight(),
                         p_candidatePassage->getHeight()));
            p_retainedPassage->setPassable(p_retainedPassage->isPassable() ||
                                           p_candidatePassage->isPassable());

            const std::size_t retainedTraversalCount =
                p_retainedPassage->getTraversalObservationCount();
            const std::size_t candidateTraversalCount =
                p_candidatePassage->getTraversalObservationCount();
            const std::size_t maximumTraversalCount =
                std::numeric_limits<std::size_t>::max();

            p_retainedPassage->setTraversalObservationCount(
                candidateTraversalCount >
                        maximumTraversalCount - retainedTraversalCount
                    ? maximumTraversalCount
                    : retainedTraversalCount + candidateTraversalCount);

            for (Plane *p_supportingWall :
                 p_candidatePassage->getAssociateWalls())
            {
                p_retainedPassage->addAssociateWall(p_supportingWall);
            }

            if (p_retainedPassage->getAssociateDoor() == nullptr &&
                p_candidatePassage->getAssociateDoor() != nullptr)
            {
                p_retainedPassage->setAssociateDoor(
                    p_candidatePassage->getAssociateDoor());
            }

            for (Room *p_room : p_activeMap->GetAllRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    p_room->replacePassageAssociation(p_candidatePassage,
                                                      p_retainedPassage);
                }
            }

            for (KeyFrame *p_keyFrame : p_activeMap->GetAllKeyFrames())
            {
                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    p_keyFrame->ReplaceMapPassage(p_candidatePassage,
                                                  p_retainedPassage);
                }
            }

            p_activeMap->EraseMapPassage(p_candidatePassage);
            p_candidatePassage->setProspectiveRoom(nullptr);
            p_candidatePassage->setMap(nullptr);
            retiredPassages.insert(p_candidatePassage);
        }
    }
}

void Utils::propagateSemanticPoseCorrections(
    Map                   *p_map_inout,
    const KeyFramePoseMap &keyFramePosesBefore_WorldToCamera_in,
    const KeyFramePoseMap &keyFramePosesAfter_WorldToCamera_in,
    const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in)
{
    if (p_map_inout == nullptr)
    {
        return;
    }

    struct PoseCorrectionNode
    {
        KeyFrame       *p_keyFrame;
        g2o::Sim3       poseBefore_WorldToCamera;
        g2o::Sim3       poseAfter_WorldToCamera;
        g2o::Sim3       correction_oldWorldToNewWorld;
        Eigen::Vector3d cameraCenter_OldWorld_m;
    };

    const auto isFiniteSim3 = [](const g2o::Sim3 &transform_in)
    {
        return std::isfinite(transform_in.scale()) &&
               std::abs(transform_in.scale()) > 1e-12 &&
               transform_in.translation().allFinite() &&
               transform_in.rotation().coeffs().allFinite();
    };

    std::vector<PoseCorrectionNode> correctionNodes;
    correctionNodes.reserve(keyFramePosesBefore_WorldToCamera_in.size());

    for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
         keyFramePosesBefore_WorldToCamera_in)
    {
        if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
            !isFiniteSim3(poseBefore_WorldToCamera))
        {
            continue;
        }

        const auto poseAfterIterator =
            keyFramePosesAfter_WorldToCamera_in.find(p_keyFrame);

        if (poseAfterIterator == keyFramePosesAfter_WorldToCamera_in.end() ||
            !isFiniteSim3(poseAfterIterator->second))
        {
            continue;
        }

        const g2o::Sim3 correction_oldWorldToNewWorld =
            poseAfterIterator->second.inverse() * poseBefore_WorldToCamera;

        const Eigen::Vector3d cameraCenter_OldWorld_m =
            poseBefore_WorldToCamera.inverse().map(Eigen::Vector3d::Zero());

        if (!isFiniteSim3(correction_oldWorldToNewWorld) ||
            !cameraCenter_OldWorld_m.allFinite())
        {
            continue;
        }

        correctionNodes.push_back({p_keyFrame,
                                   poseBefore_WorldToCamera,
                                   poseAfterIterator->second,
                                   correction_oldWorldToNewWorld,
                                   cameraCenter_OldWorld_m});
    }

    std::sort(correctionNodes.begin(),
              correctionNodes.end(),
              [](const PoseCorrectionNode &firstNode_in,
                 const PoseCorrectionNode &secondNode_in) {
                  return firstNode_in.p_keyFrame->mnId <
                         secondNode_in.p_keyFrame->mnId;
              });

    const auto findNodeForKeyFrame =
        [&correctionNodes](
            const KeyFrame *p_keyFrame_in) -> const PoseCorrectionNode *
    {
        if (p_keyFrame_in == nullptr)
        {
            return nullptr;
        }

        const auto nodeIterator =
            std::find_if(correctionNodes.begin(),
                         correctionNodes.end(),
                         [p_keyFrame_in](const PoseCorrectionNode &node_in)
                         { return node_in.p_keyFrame == p_keyFrame_in; });

        return nodeIterator == correctionNodes.end() ? nullptr
                                                     : &(*nodeIterator);
    };

    const auto findNearestNode =
        [&correctionNodes](const Eigen::Vector3d &point_OldWorld_m)
        -> const PoseCorrectionNode *
    {
        if (!point_OldWorld_m.allFinite())
        {
            return nullptr;
        }

        const PoseCorrectionNode *p_nearestNode = nullptr;
        double                    nearestSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        for (const PoseCorrectionNode &node : correctionNodes)
        {
            const double squaredDistance_m2 =
                (node.cameraCenter_OldWorld_m - point_OldWorld_m).squaredNorm();

            if (squaredDistance_m2 < nearestSquaredDistance_m2)
            {
                nearestSquaredDistance_m2 = squaredDistance_m2;
                p_nearestNode             = &node;
            }
        }

        return p_nearestNode;
    };

    const auto selectCorrectionForPoint =
        [&findNearestNode, &fallbackTransform_oldWorldToNewWorld_in](
            const Eigen::Vector3d &point_OldWorld_m) -> const g2o::Sim3 &
    {
        const PoseCorrectionNode *p_nearestNode =
            findNearestNode(point_OldWorld_m);

        return p_nearestNode != nullptr
                   ? p_nearestNode->correction_oldWorldToNewWorld
                   : fallbackTransform_oldWorldToNewWorld_in;
    };

    std::map<Plane *, g2o::Sim3>       planeCorrections_oldWorldToNewWorld;
    std::map<Plane *, Eigen::Vector3d> planeCentroids_OldWorld_m;

    for (Plane *p_plane : p_map_inout->GetAllPlanes())
    {
        if (p_plane == nullptr || p_plane->isBad())
        {
            continue;
        }

        const Eigen::Vector3d planeCentroid_OldWorld_m = p_plane->getCentroid();

        planeCentroids_OldWorld_m.insert_or_assign(p_plane,
                                                   planeCentroid_OldWorld_m);

        const PoseCorrectionNode *p_selectedNode =
            findNodeForKeyFrame(p_plane->refKeyFrame);

        if (p_selectedNode == nullptr)
        {
            double nearestObserverSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (const auto &[p_observingKeyFrame, observation] :
                 p_plane->getObservations())
            {
                (void)observation;

                const PoseCorrectionNode *p_observerNode =
                    findNodeForKeyFrame(p_observingKeyFrame);

                if (p_observerNode == nullptr)
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (p_observerNode->cameraCenter_OldWorld_m -
                     planeCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < nearestObserverSquaredDistance_m2)
                {
                    nearestObserverSquaredDistance_m2 = squaredDistance_m2;
                    p_selectedNode                    = p_observerNode;
                }
            }
        }

        const g2o::Sim3 &correction_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->correction_oldWorldToNewWorld
                : selectCorrectionForPoint(planeCentroid_OldWorld_m);

        p_plane->applyTransform(correction_oldWorldToNewWorld);
        planeCorrections_oldWorldToNewWorld.insert_or_assign(
            p_plane,
            correction_oldWorldToNewWorld);
    }

    std::map<Marker *, g2o::Sim3> markerCorrections_oldWorldToNewWorld;

    for (Marker *p_marker : p_map_inout->GetAllMarkers())
    {
        if (p_marker == nullptr)
        {
            continue;
        }

        const Sophus::SE3f markerPose_MarkerToOldWorld =
            p_marker->getGlobalPose();

        const PoseCorrectionNode *p_selectedNode = nullptr;
        double                    smallestReconstructionError_m2 =
            std::numeric_limits<double>::infinity();

        for (const auto &[p_observingKeyFrame, markerPose_MarkerToCamera] :
             p_marker->getObservations())
        {
            const PoseCorrectionNode *p_observerNode =
                findNodeForKeyFrame(p_observingKeyFrame);

            if (p_observerNode == nullptr)
            {
                continue;
            }

            const Eigen::Vector3d reconstructedPosition_OldWorld_m =
                p_observerNode->poseBefore_WorldToCamera.inverse().map(
                    markerPose_MarkerToCamera.translation().cast<double>());

            const double reconstructionError_m2 =
                (reconstructedPosition_OldWorld_m -
                 markerPose_MarkerToOldWorld.translation().cast<double>())
                    .squaredNorm();

            if (reconstructionError_m2 < smallestReconstructionError_m2)
            {
                smallestReconstructionError_m2 = reconstructionError_m2;
                p_selectedNode                 = p_observerNode;
            }
        }

        const g2o::Sim3 &markerCorrection_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->correction_oldWorldToNewWorld
                : selectCorrectionForPoint(
                      markerPose_MarkerToOldWorld.translation().cast<double>());

        /*
         * Preserve the fused global marker estimate. Reconstructing it from a
         * single noisy observation would move the marker even for an identity
         * pose correction, and repeated GBA/remerge cycles would accumulate
         * that observation noise.
         */
        p_marker->applyTransform(markerCorrection_oldWorldToNewWorld);
        markerCorrections_oldWorldToNewWorld.insert_or_assign(
            p_marker,
            markerCorrection_oldWorldToNewWorld);
    }

    for (ORB_SLAM3::Passage *p_passage : p_map_inout->GetAllPassages())
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        const Eigen::Vector3d passageCentroid_OldWorld_m =
            p_passage->getCentroid();

        const g2o::Sim3 *p_passageCorrection = nullptr;

        if (Plane *p_doorPlane = p_passage->getAssociateDoor();
            p_doorPlane != nullptr)
        {
            const auto correctionIterator =
                planeCorrections_oldWorldToNewWorld.find(p_doorPlane);

            if (correctionIterator != planeCorrections_oldWorldToNewWorld.end())
            {
                p_passageCorrection = &correctionIterator->second;
            }
        }

        if (p_passageCorrection == nullptr)
        {
            double closestWallSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (Plane *p_wall : p_passage->getAssociateWalls())
            {
                const auto correctionIterator =
                    planeCorrections_oldWorldToNewWorld.find(p_wall);

                if (p_wall == nullptr ||
                    correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end())
                {
                    continue;
                }

                const auto centroidIterator =
                    planeCentroids_OldWorld_m.find(p_wall);

                if (centroidIterator == planeCentroids_OldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - passageCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_passageCorrection           = &correctionIterator->second;
                }
            }
        }

        p_passage->applyTransform(
            p_passageCorrection != nullptr
                ? *p_passageCorrection
                : selectCorrectionForPoint(passageCentroid_OldWorld_m));
    }

    std::map<Room *, g2o::Sim3>       roomCorrections_oldWorldToNewWorld;
    std::map<Room *, Eigen::Vector3d> roomCentroids_OldWorld_m;
    std::set<Room *>                  correctedRooms;
    std::vector<Room *>       rooms = p_map_inout->GetAllDetectedMapRooms();
    const std::vector<Room *> markerRooms =
        p_map_inout->GetAllMarkerBasedMapRooms();
    rooms.insert(rooms.end(), markerRooms.begin(), markerRooms.end());

    for (Room *p_room : rooms)
    {
        if (p_room == nullptr || p_room->isBad() ||
            !correctedRooms.insert(p_room).second)
        {
            continue;
        }

        const Eigen::Vector3d roomCentroid_OldWorld_m = p_room->getCentroid();
        roomCentroids_OldWorld_m.insert_or_assign(p_room,
                                                  roomCentroid_OldWorld_m);

        const g2o::Sim3 *p_roomCorrection = nullptr;
        Marker          *p_metaMarker     = p_room->getMetaMarker();

        if (p_metaMarker != nullptr)
        {
            const auto markerCorrectionIterator =
                markerCorrections_oldWorldToNewWorld.find(p_metaMarker);

            if (markerCorrectionIterator !=
                markerCorrections_oldWorldToNewWorld.end())
            {
                p_roomCorrection = &markerCorrectionIterator->second;
            }
        }

        if (p_roomCorrection == nullptr)
        {
            double closestWallSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            for (Plane *p_wall : p_room->getWalls())
            {
                const auto correctionIterator =
                    planeCorrections_oldWorldToNewWorld.find(p_wall);
                const auto centroidIterator =
                    planeCentroids_OldWorld_m.find(p_wall);

                if (correctionIterator ==
                        planeCorrections_oldWorldToNewWorld.end() ||
                    centroidIterator == planeCentroids_OldWorld_m.end())
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (centroidIterator->second - roomCentroid_OldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < closestWallSquaredDistance_m2)
                {
                    closestWallSquaredDistance_m2 = squaredDistance_m2;
                    p_roomCorrection              = &correctionIterator->second;
                }
            }
        }

        const g2o::Sim3 &roomCorrection_oldWorldToNewWorld =
            p_roomCorrection != nullptr
                ? *p_roomCorrection
                : selectCorrectionForPoint(roomCentroid_OldWorld_m);

        p_room->applyTransform(roomCorrection_oldWorldToNewWorld);
        roomCorrections_oldWorldToNewWorld.insert_or_assign(
            p_room,
            roomCorrection_oldWorldToNewWorld);
    }

    for (Floor *p_floor : p_map_inout->GetAllFloors())
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        const Eigen::Vector3d floorCentroid_OldWorld_m = p_floor->getCentroid();

        const g2o::Sim3 *p_floorCorrection = nullptr;
        double           closestRoomSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        for (Room *p_room : p_floor->getRooms())
        {
            if (p_room == nullptr || p_room->isBad() ||
                p_room->getMap() != p_map_inout)
            {
                continue;
            }

            const auto correctionIterator =
                roomCorrections_oldWorldToNewWorld.find(p_room);
            const auto centroidIterator = roomCentroids_OldWorld_m.find(p_room);

            if (correctionIterator ==
                    roomCorrections_oldWorldToNewWorld.end() ||
                centroidIterator == roomCentroids_OldWorld_m.end())
            {
                continue;
            }

            const double squaredDistance_m2 =
                (centroidIterator->second - floorCentroid_OldWorld_m)
                    .squaredNorm();

            if (squaredDistance_m2 < closestRoomSquaredDistance_m2)
            {
                closestRoomSquaredDistance_m2 = squaredDistance_m2;
                p_floorCorrection             = &correctionIterator->second;
            }
        }

        p_floor->applyTransform(
            p_floorCorrection != nullptr
                ? *p_floorCorrection
                : selectCorrectionForPoint(floorCentroid_OldWorld_m));
    }

    auto skeletonClusters_OldWorld_m = p_map_inout->GetSkeletonClusterPoints();

    for (std::vector<Eigen::Vector3d> &cluster_OldWorld_m :
         skeletonClusters_OldWorld_m)
    {
        if (cluster_OldWorld_m.empty())
        {
            continue;
        }

        Eigen::Vector3d clusterCentroid_OldWorld_m = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d &point_OldWorld_m : cluster_OldWorld_m)
        {
            clusterCentroid_OldWorld_m += point_OldWorld_m;
        }
        clusterCentroid_OldWorld_m /=
            static_cast<double>(cluster_OldWorld_m.size());

        const g2o::Sim3 &clusterCorrection_oldWorldToNewWorld =
            selectCorrectionForPoint(clusterCentroid_OldWorld_m);

        for (Eigen::Vector3d &point_OldWorld_m : cluster_OldWorld_m)
        {
            point_OldWorld_m =
                clusterCorrection_oldWorldToNewWorld.map(point_OldWorld_m);
        }
    }

    p_map_inout->SetSkeletonClusterPoints(skeletonClusters_OldWorld_m);

    auto skeletonEdges_OldWorld_m = p_map_inout->GetSkeletonEdges();

    for (auto &edge_OldWorld_m : skeletonEdges_OldWorld_m)
    {
        const Eigen::Vector3d firstEndpoint_OldWorld_m = edge_OldWorld_m.first;
        const Eigen::Vector3d secondEndpoint_OldWorld_m =
            edge_OldWorld_m.second;

        edge_OldWorld_m.first =
            selectCorrectionForPoint(firstEndpoint_OldWorld_m)
                .map(firstEndpoint_OldWorld_m);
        edge_OldWorld_m.second =
            selectCorrectionForPoint(secondEndpoint_OldWorld_m)
                .map(secondEndpoint_OldWorld_m);
    }

    p_map_inout->SetSkeletonEdges(skeletonEdges_OldWorld_m);
}

void Utils::consolidateProvisionalRooms(ORB_SLAM3::Room *selectedRoom,
                                        Atlas           *mpAtlas)
{
    /* Confirm the selected room is valid */
    if (selectedRoom == nullptr || selectedRoom->isBad())
    {
        return;
    }

    /* Extract all walls assigned to the cluster-backed room */
    const std::vector<ORB_SLAM3::Plane *> selectedWalls =
        selectedRoom->getWalls();

    /* Create a set containing the selected wall IDs */
    std::unordered_set<int> selectedWallIds;
    selectedWallIds.reserve(selectedWalls.size());

    /* Insert every valid selected wall ID into the set */
    for (ORB_SLAM3::Plane *wall : selectedWalls)
    {
        if (wall != nullptr && !wall->isBad())
        {
            selectedWallIds.insert(wall->getId());
        }
    }

    /* A room without walls cannot absorb another structural element */
    if (selectedWallIds.empty())
    {
        return;
    }

    /* Extract all rooms and provisional structural elements */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Iterate through every possible redundant structural element */
    for (ORB_SLAM3::Room *candidateRoom : allRooms)
    {
        /* Skip invalid rooms and the selected room itself */
        if (candidateRoom == nullptr || candidateRoom == selectedRoom ||
            candidateRoom->isBad())
        {
            continue;
        }

        const std::vector<ORB_SLAM3::Passage *> activePassages =
            mpAtlas->GetAllPassages();
        const bool candidateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [candidateRoom](ORB_SLAM3::Passage *p_passage)
            {
                return p_passage != nullptr &&
                       p_passage->getProspectiveRoom() == candidateRoom;
            });

        if (candidateIsLiveProspective)
        {
            continue;
        }

        /*!
         * Only automatically absorb undefined structural elements.
         * Classified rooms and corridors must never be merged automatically.
         */
        if (candidateRoom->getRoomVariant() !=
            ORB_SLAM3::Room::roomVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the candidate room walls */
        const std::vector<ORB_SLAM3::Plane *> candidateWalls =
            candidateRoom->getWalls();

        /*!
         * Orphan-wall fallback creates one provisional SE per wall. Restrict
         * automatic consolidation to those single-wall provisional elements.
         * This prevents an emerging room on the opposite side of a shared wall
         * from being removed before it has enough evidence for classification.
         */
        std::vector<ORB_SLAM3::Plane *> validCandidateWalls;
        validCandidateWalls.reserve(candidateWalls.size());

        for (ORB_SLAM3::Plane *candidateWall : candidateWalls)
        {
            if (candidateWall != nullptr && !candidateWall->isBad())
            {
                validCandidateWalls.push_back(candidateWall);
            }
        }

        /* Only single-wall provisional elements are safe to absorb */
        if (validCandidateWalls.size() != 1)
        {
            continue;
        }

        /* Extract the single wall represented by the provisional element */
        ORB_SLAM3::Plane *candidateWall = validCandidateWalls.front();

        /* The selected room must already contain the candidate wall */
        if (selectedWallIds.count(candidateWall->getId()) == 0)
        {
            continue;
        }

        /*!
         * Confirm the candidate centroid is still close to its wall plane.
         * This identifies a wall-centred orphan SE rather than a free-space
         * cluster which may represent a genuine room on the opposite side.
         */
        Eigen::Vector4d wallEquation =
            candidateWall->getGlobalEquation().coeffs();

        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= normalNorm;

        const double candidatePlaneDistance =
            std::abs(wallEquation.head<3>().dot(candidateRoom->getCentroid()) +
                     wallEquation(3));

        constexpr double provisionalWallDistanceThreshold = 0.25;

        if (candidatePlaneDistance > provisionalWallDistanceThreshold)
        {
            continue;
        }

        /* Preserve passage relationships before invalidating the candidate */
        const std::vector<ORB_SLAM3::Passage *> candidatePassages =
            candidateRoom->getPassages();

        for (ORB_SLAM3::Passage *passage : candidatePassages)
        {
            /* Skip invalid passages */
            if (passage == nullptr)
            {
                continue;
            }

            /* Check whether the selected room already contains the passage */
            const std::vector<ORB_SLAM3::Passage *> selectedPassages =
                selectedRoom->getPassages();

            const bool alreadyPresent = std::any_of(
                selectedPassages.begin(),
                selectedPassages.end(),
                [passage](ORB_SLAM3::Passage *existingPassage)
                {
                    return existingPassage != nullptr &&
                           existingPassage->getId() == passage->getId();
                });

            /* Copy the passage relationship if required */
            if (!alreadyPresent)
            {
                selectedRoom->setDoorways(passage);
            }
        }

        /* Preserve floor membership before retiring the provisional room. */
        for (ORB_SLAM3::Floor *p_floor : mpAtlas->GetAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(candidateRoom, selectedRoom);
            }
        }

        /* Mark the redundant provisional structural element as invalid */
        candidateRoom->setBad();
    }
}

Eigen::Isometry3d Utils::computeMapTransform_Horn(
    const std::vector<Eigen::Vector3d> &normalsA_in,
    const std::vector<Eigen::Vector3d> &centroidsA_in,
    const std::vector<Eigen::Vector3d> &normalsB_in,
    const std::vector<Eigen::Vector3d> &centroidsB_in)
{
    if (normalsA_in.size() != normalsB_in.size() || normalsA_in.size() < 3 ||
        centroidsA_in.size() != normalsA_in.size() ||
        centroidsB_in.size() != normalsB_in.size())
    {
        std::cout << "[MapMerge] computeMapTransform_Horn: insufficient or "
                     "mismatched correspondences."
                  << std::endl;
        return Eigen::Isometry3d::Identity();
    }

    Eigen::Vector3d centroidA = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroidB = Eigen::Vector3d::Zero();

    for (std::size_t index = 0; index < normalsA_in.size(); ++index)
    {
        if (!normalsA_in[index].allFinite() ||
            !normalsB_in[index].allFinite() ||
            !centroidsA_in[index].allFinite() ||
            !centroidsB_in[index].allFinite() ||
            normalsA_in[index].squaredNorm() < 1e-12 ||
            normalsB_in[index].squaredNorm() < 1e-12)
        {
            std::cout << "[MapMerge] computeMapTransform_Horn: invalid "
                         "correspondence data."
                      << std::endl;
            return Eigen::Isometry3d::Identity();
        }

        centroidA += centroidsA_in[index];
        centroidB += centroidsB_in[index];
    }

    centroidA /= static_cast<double>(normalsA_in.size());
    centroidB /= static_cast<double>(normalsB_in.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (std::size_t index = 0; index < normalsA_in.size(); ++index)
    {
        const Eigen::Vector3d normalA = normalsA_in[index].normalized();
        const Eigen::Vector3d normalB = normalsB_in[index].normalized();
        covariance += normalA * normalB.transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                                Eigen::ComputeFullU |
                                                    Eigen::ComputeFullV);

    Eigen::Matrix3d signCorrection = Eigen::Matrix3d::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0)
    {
        signCorrection(2, 2) = -1.0;
    }

    Eigen::Isometry3d transformBFromA = Eigen::Isometry3d::Identity();
    transformBFromA.linear() =
        svd.matrixV() * signCorrection * svd.matrixU().transpose();
    transformBFromA.translation() =
        centroidB - transformBFromA.linear() * centroidA;

    return transformBFromA;
}

std::size_t
    Utils::matchWallsBetweenRooms(const Room                   *p_roomA_in,
                                  const Room                   *p_roomB_in,
                                  std::vector<Eigen::Vector3d> &normalsA_out,
                                  std::vector<Eigen::Vector3d> &centroidsA_out,
                                  std::vector<Eigen::Vector3d> &normalsB_out,
                                  std::vector<Eigen::Vector3d> &centroidsB_out)
{
    if (p_roomA_in == nullptr || p_roomB_in == nullptr)
    {
        return 0;
    }

    constexpr double kWallCorrespondenceCosTheta = 0.85;

    const auto collectValidWalls = [](const Room *p_room_in)
        -> std::vector<std::pair<Plane *, Eigen::Vector3d>>
    {
        std::vector<std::pair<Plane *, Eigen::Vector3d>> validWalls;

        for (Plane *p_wall : p_room_in->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const std::optional<Eigen::Vector3d> normal_World =
                p_room_in->getWallNormalTowardRoom_World(p_wall);

            if (!normal_World.has_value())
            {
                continue;
            }

            validWalls.emplace_back(p_wall, normal_World.value());
        }

        std::sort(validWalls.begin(),
                  validWalls.end(),
                  [](const std::pair<Plane *, Eigen::Vector3d> &first_in,
                     const std::pair<Plane *, Eigen::Vector3d> &second_in) {
                      return first_in.first->getId() < second_in.first->getId();
                  });

        return validWalls;
    };

    const std::vector<std::pair<Plane *, Eigen::Vector3d>> validWallsA =
        collectValidWalls(p_roomA_in);
    const std::vector<std::pair<Plane *, Eigen::Vector3d>> validWallsB =
        collectValidWalls(p_roomB_in);

    std::vector<bool> matchedA(validWallsA.size(), false);
    std::vector<bool> matchedB(validWallsB.size(), false);

    std::size_t acceptedPairCount = 0;

    while (true)
    {
        double      bestNormalAlignment = -1.0;
        std::size_t bestIndexA          = 0;
        std::size_t bestIndexB          = 0;

        for (std::size_t indexA = 0; indexA < validWallsA.size(); ++indexA)
        {
            if (matchedA[indexA])
            {
                continue;
            }

            for (std::size_t indexB = 0; indexB < validWallsB.size(); ++indexB)
            {
                if (matchedB[indexB])
                {
                    continue;
                }

                const double normalAlignment =
                    validWallsA[indexA].second.dot(validWallsB[indexB].second);

                if (normalAlignment > bestNormalAlignment)
                {
                    bestNormalAlignment = normalAlignment;
                    bestIndexA          = indexA;
                    bestIndexB          = indexB;
                }
            }
        }

        if (bestNormalAlignment <= kWallCorrespondenceCosTheta)
        {
            break;
        }

        matchedA[bestIndexA] = true;
        matchedB[bestIndexB] = true;

        normalsA_out.push_back(validWallsA[bestIndexA].second);
        centroidsA_out.push_back(validWallsA[bestIndexA].first->getCentroid());
        normalsB_out.push_back(validWallsB[bestIndexB].second);
        centroidsB_out.push_back(validWallsB[bestIndexB].first->getCentroid());

        acceptedPairCount++;
    }

    return acceptedPairCount;
}

bool Utils::collectCorrespondingWalls(
    Map                          *p_mapA_in,
    Map                          *p_mapB_in,
    std::vector<Eigen::Vector3d> &normalsA_out,
    std::vector<Eigen::Vector3d> &centroidsA_out,
    std::vector<Eigen::Vector3d> &normalsB_out,
    std::vector<Eigen::Vector3d> &centroidsB_out)
{
    if (p_mapA_in == nullptr || p_mapB_in == nullptr)
    {
        return false;
    }

    std::vector<Room *> roomsA = p_mapA_in->GetAllRooms();
    std::vector<Room *> roomsB = p_mapB_in->GetAllRooms();

    std::sort(roomsA.begin(),
              roomsA.end(),
              [](const Room *p_first, const Room *p_second)
              { return p_first->getId() < p_second->getId(); });

    std::sort(roomsB.begin(),
              roomsB.end(),
              [](const Room *p_first, const Room *p_second)
              { return p_first->getId() < p_second->getId(); });

    for (Room *p_roomB : roomsB)
    {
        if (p_roomB == nullptr || p_roomB->isBad() || !p_roomB->hasRoomTag())
        {
            continue;
        }

        for (Room *p_roomA : roomsA)
        {
            if (p_roomA == nullptr || p_roomA->isBad())
            {
                continue;
            }

            if (p_roomA->getRoomTag() != p_roomB->getRoomTag())
            {
                continue;
            }

            matchWallsBetweenRooms(p_roomA,
                                   p_roomB,
                                   normalsA_out,
                                   centroidsA_out,
                                   normalsB_out,
                                   centroidsB_out);
            break;
        }
    }

    return normalsA_out.size() >= 3 &&
           normalsA_out.size() == normalsB_out.size();
}

double Utils::calcSoftMin(vector<double> &values)
{
    // parameter controlling the softness/sharpness of the soft-min
    // the smaller the value, the more conservative the soft-min
    const double tau = 0.1;

    // soft-min = sum(exp(-value/tau) * value) / sum(exp(-value/tau))
    Eigen::Map<Eigen::VectorXd> confs(values.data(), values.size());
    Eigen::VectorXd             term = ((1.0 - confs.array()) / tau).exp();
    return ((term / term.sum()).array() * confs.array()).sum();
}
} // namespace ORB_SLAM3
