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

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::alignGeometryToEquation(
    const g2o::Plane3D &targetEquation_NewWorld_in)
{
    std::scoped_lock lock(positionMutex, typeMutex, featuresMutex);

    Eigen::Vector4d currentCoefficients = globalEquation.coeffs();
    Eigen::Vector4d targetCoefficients  = targetEquation_NewWorld_in.coeffs();

    const double currentNormalNorm = currentCoefficients.head<3>().norm();
    const double targetNormalNorm  = targetCoefficients.head<3>().norm();

    if (!currentCoefficients.allFinite() || !targetCoefficients.allFinite() ||
        currentNormalNorm < 1e-12 || targetNormalNorm < 1e-12)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    currentCoefficients /= currentNormalNorm;
    targetCoefficients /= targetNormalNorm;

    /* Plane equations are sign-equivalent; use the smaller rotation. */
    if (currentCoefficients.head<3>().dot(targetCoefficients.head<3>()) < 0.0)
    {
        targetCoefficients = -targetCoefficients;
    }

    const Eigen::Vector3d currentNormal = currentCoefficients.head<3>();
    const Eigen::Vector3d targetNormal  = targetCoefficients.head<3>();

    const double normalDotProduct =
        std::clamp(currentNormal.dot(targetNormal), -1.0, 1.0);

    Eigen::Quaterniond rotation_oldPlaneToOptimizedPlane;

    if (normalDotProduct > 1.0 - 1e-12)
    {
        rotation_oldPlaneToOptimizedPlane = Eigen::Quaterniond::Identity();
    }
    else if (normalDotProduct < -1.0 + 1e-12)
    {
        const Eigen::Vector3d referenceAxis = std::abs(currentNormal.x()) < 0.9
                                                  ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d rotationAxis =
            currentNormal.cross(referenceAxis).normalized();

        rotation_oldPlaneToOptimizedPlane = Eigen::Quaterniond(
            Eigen::AngleAxisd(std::acos(-1.0), rotationAxis));
    }
    else
    {
        const Eigen::Vector3d rotationAxis = currentNormal.cross(targetNormal);

        rotation_oldPlaneToOptimizedPlane =
            Eigen::Quaterniond(1.0 + normalDotProduct,
                               rotationAxis.x(),
                               rotationAxis.y(),
                               rotationAxis.z())
                .normalized();
    }

    if (!rotation_oldPlaneToOptimizedPlane.coeffs().allFinite())
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /*!
     * Rotate about a point on the current plane, then translate only along
     * the optimized normal until the transformed support lies on the target
     * equation. This avoids an arbitrary rotation about the world origin.
     */
    const double centroidSignedDistance_m =
        currentNormal.dot(centroid) + currentCoefficients(3);

    const Eigen::Vector3d anchor_OldWorld_m =
        centroid - centroidSignedDistance_m * currentNormal;

    const Eigen::Matrix3d rotationMatrix =
        rotation_oldPlaneToOptimizedPlane.toRotationMatrix();

    const Eigen::Vector3d rotationTranslation_m =
        anchor_OldWorld_m - rotationMatrix * anchor_OldWorld_m;

    const double distanceAfterRotation_m =
        currentCoefficients(3) - targetNormal.dot(rotationTranslation_m);

    const Eigen::Vector3d normalTranslation_m =
        (distanceAfterRotation_m - targetCoefficients(3)) * targetNormal;

    const Eigen::Vector3d translation_oldPlaneToOptimizedPlane_m =
        rotationTranslation_m + normalTranslation_m;

    centroid =
        rotationMatrix * centroid + translation_oldPlaneToOptimizedPlane_m;

    for (pcl::PointXYZRGBA &point : planeCloud->points)
    {
        Eigen::Vector3d point_NewWorld_m(point.x, point.y, point.z);
        point_NewWorld_m = rotationMatrix * point_NewWorld_m +
                           translation_oldPlaneToOptimizedPlane_m;

        point.x = static_cast<float>(point_NewWorld_m.x());
        point.y = static_cast<float>(point_NewWorld_m.y());
        point.z = static_cast<float>(point_NewWorld_m.z());
    }

    globalEquation = g2o::Plane3D(targetCoefficients);

    p_octree->deleteTree();
    p_octree->setInputCloud(planeCloud);
    p_octree->addPointsFromInputCloud();
    if (updatePlaneBoundsWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updatePlaneBoundsWithoutLock returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    ++cloudGeneration;
    successfulRefitGeneration = cloudGeneration;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
