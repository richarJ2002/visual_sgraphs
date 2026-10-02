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
 * @file            refitMappedPlaneFromCloud.cc
 *
 * @brief           Implements GeoSemHelpers::refitMappedPlaneFromCloud(),
 *                  declared in GeoSemHelpers.h.
 */

#include "GeoSemHelpers.h"
#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <Eigen/Eigenvalues>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::refitMappedPlaneFromCloud(
    vs_graphs::core::geometric::Plane *p_plane_inout,
    bool                              &wasPlaneRefit_out)
{
    /* Confirm the mapped plane is valid */
    bool planeIsBad{};
    if (!(p_plane_inout == nullptr) &&
        p_plane_inout->isBad(planeIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_plane_inout == nullptr || planeIsBad)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Claim one immutable generation; fitting never observes concurrent growth.
     */
    std::optional<geometric::Plane::GeometrySnapshot> geometrySnapshot{};
    if (p_plane_inout->beginMapCloudRefit(geometrySnapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: beginMapCloudRefit returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Require sufficient points for a stable covariance estimate */
    if (!geometrySnapshot.has_value() ||
        geometrySnapshot->supportCloud == nullptr ||
        geometrySnapshot->supportCloud->size() < 20)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud =
        geometrySnapshot->supportCloud;

    /* Calculate the centroid from all valid cloud points */
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    /* Init a counter to count the number of valid point clouds */
    std::size_t validPointCount = 0;

    for (const pcl::PointXYZRGBA &point : p_cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        centroid += Eigen::Vector3d(static_cast<double>(point.x),
                                    static_cast<double>(point.y),
                                    static_cast<double>(point.z));

        validPointCount++;
    }

    /* Return when too few valid points remain */
    if (validPointCount < 20)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    centroid /= static_cast<double>(validPointCount);

    /* Calculate the covariance matrix of the mapped plane cloud */
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (const pcl::PointXYZRGBA &point : p_cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d pointVector(static_cast<double>(point.x),
                                          static_cast<double>(point.y),
                                          static_cast<double>(point.z));

        const Eigen::Vector3d difference = pointVector - centroid;

        covariance += difference * difference.transpose();
    }

    covariance /= static_cast<double>(validPointCount);

    /*!
     * The eigenvector belonging to the smallest eigenvalue is the normal of
     * the best-fitting plane.
     */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        covariance);

    if (eigenSolver.info() != Eigen::Success)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    Eigen::Vector3d fittedNormal = eigenSolver.eigenvectors().col(0);

    if (!fittedNormal.allFinite() || fittedNormal.norm() < 1e-8)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    fittedNormal.normalize();

    /*!
     * Preserve the previous normal direction to prevent the plane equation
     * from changing sign between updates.
     */
    Eigen::Vector4d previousEquation = geometrySnapshot->planeEquation_world;

    const double previousNormalNorm = previousEquation.head<3>().norm();

    if (std::isfinite(previousNormalNorm) && previousNormalNorm > 1e-8)
    {
        const Eigen::Vector3d previousNormal =
            previousEquation.head<3>() / previousNormalNorm;

        if (fittedNormal.dot(previousNormal) < 0.0)
        {
            fittedNormal *= -1.0;
        }
    }

    /* Construct the fitted global plane equation */
    Eigen::Vector4d fittedEquation;

    fittedEquation.head<3>() = fittedNormal;

    fittedEquation(3) = -fittedNormal.dot(centroid);

    /* Publish the complete fitted geometry and recompute finite bounds once. */
    bool planeWasRefitPublished{};
    if (p_plane_inout->completeMapCloudRefit(geometrySnapshot->cloudGeneration,
                                             centroid,
                                             g2o::Plane3D(fittedEquation),
                                             validPointCount,
                                             planeWasRefitPublished) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: completeMapCloudRefit returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    wasPlaneRefit_out = planeWasRefitPublished;
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
