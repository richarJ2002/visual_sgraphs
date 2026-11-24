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
 * @file            fitEquationToCloud.cc
 *
 * @brief           Implements Plane::fitEquationToCloud(), declared in
 *                  mapping/geometric/Plane/objects/PlaneClass.h.
 */

#include "mapping/geometric/Plane/objects/PlaneClass.h"

#include <cmath>
#include <cstddef>

#include <Eigen/Eigenvalues>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::fitEquationToCloud(
    const pcl::PointCloud<pcl::PointXYZRGBA> &supportCloud_world_in,
    const Eigen::Vector4d                    &previousEquation_world_in,
    Eigen::Vector3d                          &planeCentroid_world_m_out,
    Eigen::Vector4d                          &planeEquation_world_out,
    std::size_t                              &finitePointCount_out,
    bool                                     &wasFitFound_out)
{
    /* Require sufficient points for a stable covariance estimate */
    constexpr std::size_t MINIMUM_FIT_POINT_COUNT = 20U;
    wasFitFound_out                               = false;

    /* Calculate the centroid from all valid cloud points */
    Eigen::Vector3d fittedCentroid_world_m = Eigen::Vector3d::Zero();
    std::size_t     validPointCount        = 0;
    for (const pcl::PointXYZRGBA &point : supportCloud_world_in.points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        fittedCentroid_world_m += Eigen::Vector3d(static_cast<double>(point.x),
                                                  static_cast<double>(point.y),
                                                  static_cast<double>(point.z));
        validPointCount++;
    }

    if (validPointCount < MINIMUM_FIT_POINT_COUNT)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    fittedCentroid_world_m /= static_cast<double>(validPointCount);

    /* Calculate the covariance matrix of the cloud about its centroid */
    Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
    for (const pcl::PointXYZRGBA &point : supportCloud_world_in.points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d pointOffset_world_m =
            Eigen::Vector3d(static_cast<double>(point.x),
                            static_cast<double>(point.y),
                            static_cast<double>(point.z)) -
            fittedCentroid_world_m;

        covarianceMatrix +=
            pointOffset_world_m * pointOffset_world_m.transpose();
    }

    covarianceMatrix /= static_cast<double>(validPointCount);

    /*!
     * The eigenvector belonging to the smallest eigenvalue is the normal of
     * the best-fitting plane.
     */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        covarianceMatrix);

    if (eigenSolver.info() != Eigen::Success)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    Eigen::Vector3d fittedNormal_world = eigenSolver.eigenvectors().col(0);

    if (!fittedNormal_world.allFinite() || fittedNormal_world.norm() < 1e-8)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    fittedNormal_world.normalize();

    /*!
     * Preserve the previous normal direction to prevent the plane equation
     * from changing sign between updates.
     */
    const double previousNormalNorm =
        previousEquation_world_in.head<3>().norm();

    if (std::isfinite(previousNormalNorm) && previousNormalNorm > 1e-8 &&
        fittedNormal_world.dot(previousEquation_world_in.head<3>() /
                               previousNormalNorm) < 0.0)
    {
        fittedNormal_world *= -1.0;
    }

    planeEquation_world_out.head<3>() = fittedNormal_world;
    planeEquation_world_out(3) =
        -fittedNormal_world.dot(fittedCentroid_world_m);
    planeCentroid_world_m_out = fittedCentroid_world_m;
    finitePointCount_out      = validPointCount;
    wasFitFound_out           = true;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
