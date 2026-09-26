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

#include "GeoSemHelpers.h"
#include "Utils/Utils/objects/Utils.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

void GeoSemHelpers::updateMapPlane(
    Atlas                                          *p_atlas_in,
    vs_graphs::core::KeyFrame                      *pKF,
    const g2o::Plane3D                              estimatedPlane,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr         planeCloud,
    int                                             planeId,
    vs_graphs::core::geometric::Plane::PlaneVariant semanticType,
    double                                          confidence)
{
    // Find the matched plane among all planes of the map
    vs_graphs::core::geometric::Plane *currentPlane =
        p_atlas_in->getPlaneById(planeId);

    // the observation of the plane
    vs_graphs::core::geometric::Plane::Observation obs;

    // the observation of the plane equation
    obs.localPlane = estimatedPlane;

    // the observation of the plane point cloud (measurement)
    Eigen::Matrix4d pointPlaneConstraintMatrix;
    pointPlaneConstraintMatrix.setZero();
    if (types::SystemParams::getParams()->optimization.planePoint.enabled)
    {
        for (auto &point : planeCloud->points)
        {
            Eigen::Vector4d pointVec;
            pointVec << point.x, point.y, point.z, 1;
            pointPlaneConstraintMatrix += pointVec * pointVec.transpose() *
                                          (static_cast<int>(point.a) / 255.0);
        }
    }
    obs.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

    // the semantic class of the observation
    obs.semanticType = semanticType;

    // the aggregated confidence of the plane
    obs.confidence = confidence;
    currentPlane->addObservation(pKF, obs);

    // Add the plane to the list of planes in the current KeyFrame
    pKF->addMapPlane(currentPlane);

    // transform the plane cloud to the global frame
    pcl::transformPointCloud(*planeCloud,
                             *planeCloud,
                             pKF->getPoseInverse().matrix().cast<float>());

    /* Update the point cloud of the mapped plane */
    if (!planeCloud->empty())
    {
        currentPlane->setMapClouds(planeCloud);

        /*!
         * Refit the mapped global equation from the complete accumulated point
         * cloud.
         *
         * @note        Without refitting, the point cloud and centroid change
         *              but the original plane equation becomes stale.
         */
        refitMappedPlaneFromCloud(currentPlane);
    }

    if (types::SystemParams::getParams()->optimization.planeMapPoint.enabled)
    {
        for (const auto &mapPoint : pKF->getMapPoints())
            if (currentPlane->isPointinPlaneCloud(
                    mapPoint->getWorldPos().cast<double>()))
                currentPlane->setMapPoints(mapPoint);
    }
}

} // namespace core
} // namespace vs_graphs
