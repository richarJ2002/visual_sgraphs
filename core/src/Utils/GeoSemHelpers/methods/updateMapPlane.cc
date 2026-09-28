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
    vs_graphs::core::KeyFrame                      *p_keyFrame_inout,
    const g2o::Plane3D                              estimatedPlane_in,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr         p_planeCloud_in,
    int                                             planeId_in,
    vs_graphs::core::geometric::Plane::PlaneVariant semanticType_in,
    double                                          confidence_in)
{
    // Find the matched plane among all planes of the map
    vs_graphs::core::geometric::Plane *p_currentPlane =
        p_atlas_in->getPlaneById(planeId_in);

    // the observation of the plane
    vs_graphs::core::geometric::Plane::Observation observation;

    // the observation of the plane equation
    observation.localPlane = estimatedPlane_in;

    // the observation of the plane point cloud (measurement)
    Eigen::Matrix4d pointPlaneConstraintMatrix;
    pointPlaneConstraintMatrix.setZero();
    if (types::SystemParams::getParams()->optimization.planePoint.enabled)
    {
        for (auto &point : p_planeCloud_in->points)
        {
            Eigen::Vector4d pointVector;
            pointVector << point.x, point.y, point.z, 1;
            pointPlaneConstraintMatrix += pointVector *
                                          pointVector.transpose() *
                                          (static_cast<int>(point.a) / 255.0);
        }
    }
    observation.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

    // the semantic class of the observation
    observation.semanticType = semanticType_in;

    // the aggregated confidence of the plane
    observation.confidence = confidence_in;
    p_currentPlane->addObservation(p_keyFrame_inout, observation);

    // Add the plane to the list of planes in the current KeyFrame
    p_keyFrame_inout->addMapPlane(p_currentPlane);

    // transform the plane cloud to the global frame
    pcl::transformPointCloud(
        *p_planeCloud_in,
        *p_planeCloud_in,
        p_keyFrame_inout->getPoseInverse().matrix().cast<float>());

    /* Update the point cloud of the mapped plane */
    if (!p_planeCloud_in->empty())
    {
        p_currentPlane->setMapClouds(p_planeCloud_in);

        /*!
         * Refit the mapped global equation from the complete accumulated point
         * cloud.
         *
         * @note        Without refitting, the point cloud and centroid change
         *              but the original plane equation becomes stale.
         */
        refitMappedPlaneFromCloud(p_currentPlane);
    }

    if (types::SystemParams::getParams()->optimization.planeMapPoint.enabled)
    {
        for (const auto &mapPoint : p_keyFrame_inout->getMapPoints())
            if (p_currentPlane->isPointinPlaneCloud(
                    mapPoint->getWorldPos().cast<double>()))
                p_currentPlane->setMapPoints(mapPoint);
    }
}

} // namespace core
} // namespace vs_graphs
