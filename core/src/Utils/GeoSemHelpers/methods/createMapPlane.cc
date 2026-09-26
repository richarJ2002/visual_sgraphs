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
#include <cmath>

namespace vs_graphs
{
namespace core
{

vs_graphs::core::geometric::Plane *GeoSemHelpers::createMapPlane(
    Atlas                                          *p_atlas_inout,
    vs_graphs::core::KeyFrame                      *pKF,
    const g2o::Plane3D                              estimatedPlane,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr   planeCloud,
    vs_graphs::core::geometric::Plane::PlaneVariant semanticType,
    double                                          confidence)
{
    vs_graphs::core::Map *p_currentMap = p_atlas_inout->getCurrentMap();

    if (p_currentMap == nullptr)
    {
        return nullptr;
    }

    vs_graphs::core::geometric::Plane *newMapPlane =
        new vs_graphs::core::geometric::Plane();
    newMapPlane->setColor();
    newMapPlane->setLocalEquation(estimatedPlane);
    newMapPlane->setMap(p_currentMap);
    newMapPlane->setId(p_currentMap->reservePlaneId());
    newMapPlane->p_refKeyFrame = pKF;

    /* Stamp which face of the physical surface this is, from the camera that
     * observed it. Only the side turned toward a camera can ever be seen, so
     * this position permanently identifies the face -- and therefore which
     * room it bounds -- without any later re-derivation from observation
     * history (see Plane::observationOrigin_World_m). */
    if (pKF != nullptr)
    {
        const Eigen::Vector3d observationOrigin_World_m =
            pKF->getCameraCenter().cast<double>();

        if (observationOrigin_World_m.allFinite())
        {
            newMapPlane->setObservationOrigin_World(observationOrigin_World_m);
        }
    }

    /* ---------------------------------------------------------------------- *
     * CONSTRUCT POINT PLANE CONSTRAINT MATRIX
     * ---------------------------------------------------------------------- */

    /* Init variable which is used to construct plane constraint matrix  */
    Eigen::Matrix4d pointPlaneConstraintMatrix;

    /* Clear variable */
    pointPlaneConstraintMatrix.setZero();

    /* If plane optimization enabled */
    if (types::SystemParams::getParams()->optimization.planePoint.enabled)
    {
        /* Iterate through points in point cloud */
        for (auto &point : planeCloud->points)
        {
            /* Create the homogeneous coordinate point vector object */
            Eigen::Vector4d pointVec;

            /* Load the point into the vector */
            pointVec << point.x, point.y, point.z, 1;

            /* Accumulate the square matrix from poitns */
            pointPlaneConstraintMatrix += pointVec * pointVec.transpose();
        }
    }

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATION OF PLANE
     * ---------------------------------------------------------------------- */

    /* Init observation struct to store information about the plane */
    vs_graphs::core::geometric::Plane::Observation obs;

    /* Store the result of the plane constaint matrix */
    obs.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

    /* Store the observes semantic type of the plane */
    obs.semanticType = semanticType;

    /* Store the aggregatede confidence of the plane */
    obs.confidence = confidence;

    /* Store the equation of the plane with respect to the camera */
    obs.localPlane = estimatedPlane;

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATIONS OF NEW PLANE
     * ---------------------------------------------------------------------- */

    /* Add observation and keyframe to plane */
    newMapPlane->addObservation(pKF, obs);

    /* Set the plane type */
    newMapPlane->setPlaneType(semanticType);

    /* Get the global equation of the plane */
    g2o::Plane3D globalEquation_World = utils::utils::Utils::applyPoseToPlane(
        pKF->getPoseInverse().matrix().cast<double>(),
        estimatedPlane);

    /* Set the global equation of the plane in the map world plane */
    newMapPlane->setGlobalEquation(globalEquation_World);

    /* Transform the plane cloud to the global frame */
    pcl::transformPointCloud(*planeCloud,
                             *planeCloud,
                             pKF->getPoseInverse().matrix().cast<float>());

    /* Fill the plane with the pointcloud */
    if (!planeCloud->points.empty())
    {
        /* Add the point clouds to the new map plane */
        newMapPlane->replaceMapClouds(planeCloud);
        refitMappedPlaneFromCloud(newMapPlane);
    }

    /* ---------------------------------------------------------------------- *
     * ASSOCIATE ORB MAP POINTS WITH THE PLANE
     * ---------------------------------------------------------------------- */

    /*!
     * Associate sparse ORB map landmarks whose world positions are supported by
     * the observed finite plane cloud. These associations may later be used to
     * construct map-point-to-plane constraints during graph optimisation.
     */
    if (types::SystemParams::getParams()->optimization.planeMapPoint.enabled)
    {
        /* Iterate through the orb points (expressed in global frame) */
        for (const auto &mapPoint : pKF->getMapPoints())
        {
            /* If the orb feature is within the plane, set as map point */
            if (newMapPlane->isPointinPlaneCloud(
                    mapPoint->getWorldPos().cast<double>()))
            {
                newMapPlane->setMapPoints(mapPoint);
            }
        }
    }

    /* Add the plane to the keyframe */
    pKF->addMapPlane(newMapPlane);

    /* Add the palne to the current map */
    p_atlas_inout->addMapPlane(newMapPlane);

    return newMapPlane;
}

} // namespace core
} // namespace vs_graphs
