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
 * @file            createMapPlane.cc
 *
 * @brief           Implements GeoSemHelpers::createMapPlane(), declared in
 *                  GeoSemHelpers.h.
 */

#include "GeoSemHelpers.h"
#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::createMapPlane(
    Atlas                                          *p_atlas_inout,
    vs_graphs::core::KeyFrame                      *p_keyFrame_inout,
    const g2o::Plane3D                              estimatedPlane_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr   p_planeCloud_in,
    vs_graphs::core::geometric::Plane             *&p_mapPlane_out,
    vs_graphs::core::geometric::Plane::PlaneVariant semanticType_in,
    double                                          confidence_in)
{
    vs_graphs::core::Map *p_currentMap = nullptr;
    if (p_atlas_inout->getCurrentMap(p_currentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_currentMap == nullptr)
    {
        p_mapPlane_out = nullptr;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    vs_graphs::core::geometric::Plane *p_newMapPlane =
        new vs_graphs::core::geometric::Plane();
    if (p_newMapPlane->setColor() !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setColor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPlane->setLocalEquation(estimatedPlane_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setLocalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPlane->setMap(p_currentMap) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int currentMapPlaneId{};
    if (p_currentMap->reservePlaneId(currentMapPlaneId) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reservePlaneId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPlane->setId(currentMapPlaneId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    p_newMapPlane->p_refKeyFrame = p_keyFrame_inout;

    /* Stamp which face of the physical surface this is, from the camera that
     * observed it. Only the side turned toward a camera can ever be seen, so
     * this position permanently identifies the face -- and therefore which
     * room it bounds -- without any later re-derivation from observation
     * history (see Plane::observationOrigin_World_m). */
    if (p_keyFrame_inout != nullptr)
    {
        Eigen::Vector3f keyFrameCameraCenter{};
        if (p_keyFrame_inout->getCameraCenter(keyFrameCameraCenter) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d observationOrigin_World_m =
            keyFrameCameraCenter.cast<double>();

        if (observationOrigin_World_m.allFinite())
        {
            if (p_newMapPlane->setObservationOrigin_World(
                    observationOrigin_World_m) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setObservationOrigin_World returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_params->optimization.planePoint.enabled)
    {
        /* Iterate through points in point cloud */
        for (pcl::PointXYZRGBA &point : p_planeCloud_in->points)
        {
            /* Create the homogeneous coordinate point vector object */
            Eigen::Vector4d pointVector;

            /* Load the point into the vector */
            pointVector << point.x, point.y, point.z, 1;

            /* Accumulate the square matrix from poitns */
            pointPlaneConstraintMatrix += pointVector * pointVector.transpose();
        }
    }

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATION OF PLANE
     * ---------------------------------------------------------------------- */

    /* Init observation struct to store information about the plane */
    vs_graphs::core::geometric::Plane::Observation observation;

    /* Store the result of the plane constaint matrix */
    observation.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

    /* Store the observes semantic type of the plane */
    observation.semanticType = semanticType_in;

    /* Store the aggregatede confidence of the plane */
    observation.confidence = confidence_in;

    /* Store the equation of the plane with respect to the camera */
    observation.localPlane = estimatedPlane_in;

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATIONS OF NEW PLANE
     * ---------------------------------------------------------------------- */

    /* Add observation and keyframe to plane */
    if (p_newMapPlane->addObservation(p_keyFrame_inout, observation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addObservation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Set the plane type */
    if (p_newMapPlane->setPlaneType(semanticType_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Get the global equation of the plane */
    g2o::Plane3D globalEquation_World{};
    Sophus::SE3f keyFramePoseInverse{};
    if (p_keyFrame_inout->getPoseInverse(keyFramePoseInverse) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (utils::utils::Utils::applyPoseToPlane(
            keyFramePoseInverse.matrix().cast<double>(),
            estimatedPlane_in,
            globalEquation_World) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: applyPoseToPlane returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Set the global equation of the plane in the map world plane */
    if (p_newMapPlane->setGlobalEquation(globalEquation_World) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Transform the plane cloud to the global frame */
    Sophus::SE3f keyFramePoseInverse2{};
    if (p_keyFrame_inout->getPoseInverse(keyFramePoseInverse2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    pcl::transformPointCloud(*p_planeCloud_in,
                             *p_planeCloud_in,
                             keyFramePoseInverse2.matrix().cast<float>());

    /* Fill the plane with the pointcloud */
    if (!p_planeCloud_in->points.empty())
    {
        /* Add the point clouds to the new map plane */
        if (p_newMapPlane->replaceMapClouds(p_planeCloud_in) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: replaceMapClouds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool wasPlaneRefit{};
        if (refitMappedPlaneFromCloud(p_newMapPlane, wasPlaneRefit) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: refitMappedPlaneFromCloud returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* ---------------------------------------------------------------------- *
     * ASSOCIATE ORB MAP POINTS WITH THE PLANE
     * ---------------------------------------------------------------------- */

    /*!
     * Associate sparse ORB map landmarks whose world positions are supported by
     * the observed finite plane cloud. These associations may later be used to
     * construct map-point-to-plane constraints during graph optimisation.
     */
    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_params2->optimization.planeMapPoint.enabled)
    {
        /* Iterate through the orb points (expressed in global frame) */
        std::set<MapPoint *> keyFrameMapPoints{};
        if (p_keyFrame_inout->getMapPoints(keyFrameMapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (MapPoint *const &mapPoint : keyFrameMapPoints)
        {
            /* If the orb feature is within the plane, set as map point */
            bool            newMapPlaneIsPointinPlaneCloud{};
            Eigen::Vector3f mapPointWorldPos{};
            if (mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_newMapPlane->isPointinPlaneCloud(
                    mapPointWorldPos.cast<double>(),
                    newMapPlaneIsPointinPlaneCloud) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: isPointinPlaneCloud returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (newMapPlaneIsPointinPlaneCloud)
            {
                if (p_newMapPlane->setMapPoints(mapPoint) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setMapPoints returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    /* Add the plane to the keyframe */
    if (p_keyFrame_inout->addMapPlane(p_newMapPlane) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapPlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Add the palne to the current map */
    if (p_atlas_inout->addMapPlane(p_newMapPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapPlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    p_mapPlane_out = p_newMapPlane;
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
