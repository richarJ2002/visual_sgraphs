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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::updateMapPlane(
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
    if (p_currentPlane->addObservation(p_keyFrame_inout, observation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addObservation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Add the plane to the list of planes in the current KeyFrame
    if (p_keyFrame_inout->addMapPlane(p_currentPlane) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapPlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // transform the plane cloud to the global frame
    Sophus::SE3f keyFramePoseInverse{};
    if (p_keyFrame_inout->getPoseInverse(keyFramePoseInverse) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    pcl::transformPointCloud(*p_planeCloud_in,
                             *p_planeCloud_in,
                             keyFramePoseInverse.matrix().cast<float>());

    /* Update the point cloud of the mapped plane */
    if (!p_planeCloud_in->empty())
    {
        if (p_currentPlane->setMapClouds(p_planeCloud_in) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMapClouds returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Refit the mapped global equation from the complete accumulated point
         * cloud.
         *
         * @note        Without refitting, the point cloud and centroid change
         *              but the original plane equation becomes stale.
         */
        bool wasPlaneRefit{};
        if (refitMappedPlaneFromCloud(p_currentPlane, wasPlaneRefit) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: refitMappedPlaneFromCloud returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

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
        std::set<MapPoint *> keyFrameMapPoints{};
        if (p_keyFrame_inout->getMapPoints(keyFrameMapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (const auto &mapPoint : keyFrameMapPoints)
        {
            bool            currentPlaneIsPointinPlaneCloud{};
            Eigen::Vector3f mapPointWorldPos{};
            if (mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentPlane->isPointinPlaneCloud(
                    mapPointWorldPos.cast<double>(),
                    currentPlaneIsPointinPlaneCloud) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: isPointinPlaneCloud returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (currentPlaneIsPointinPlaneCloud)
            {
                if (p_currentPlane->setMapPoints(mapPoint) !=
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

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
