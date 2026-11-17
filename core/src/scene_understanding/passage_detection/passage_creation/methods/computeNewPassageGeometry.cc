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
 * @file            computeNewPassageGeometry.cc
 *
 * @brief           Implements SemanticsManager::computeNewPassageGeometry(),
 * declared in SemanticsManagerClass.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::computeNewPassageGeometry(
    vs_graphs::core::geometric::Plane *p_doorPlane_in,
    vs_graphs::core::geometric::Plane *p_wallPlane_in,
    const Eigen::Vector3d             &passageCentroid_world_m_in,
    double                            &passageWidth_m_out,
    double                            &passageHeight_m_out,
    Eigen::Vector3d                   &passageCentroid_world_m_out,
    g2o::Plane3D                      &passageEquation_out,
    bool                              &isGeometryValid_out)
{
    isGeometryValid_out = false;

    /* Extract the max door height and width */
    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    passageWidth_m_out             = p_params2->semSeg.maxDoorWidth;
    types::SystemParams *p_params3 = nullptr;
    if (types::SystemParams::getParams(p_params3) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    passageHeight_m_out = p_params3->semSeg.maxDoorHeight;

    /* Initialize variables to define the passage */
    passageEquation_out = g2o::Plane3D();

    if (p_doorPlane_in != nullptr)
    {
        /* Confirm the door plane is not bad */
        bool doorPlaneIsBad{};
        if (p_doorPlane_in->isBad(doorPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (doorPlaneIsBad)
        {
            int doorPlaneGetId{};
            if (p_doorPlane_in->getId(doorPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cerr << "[GeoSemHelper] Cannot create passage: door plane "
                      << doorPlaneGetId << " is bad." << std::endl;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        /* Extract centroid and plane equation */
        Eigen::Vector3d doorPlaneGetCentroid{};
        if (p_doorPlane_in->getCentroid(doorPlaneGetCentroid) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        passageCentroid_world_m_out = doorPlaneGetCentroid;
        g2o::Plane3D doorPlaneGetGlobalEquation{};
        if (p_doorPlane_in->getGlobalEquation(doorPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        passageEquation_out = doorPlaneGetGlobalEquation;

        /* Extract point cloud of door */
        geometric::Plane::GeometrySnapshot doorPlaneGetGeometrySnapshot{};
        if (p_doorPlane_in->getGeometrySnapshot(doorPlaneGetGeometrySnapshot) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_doorCloud =
            doorPlaneGetGeometrySnapshot.supportCloud;

        /* Use measured door dimensions when a valid point cloud if available */
        if (p_doorCloud != nullptr && !p_doorCloud->empty())
        {
            /* Compute the dimensions of the door */
            std::pair<double, double> measuredDimensions{};
            if (utils::utils::Utils::computePlaneWidthHeight(
                    p_doorCloud,
                    measuredDimensions) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computePlaneWidthHeight returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /* Extract the dimensions of the door from the tuple */
            const double measuredWidth  = measuredDimensions.first;
            const double measuredHeight = measuredDimensions.second;

            /* Clip the width dimension of the door */
            if (std::isfinite(measuredWidth) && measuredWidth > 0.0)
            {
                types::SystemParams *p_params4 = nullptr;
                if (types::SystemParams::getParams(p_params4) !=
                    types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParams returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageWidth_m_out = std::min(
                    measuredWidth,
                    static_cast<double>(p_params4->semSeg.maxDoorWidth));
            }

            /* Clip the height dimension of the door */
            if (std::isfinite(measuredHeight) && measuredHeight > 0.0)
            {
                types::SystemParams *p_params5 = nullptr;
                if (types::SystemParams::getParams(p_params5) !=
                    types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParams returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                passageHeight_m_out = std::min(
                    measuredHeight,
                    static_cast<double>(p_params5->semSeg.maxDoorHeight));
            }
        }
    }
    else
    {
        /*!
         * No door exists. This is an open passage detected from the camera
         * trajectory crossing a wall plane.
         */
        passageCentroid_world_m_out = passageCentroid_world_m_in;

        /* Extract the plane coefficients of the wall */
        g2o::Plane3D wallPlaneGetGlobalEquation{};
        if (p_wallPlane_in->getGlobalEquation(wallPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d wallEquation = wallPlaneGetGlobalEquation.coeffs();

        /* Extract the magnitude of the norm from the coefficients */
        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            int wallPlaneGetId3{};
            if (p_wallPlane_in->getId(wallPlaneGetId3) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cerr << "[GeoSemHelper] Cannot create open passage: wall "
                      << wallPlaneGetId3 << " has an invalid plane equation."
                      << std::endl;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        /* Normalize the norm vector */
        const Eigen::Vector3d normalizedNormal =
            wallEquation.head<3>() / normalNorm;

        /* Create parrallel plane, passing through detected wall centroid */
        Eigen::Vector4d correctionEquation;
        correctionEquation.head<3>() = normalizedNormal;
        correctionEquation(3) =
            -normalizedNormal.dot(passageCentroid_world_m_out.cast<double>());

        /* Create an equation for the passage plane */
        passageEquation_out = g2o::Plane3D(correctionEquation);
    }

    /* Confirm the centroid of the plane is valid */
    if (!passageCentroid_world_m_out.allFinite())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: invalid centroid."
                  << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    isGeometryValid_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
