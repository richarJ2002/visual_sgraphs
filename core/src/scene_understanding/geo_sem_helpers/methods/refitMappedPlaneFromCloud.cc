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

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <rclcpp/logging.hpp>

namespace vs_graphs::core::utils::geo_sem_helpers
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

    if (!geometrySnapshot.has_value() ||
        geometrySnapshot->supportCloud == nullptr)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    Eigen::Vector3d fittedCentroid_world_m = Eigen::Vector3d::Zero();
    Eigen::Vector4d fittedEquation_world   = Eigen::Vector4d::Zero();
    std::size_t     validPointCount        = 0U;
    bool            wasFitFound            = false;
    if (geometric::Plane::fitEquationToCloud(
            *geometrySnapshot->supportCloud,
            geometrySnapshot->planeEquation_world,
            fittedCentroid_world_m,
            fittedEquation_world,
            validPointCount,
            wasFitFound) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: fitEquationToCloud returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!wasFitFound)
    {
        wasPlaneRefit_out = false;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Publish the complete fitted geometry and recompute finite bounds once. */
    bool planeWasRefitPublished{};
    if (p_plane_inout->completeMapCloudRefit(geometrySnapshot->cloudGeneration,
                                             fittedCentroid_world_m,
                                             g2o::Plane3D(fittedEquation_world),
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

} /* namespace vs_graphs::core::utils::geo_sem_helpers */
