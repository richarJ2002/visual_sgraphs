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
 * @file            isPointOnForeignSideOfWallFace.cc
 *
 * @brief           Implements isPointOnForeignSideOfWallFace(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <cmath>
#include <optional>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    isPointOnForeignSideOfWallFace(geometric::Plane      *p_wall_in,
                                   const Eigen::Vector3d &point_world_m_in,
                                   bool                  &isOnForeignSide_out)
{
    isOnForeignSide_out = false;
    bool wallIsBad{};
    if (p_wall_in != nullptr &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_wall_in == nullptr || wallIsBad)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The face's identity: the camera position it was first observed from.
     * Only the side of a physical surface turned toward a camera can be
     * seen, so this fixes which of the wall's two faces this plane is -- and
     * therefore which room it bounds -- for the plane's whole lifetime. */
    std::optional<Eigen::Vector3d> observationOrigin_world_m{};
    if (p_wall_in->getObservationOrigin_world(observationOrigin_world_m) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!observationOrigin_world_m.has_value() ||
        !observationOrigin_world_m->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D wallGetGlobalEquation{};
    if (p_wall_in->getGlobalEquation(wallGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d planeEquation_world = wallGetGlobalEquation.coeffs();
    const double    normalNorm          = planeEquation_world.head<3>().norm();
    if (!planeEquation_world.allFinite() || normalNorm <= 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    planeEquation_world /= normalNorm;

    /* Matches the resolvable-side floor already used by the association path
     * (utils::utils::Utils::findMatchingPlane) and
     * Plane::getObservationSideSnapshot(): a position essentially ON the plane
     * does not identify a side. */
    constexpr double minimumResolvableSide_m = 0.10;
    const double     observedSide_m =
        planeEquation_world.head<3>().dot(observationOrigin_world_m.value()) +
        planeEquation_world(3);
    const double pointSide_m =
        planeEquation_world.head<3>().dot(point_world_m_in) +
        planeEquation_world(3);

    /* Opposite sides: the camera that produced this face was on the far side
     * of it from this point, so the face bounds the neighbouring space. */
    isOnForeignSide_out = std::isfinite(observedSide_m) &&
                          std::abs(observedSide_m) >= minimumResolvableSide_m &&
                          std::isfinite(pointSide_m) &&
                          std::abs(pointSide_m) >= minimumResolvableSide_m &&
                          observedSide_m * pointSide_m < 0.0;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
