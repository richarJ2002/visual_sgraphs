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
 * @file            isWallFaceForeignToRoom.cc
 *
 * @brief           Implements SemanticsManager::isWallFaceForeignToRoom(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::isWallFaceForeignToRoom(semantic::Room   *p_room_in,
                                              geometric::Plane *p_wall_in,
                                              bool &isWallFaceForeignToRoom_out)
{
    bool room_inIsBad{};
    if (!(p_room_in == nullptr) &&
        p_room_in->isBad(room_inIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool wallIsBad{};
    if (!(p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr ||
        wallIsBad)
    {
        isWallFaceForeignToRoom_out = false;
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
        isWallFaceForeignToRoom_out = false;
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
    Eigen::Vector4d equation_world = wallGetGlobalEquation.coeffs();
    const double    normalNorm     = equation_world.head<3>().norm();

    if (!equation_world.allFinite() || normalNorm <= 1e-8)
    {
        isWallFaceForeignToRoom_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    equation_world /= normalNorm;

    const double observedSide_m =
        equation_world.head<3>().dot(observationOrigin_world_m.value()) +
        equation_world(3);
    Eigen::Vector3d room_inCentroid{};
    if (p_room_in->getCentroid(room_inCentroid) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const double roomSide_m =
        equation_world.head<3>().dot(room_inCentroid.cast<double>()) +
        equation_world(3);

    if (!std::isfinite(observedSide_m) || !std::isfinite(roomSide_m))
    {
        isWallFaceForeignToRoom_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Matches the resolvable-side floor already used by the association path
     * (utils::utils::Utils::associatePlanes) and
     * Plane::getObservationSideSnapshot(): a position essentially ON the plane
     * does not identify a side. */
    constexpr double minimumResolvableSide_m = 0.10;

    if (std::abs(observedSide_m) < minimumResolvableSide_m ||
        std::abs(roomSide_m) < minimumResolvableSide_m)
    {
        isWallFaceForeignToRoom_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Opposite sides: the camera that produced this face was on the far side
     * of it from this room, so this is the neighbouring room's face. The
     * room's own face of the same physical wall is a separate plane, which
     * it has evidently not observed yet. */
    isWallFaceForeignToRoom_out = observedSide_m * roomSide_m < 0.0;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
