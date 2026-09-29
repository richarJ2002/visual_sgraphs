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

#include "SemanticsManager.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

bool SemanticsManager::isWallFaceForeignToRoom(semantic::Room   *p_room_in,
                                               geometric::Plane *p_wall_in)
{
    bool room_inIsBad{};
    if (!(p_room_in == nullptr) &&
        p_room_in->isBad(room_inIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    bool wallIsBad{};
    if (!(p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    if (p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr ||
        wallIsBad)
    {
        return false;
    }

    /* The face's identity: the camera position it was first observed from.
     * Only the side of a physical surface turned toward a camera can be
     * seen, so this fixes which of the wall's two faces this plane is -- and
     * therefore which room it bounds -- for the plane's whole lifetime. */
    std::optional<Eigen::Vector3d> observationOrigin_World_m{};
    if (p_wall_in->getObservationOrigin_World(observationOrigin_World_m) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getObservationOrigin_World cannot fail; continue as before.
    }

    if (!observationOrigin_World_m.has_value() ||
        !observationOrigin_World_m->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        return false;
    }

    g2o::Plane3D wallGetGlobalEquation{};
    if (p_wall_in->getGlobalEquation(wallGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getGlobalEquation cannot fail; continue as before.
    }
    Eigen::Vector4d equation_World = wallGetGlobalEquation.coeffs();
    const double    normalNorm     = equation_World.head<3>().norm();

    if (!equation_World.allFinite() || normalNorm <= 1e-8)
    {
        return false;
    }

    equation_World /= normalNorm;

    const double observedSide_m =
        equation_World.head<3>().dot(observationOrigin_World_m.value()) +
        equation_World(3);
    Eigen::Vector3d room_inCentroid{};
    if (p_room_in->getCentroid(room_inCentroid) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getCentroid cannot fail; continue as before.
    }
    const double roomSide_m =
        equation_World.head<3>().dot(room_inCentroid.cast<double>()) +
        equation_World(3);

    if (!std::isfinite(observedSide_m) || !std::isfinite(roomSide_m))
    {
        return false;
    }

    /* Matches the resolvable-side floor already used by the association path
     * (utils::utils::Utils::associatePlanes) and
     * Plane::getObservationSideSnapshot(): a position essentially ON the plane
     * does not identify a side. */
    constexpr double minimumResolvableSide_m = 0.10;

    if (std::abs(observedSide_m) < minimumResolvableSide_m ||
        std::abs(roomSide_m) < minimumResolvableSide_m)
    {
        return false;
    }

    /* Opposite sides: the camera that produced this face was on the far side
     * of it from this room, so this is the neighbouring room's face. The
     * room's own face of the same physical wall is a separate plane, which
     * it has evidently not observed yet. */
    return observedSide_m * roomSide_m < 0.0;
}

} // namespace core
} // namespace vs_graphs
