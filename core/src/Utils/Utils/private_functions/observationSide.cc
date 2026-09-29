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
 * @file            observationSide.cc
 *
 * @brief           Implements getMedianObservationSide_World_m(),
 *                  declared in Utils/Utils/private_functions.h.
 */

#include "Utils/Utils/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus getMedianObservationSide_World_m(
    geometric::Plane        *p_plane_in,
    const Eigen::Vector4d   &planeEquation_World_in,
    ObservationSideEvidence &medianObservationSide_World_m_out)
{
    if (p_plane_in == nullptr)
    {
        medianObservationSide_World_m_out = {};
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    geometric::Plane::ObservationSideSnapshot snapshot{};
    if (p_plane_in->getObservationSideSnapshot(planeEquation_World_in,
                                               snapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getObservationSideSnapshot cannot fail; continue as before.
    }
    medianObservationSide_World_m_out = {
        snapshot.medianSignedDistance_m,
        snapshot.face ==
            geometric::Plane::ObservationSideSnapshot::Face::AMBIGUOUS};
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
