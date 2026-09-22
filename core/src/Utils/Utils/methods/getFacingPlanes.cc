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
 * @file            getFacingPlanes.cc
 *
 * @brief           Implements Utils::getFacingPlanes(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                      vs_graphs::core::geometric::Plane *>>
    Utils::getFacingPlanes(
        const std::vector<vs_graphs::core::geometric::Plane *> &planes_in)
{
    // Variables
    vs_graphs::core::types::SystemParams *p_sysParams =
        vs_graphs::core::types::SystemParams::getParams();
    std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                          vs_graphs::core::geometric::Plane *>>
           facingPlanes;
    double minValidSpace = p_sysParams->roomSeg.minWallDistanceThresh;

    // Loop through all the planes_in
    for (size_t idx1 = 0; idx1 < planes_in.size(); ++idx1)
    {
        vs_graphs::core::geometric::Plane *plane1 = planes_in[idx1];
        for (size_t idx2 = idx1 + 1; idx2 < planes_in.size(); ++idx2)
        {
            // Variables
            vs_graphs::core::geometric::Plane *plane2 = planes_in[idx2];
            // Check if the planes_in are facing each other
            bool isFacing = Utils::arePlanesFacingEachOther(plane1, plane2);
            if (isFacing)
            {
                if (Utils::arePlanesApartEnough(plane1, plane2, minValidSpace))
                    facingPlanes.push_back(std::make_pair(plane1, plane2));
            }
        }
    }
    return facingPlanes;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
