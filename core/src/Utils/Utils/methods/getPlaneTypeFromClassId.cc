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
 * @file            getPlaneTypeFromClassId.cc
 *
 * @brief           Implements Utils::getPlaneTypeFromClassId(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::getPlaneTypeFromClassId(
    int                                              classId_in,
    vs_graphs::core::geometric::Plane::PlaneVariant &planeTypeFromClassId_out)
{
    switch (classId_in)
    {
    case 0:
    {
        planeTypeFromClassId_out =
            vs_graphs::core::geometric::Plane::PlaneVariant::GROUND;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case 1:
    {
        planeTypeFromClassId_out =
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case 2:
    {
        planeTypeFromClassId_out =
            vs_graphs::core::geometric::Plane::PlaneVariant::DOOR;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case 3:
    {
        planeTypeFromClassId_out =
            vs_graphs::core::geometric::Plane::PlaneVariant::WINDOW;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    default:
    {
        planeTypeFromClassId_out =
            vs_graphs::core::geometric::Plane::PlaneVariant::UNDEFINED;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
