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
 * @file            getClassIdFromPlaneType.cc
 *
 * @brief           Implements Utils::getClassIdFromPlaneType(), declared in
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

UtilsStatus Utils::getClassIdFromPlaneType(
    vs_graphs::core::geometric::Plane::PlaneVariant planeType_in,
    int                                            &classIdFromPlaneType_out)
{
    switch (planeType_in)
    {
    case vs_graphs::core::geometric::Plane::PlaneVariant::GROUND:
    {
        classIdFromPlaneType_out = 0;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case vs_graphs::core::geometric::Plane::PlaneVariant::WALL:
    {
        classIdFromPlaneType_out = 1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case vs_graphs::core::geometric::Plane::PlaneVariant::DOOR:
    {
        classIdFromPlaneType_out = 2;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    case vs_graphs::core::geometric::Plane::PlaneVariant::WINDOW:
    {
        classIdFromPlaneType_out = 3;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    default:
    {
        classIdFromPlaneType_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
