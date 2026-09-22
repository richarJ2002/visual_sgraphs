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

vs_graphs::core::geometric::Plane::PlaneVariant
    Utils::getPlaneTypeFromClassId(int classId_in)
{
    switch (classId_in)
    {
    case 0:
        return vs_graphs::core::geometric::Plane::PlaneVariant::GROUND;
    case 1:
        return vs_graphs::core::geometric::Plane::PlaneVariant::WALL;
    case 2:
        return vs_graphs::core::geometric::Plane::PlaneVariant::DOOR;
    case 3:
        return vs_graphs::core::geometric::Plane::PlaneVariant::WINDOW;
    default:
        return vs_graphs::core::geometric::Plane::PlaneVariant::UNDEFINED;
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
