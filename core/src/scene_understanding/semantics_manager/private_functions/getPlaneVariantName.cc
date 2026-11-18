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
 * @file            getPlaneVariantName.cc
 *
 * @brief           Implements getPlaneVariantName(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include "mapping/geometric/Plane/Plane.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    getPlaneVariantName(const geometric::Plane::PlaneVariant planeVariant_in,
                        const char                         *&variantName_out)
{
    switch (planeVariant_in)
    {
    case geometric::Plane::PlaneVariant::WALL:
        variantName_out = "WALL";
        break;
    case geometric::Plane::PlaneVariant::GROUND:
        variantName_out = "GROUND";
        break;
    case geometric::Plane::PlaneVariant::DOOR:
        variantName_out = "DOOR";
        break;
    case geometric::Plane::PlaneVariant::WINDOW:
        variantName_out = "WINDOW";
        break;
    case geometric::Plane::PlaneVariant::CEILING:
        variantName_out = "CEILING";
        break;
    case geometric::Plane::PlaneVariant::UNDEFINED:
    default:
        variantName_out = "UNDEFINED";
        break;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
