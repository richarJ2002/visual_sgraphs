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
 * @file            private_functions.h
 *
 * @brief           Declares the helpers of the plane fusion stage.
 *                  Package-internal: not included by any public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_PLANE_FUSION_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_PLANE_FUSION_PRIVATE_FUNCTIONS_H

#include "mapping/geometry_helpers/objects/TwinFaceCheckEnum.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Name of a twin-face check as written in the twin_face
 *                  SG_PIPELINE records.
 *
 * @param[in]       twinFaceCheck_in
 *                  Deciding check of a wall face pair.
 *
 * @return          Static upper-case name; "UNKNOWN" for a value outside the
 *                  enumeration.
 */
[[nodiscard]] const char *
    getTwinFaceCheckName(utils::utils::TwinFaceCheck twinFaceCheck_in);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_PLANE_FUSION_PRIVATE_FUNCTIONS_H */
