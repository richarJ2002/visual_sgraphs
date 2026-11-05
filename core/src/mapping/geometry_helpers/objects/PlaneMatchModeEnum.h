/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            PlaneMatchModeEnum.h
 *
 * @brief           Declares which plane relation Utils::findMatchingPlane()
 *                  tests.
 */

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHMODEENUM_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHMODEENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

/*!
 * @brief           Relation between an observed plane and a mapped plane that
 *                  counts as a match. Values are fixed and never reordered.
 */
enum class PlaneMatchMode : std::uint8_t
{
    /*!
     * @brief           Both planes are the same face of one physical surface:
     *                  aligned normals, coplanar, overlapping or adjacent
     *                  support, and seen from the same side.
     */
    SAME_FACE = 0U,

    /*!
     * @brief           Both planes are the two opposite faces of one wall:
     *                  parallel, no more than a wall thickness apart, seen
     *                  from opposite sides and with overlapping footprints.
     */
    OPPOSITE_FACE = 1U
};

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHMODEENUM_H */
