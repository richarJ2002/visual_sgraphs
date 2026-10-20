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
 * @file            MapPointStatusEnum.h
 *
 * @brief           Declares the status returned by every MapPoint operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_MAP_POINT_OBJECTS_MAP_POINT_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_MAP_POINT_OBJECTS_MAP_POINT_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace map_point
{

/*!
 * @brief           Result of a MapPoint operation. Values are fixed and never
 *                  reordered.
 */
enum class MapPointStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    MAP_POINT_STATUS_SUCCESS = 0U
};

} /* namespace map_point */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_MAP_POINT_OBJECTS_MAP_POINT_STATUS_ENUM_H */
