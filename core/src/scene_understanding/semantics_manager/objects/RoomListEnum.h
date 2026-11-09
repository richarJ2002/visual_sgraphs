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
 * @file            RoomListEnum.h
 *
 * @brief           Declares which of a scene graph's two room lists a room
 *                  belongs to.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_LIST_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_LIST_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           List of a scene graph that holds a room. Values are fixed
 *                  and never reordered.
 */
enum class RoomList : std::uint8_t
{
    /*!
     * @brief           Rooms confirmed by the semantics manager.
     */
    ROOM_LIST_DETECTED = 0U,

    /*!
     * @brief           Rooms still being proven (provisional or prospective).
     */
    ROOM_LIST_CANDIDATE = 1U
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_LIST_ENUM_H */
