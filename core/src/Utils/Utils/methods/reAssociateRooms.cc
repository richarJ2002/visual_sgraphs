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
 * @file            reAssociateRooms.cc
 *
 * @brief           Implements Utils::reAssociateRooms(), declared in
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

UtilsStatus Utils::reAssociateRooms(Atlas *p_atlas_in)
{
    /*!
     * Re-run the targeted provisional-room consolidation pass.
     *
     * @note        This function does not merge confirmed rooms. It only allows
     *              a confirmed room or corridor to absorb redundant,
     *              single-wall provisional structural elements.
     */
    const std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas_in->getAllRooms();

    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        /* Skip invalid structural elements */
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        /*!
         * Only confirmed rooms may absorb provisional
         * structural elements.
         */
        semantic::Room::RoomVariant roomVariant{};
        if (p_room->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        const bool isConfirmedRoom =
            roomVariant == vs_graphs::core::semantic::Room::RoomVariant::ROOM;

        if (!isConfirmedRoom)
        {
            continue;
        }

        /* Require more than one valid wall before allowing consolidation */
        std::size_t validWallCount = 0;

        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

        for (vs_graphs::core::geometric::Plane *p_wall : roomWalls)
        {
            bool wallIsBad{};
            if ((p_wall != nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_wall != nullptr && !wallIsBad)
            {
                validWallCount++;
            }
        }

        if (validWallCount < 2)
        {
            continue;
        }

        /*!
         * Consolidate only redundant single-wall provisional structural
         * elements whose wall already belongs to this confirmed room.
         */
        if (Utils::consolidateProvisionalRooms(p_room, p_atlas_in) !=
            UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // consolidateProvisionalRooms cannot fail; continue as before.
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
