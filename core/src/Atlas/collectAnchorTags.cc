/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Atlas.h"

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

std::set<std::string> collectAnchorTags(Map *p_oldMap_in, Map *p_currentMap_in)
{
    std::set<std::string> anchorTags;
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        return anchorTags;
    }
    std::set<std::string> currentTags;
    for (semantic::Room *p_room : p_currentMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() && p_room->hasRoomTag() &&
            !p_room->getRoomTag().empty())
        {
            currentTags.insert(p_room->getRoomTag());
        }
    }
    for (semantic::Room *p_room : p_oldMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() && p_room->hasRoomTag() &&
            !p_room->getRoomTag().empty() &&
            currentTags.count(p_room->getRoomTag()) > 0U)
        {
            anchorTags.insert(p_room->getRoomTag());
        }
    }
    return anchorTags;
}

} // namespace core
} // namespace vs_graphs
