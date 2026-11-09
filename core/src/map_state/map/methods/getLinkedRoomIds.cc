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

/*!
 * @file            getLinkedRoomIds.cc
 *
 * @brief           Implements Map::getLinkedRoomIds(), declared in
 * map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <algorithm>
#include <mutex>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::getLinkedRoomIds(Map              *p_otherMap_in,
                                std::vector<int> &ownLinkedRoomIds_out,
                                std::vector<int> &otherLinkedRoomIds_out)
{
    ownLinkedRoomIds_out.clear();
    otherLinkedRoomIds_out.clear();
    if (p_otherMap_in == nullptr || p_otherMap_in == this)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    /* Each map's fields are read under its own lock, never both at once. */
    int  ownStartingRoomId = NO_LINKED_ROOM_ID;
    int  ownFinalRoomId    = NO_LINKED_ROOM_ID;
    int  ownBeyondRoomId   = NO_LINKED_ROOM_ID;
    Map *p_ownFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(mapMutex);
        ownStartingRoomId = startingRoom_id;
        ownFinalRoomId    = finalRoom_id;
        ownBeyondRoomId   = finalRoomBeyondPassage_id;
        p_ownFollowingMap = p_followingMap;
    }
    int  otherStartingRoomId = NO_LINKED_ROOM_ID;
    int  otherFinalRoomId    = NO_LINKED_ROOM_ID;
    int  otherBeyondRoomId   = NO_LINKED_ROOM_ID;
    Map *p_otherFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(p_otherMap_in->mapMutex);
        otherStartingRoomId = p_otherMap_in->startingRoom_id;
        otherFinalRoomId    = p_otherMap_in->finalRoom_id;
        otherBeyondRoomId   = p_otherMap_in->finalRoomBeyondPassage_id;
        p_otherFollowingMap = p_otherMap_in->p_followingMap;
    }

    /* The earlier map's final room (and the room beyond its passage) links
     * with the later map's starting room. */
    if (p_ownFollowingMap == p_otherMap_in)
    {
        ownLinkedRoomIds_out   = {ownFinalRoomId, ownBeyondRoomId};
        otherLinkedRoomIds_out = {otherStartingRoomId};
    }
    else if (p_otherFollowingMap == this)
    {
        ownLinkedRoomIds_out   = {ownStartingRoomId};
        otherLinkedRoomIds_out = {otherFinalRoomId, otherBeyondRoomId};
    }

    /* A link that was never set is no candidate. */
    ownLinkedRoomIds_out.erase(std::remove(ownLinkedRoomIds_out.begin(),
                                           ownLinkedRoomIds_out.end(),
                                           NO_LINKED_ROOM_ID),
                               ownLinkedRoomIds_out.end());
    otherLinkedRoomIds_out.erase(std::remove(otherLinkedRoomIds_out.begin(),
                                             otherLinkedRoomIds_out.end(),
                                             NO_LINKED_ROOM_ID),
                                 otherLinkedRoomIds_out.end());
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
