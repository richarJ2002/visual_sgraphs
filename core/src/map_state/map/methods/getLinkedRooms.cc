/*!
 * @file            getLinkedRooms.cc
 *
 * @brief           Implements Map::getLinkedRooms(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus
    Map::getLinkedRooms(Map                           *p_otherMap_in,
                        std::vector<semantic::Room *> &ownLinkedRooms_out,
                        std::vector<semantic::Room *> &otherLinkedRooms_out)
{
    ownLinkedRooms_out.clear();
    otherLinkedRooms_out.clear();
    if (p_otherMap_in == nullptr || p_otherMap_in == this)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    /* Each map's fields are read under its own lock, never both at once. */
    semantic::Room *p_ownStartingRoom = nullptr;
    semantic::Room *p_ownFinalRoom    = nullptr;
    semantic::Room *p_ownBeyondRoom   = nullptr;
    Map            *p_ownFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(mapMutex);
        p_ownStartingRoom = p_startingRoom;
        p_ownFinalRoom    = p_finalRoom;
        p_ownBeyondRoom   = p_finalRoomBeyondPassage;
        p_ownFollowingMap = p_followingMap;
    }
    semantic::Room *p_otherStartingRoom = nullptr;
    semantic::Room *p_otherFinalRoom    = nullptr;
    semantic::Room *p_otherBeyondRoom   = nullptr;
    Map            *p_otherFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(p_otherMap_in->mapMutex);
        p_otherStartingRoom = p_otherMap_in->p_startingRoom;
        p_otherFinalRoom    = p_otherMap_in->p_finalRoom;
        p_otherBeyondRoom   = p_otherMap_in->p_finalRoomBeyondPassage;
        p_otherFollowingMap = p_otherMap_in->p_followingMap;
    }

    /* The earlier map's final room (and the room beyond its passage) links
     * with the later map's starting room. */
    const auto appendRoom =
        [](semantic::Room *p_room_in, std::vector<semantic::Room *> &rooms_out)
    {
        if (p_room_in != nullptr)
        {
            rooms_out.push_back(p_room_in);
        }
    };
    if (p_ownFollowingMap == p_otherMap_in)
    {
        appendRoom(p_ownFinalRoom, ownLinkedRooms_out);
        appendRoom(p_ownBeyondRoom, ownLinkedRooms_out);
        appendRoom(p_otherStartingRoom, otherLinkedRooms_out);
    }
    else if (p_otherFollowingMap == this)
    {
        appendRoom(p_ownStartingRoom, ownLinkedRooms_out);
        appendRoom(p_otherFinalRoom, otherLinkedRooms_out);
        appendRoom(p_otherBeyondRoom, otherLinkedRooms_out);
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
