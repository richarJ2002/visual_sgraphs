/*!
 * @file            adoptMergedMapLinks.cc
 *
 * @brief           Implements Map::adoptMergedMapLinks(), declared in
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

MapStatus Map::adoptMergedMapLinks(Map *p_absorbedMap_in)
{
    if (p_absorbedMap_in == nullptr || p_absorbedMap_in == this)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    semantic::Room *p_absorbedStartingRoom = nullptr;
    semantic::Room *p_absorbedFinalRoom    = nullptr;
    semantic::Room *p_absorbedBeyondRoom   = nullptr;
    Map            *p_absorbedFollowingMap = nullptr;
    {
        std::unique_lock<std::mutex> lock(p_absorbedMap_in->mapMutex);
        p_absorbedStartingRoom = p_absorbedMap_in->p_startingRoom;
        p_absorbedFinalRoom    = p_absorbedMap_in->p_finalRoom;
        p_absorbedBeyondRoom   = p_absorbedMap_in->p_finalRoomBeyondPassage;
        p_absorbedFollowingMap = p_absorbedMap_in->p_followingMap;
    }

    /* The merged map starts where the earlier map started and ends where the
     * later map ended, with the later map's anchor candidates. Maps that are
     * not neighbours in the chain keep this map's links. */
    std::unique_lock<std::mutex> lock(mapMutex);
    if (p_absorbedFollowingMap == this)
    {
        p_startingRoom = p_absorbedStartingRoom;
    }
    else if (p_followingMap == p_absorbedMap_in)
    {
        p_finalRoom              = p_absorbedFinalRoom;
        p_finalRoomBeyondPassage = p_absorbedBeyondRoom;
        p_followingMap           = p_absorbedFollowingMap;
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
