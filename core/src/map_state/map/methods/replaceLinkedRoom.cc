/*!
 * @file            replaceLinkedRoom.cc
 *
 * @brief           Implements Map::replaceLinkedRoom(), declared in
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

MapStatus Map::replaceLinkedRoom(semantic::Room *p_fusedRoom_in,
                                 semantic::Room *p_keptRoom_in)
{
    std::unique_lock<std::mutex> lock(mapMutex);
    if (p_startingRoom == p_fusedRoom_in)
    {
        p_startingRoom = p_keptRoom_in;
    }
    if (p_finalRoom == p_fusedRoom_in)
    {
        p_finalRoom = p_keptRoom_in;
    }
    if (p_finalRoomBeyondPassage == p_fusedRoom_in)
    {
        p_finalRoomBeyondPassage = p_keptRoom_in;
    }
    /* Two candidates fused into one room leave a single candidate. */
    if (p_finalRoomBeyondPassage == p_finalRoom)
    {
        p_finalRoomBeyondPassage = nullptr;
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
